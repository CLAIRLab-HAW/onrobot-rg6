// rg6_control_sim: the simulation twin of the gripper WITHOUT hardware.
//
// AUTHORITATIVE is the surface of rg6_grip_bridge, the node that commands the
// RG6 on the real robot over XML-RPC at the OnRobot URCap.  What the mock
// reproduces of it is exactly that and no more:
//   action   rg6_gripper_controller/gripper_cmd (control_msgs/GripperCommand)
//   topic    rg6/bridge_state plus the driver joint on joint_states
//
// A mock that reproduces a surface existing nowhere is a template for code
// written against something that does not exist.
//
// The driver joint on joint_states animates the model in RViz/Foxglove, which
// makes the MoveIt integration testable entirely without a robot.  The five
// dependent joints hang on the driver in the rg6_v2 via <mimic> and are derived
// by robot_state_publisher and move_group themselves.
//
// Motion model: the width travels towards the target width at a constant speed.
// With sim_object_width_m > 0 the closing stops at the object width ->
// grip_detected=true (like the tool DI0 signal of the real hardware).
//
// The limit: this makes the gripper usable in the container, no more.  The real
// RG6 pathologies stay uncovered (AI2 sticks at 10 V on closed jaws, an injected
// grip tears ExternalControl off).  A success out of this node is marked as a
// non-hardware truth by the source field in /twin/result anyway.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "control_msgs/action/gripper_command.hpp"
#include "rg6_control/finger_kinematics.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using GripperCommand = control_msgs::action::GripperCommand;
using GoalHandleGripperCommand = rclcpp_action::ServerGoalHandle<GripperCommand>;

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

}  // namespace

class RG6ControlSimNode : public rclcpp::Node
{
public:
  RG6ControlSimNode()
  : Node("rg6_control_node")  // the same node name as the real one -> an identical graph view
  {
    declare_parameter<double>("width_open_m", 0.160);
    declare_parameter<double>("width_closed_m", 0.0);
    declare_parameter<double>("sim_speed_m_s", 0.16);       // the full stroke in ~1 s
    declare_parameter<double>("sim_object_width_m", 0.0);   // 0 = no object
    declare_parameter<double>("grip_default_force_n", 60.0);
    declare_parameter<double>("motion_timeout_s", 10.0);
    declare_parameter<double>("state_rate", 20.0);
    declare_parameter<double>("action_goal_angle_tol", 0.08);
    declare_parameter<std::string>("joint_prefix", "rg6_");


    width_ = get_parameter("width_open_m").as_double();
    target_width_ = width_;

    blocking_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // The state as flat JSON, under the name rg6_grip_bridge uses on the robot
    // as well.  Without it the plan_server in the container reads into the void
    // -- the gripper state is then unanswerable on both rungs.
    bridge_state_pub_ = create_publisher<std_msgs::msg::String>(
      "rg6/bridge_state", rclcpp::QoS(10));
    // 'joint_states' is relative -> put it on the wanted topic by a launch remap
    // (a200-0553: manipulators/endeffectors/joint_states).
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(10));

    tick_timer_ = create_wall_timer(20ms, [this]() { tick(); });
    const double state_rate = get_parameter("state_rate").as_double();
    state_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, state_rate)),
      [this]() { publish_bridge_state(); });

    action_server_ = rclcpp_action::create_server<GripperCommand>(
      this, "rg6_gripper_controller/gripper_cmd",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const GripperCommand::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandleGripperCommand>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandleGripperCommand> goal_handle) {
        std::thread{[this, goal_handle]() { action_execute(goal_handle); }}.detach();
      },
      rcl_action_server_get_default_options(), blocking_cb_group_);

    RCLCPP_INFO(get_logger(),
      "RG6 SIM ready (width %.0f mm open). sim_object_width_m=%.3f",
      width_ * 1000.0, get_parameter("sim_object_width_m").as_double());
  }

private:
  // --------- motion model (50 Hz tick) -------------------------------------
  void tick()
  {
    const double dt = 0.02;
    const double speed = get_parameter("sim_speed_m_s").as_double();
    const double object_w = get_parameter("sim_object_width_m").as_double();
    std::lock_guard<std::mutex> lk(mutex_);
    double effective_target = target_width_;
    // An object in the way? The closing stops at the object width -> grip detected.
    if (object_w > 0.0 && effective_target < object_w && width_ >= object_w) {
      effective_target = object_w;
    }
    const double delta = effective_target - width_;
    if (std::abs(delta) <= speed * dt) {
      width_ = effective_target;
      if (moving_) {
        moving_ = false;
        grip_detected_ = object_w > 0.0 && target_width_ < object_w &&
          std::abs(width_ - object_w) < 1e-6;
      }
    } else {
      width_ += (delta > 0 ? 1.0 : -1.0) * speed * dt;
      moving_ = true;
    }
    publish_joints_locked();
  }

  void publish_joints_locked()
  {
    // ONLY the driver joint -- exactly like rg6_grip_bridge on the real robot.
    //
    // The dependent joints of the rg6_v2 (finger_joint_mirror,
    // gripper_finger_{1,2}_truss_arm_joint and _finger_tip_joint) hang on the
    // driver via <mimic>; robot_state_publisher and move_group derive them
    // themselves, a second sender is superfluous.
    //
    // It would also be dangerous:  a RobotState carrying a joint name the model
    // does not know CRASHES move_group through RobotModel::getVariableIndex
    // (std::terminate, SIGABRT -- staged and reproduced twice).  Every consumer
    // that reads joint_states and hands it back into a MoveIt request would be a
    // launch pad for that.
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = get_clock()->now();
    msg.name = {get_parameter("joint_prefix").as_string() + "finger_joint"};
    msg.position = {angle_from_width(width_)};
    joint_pub_->publish(msg);
  }

  // Width <-> finger joint come from the table derived from the GENERATED URDF
  // of the rg6_v2 model (finger_kinematics.hpp, produced by
  // tools/derive_finger_kinematics.py).
  //
  // The table, not a crank-rocker formula:  a recomputed linkage misses both the
  // width at q=0 (153.2 mm) and the joint limits (0.0 to 1.25478 rad) on the
  // rg6_v2 and thereby puts the gripper into poses that do not exist.
  static double angle_from_width(double width_m)
  {
    return rg6_control::finger_kinematics::angle_from_width(width_m);
  }

  static double width_from_angle(double angle)
  {
    return rg6_control::finger_kinematics::width_from_angle(angle);
  }

  struct MotionResult
  {
    bool settled{false};
    bool grip_detected{false};
    double final_width_m{kNaN};
  };

  MotionResult start_motion_and_wait(
    double target_width_m, const std::function<bool()> & canceled = nullptr)
  {
    {
      std::lock_guard<std::mutex> lk(mutex_);
      target_width_ = std::clamp(
        target_width_m,
        std::min(get_parameter("width_closed_m").as_double(),
          get_parameter("width_open_m").as_double()),
        std::max(get_parameter("width_closed_m").as_double(),
          get_parameter("width_open_m").as_double()));
      grip_detected_ = false;
      moving_ = true;
    }
    const auto timeout = rclcpp::Duration::from_seconds(
      get_parameter("motion_timeout_s").as_double());
    const auto start = now();
    MotionResult result;
    while (rclcpp::ok() && (now() - start) < timeout) {
      if (canceled && canceled()) {
        break;
      }
      std::this_thread::sleep_for(20ms);
      std::lock_guard<std::mutex> lk(mutex_);
      if (!moving_) {
        result.settled = true;
        result.grip_detected = grip_detected_;
        result.final_width_m = width_;
        break;
      }
    }
    if (!result.settled) {
      std::lock_guard<std::mutex> lk(mutex_);
      result.final_width_m = width_;
      result.grip_detected = grip_detected_;
    }
    return result;
  }

  // --------- GripperCommand action -----------------------------------------
  void action_execute(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
  {
    auto result = std::make_shared<GripperCommand::Result>();
    const auto goal = goal_handle->get_goal();

    std::unique_lock<std::mutex> motion_lock(motion_mutex_, std::try_to_lock);
    if (!motion_lock.owns_lock()) {
      {
        std::lock_guard<std::mutex> lk(mutex_);
        result->position = angle_from_width(width_);
      }
      goal_handle->abort(result);
      return;
    }

    {
      std::lock_guard<std::mutex> lk(mutex_);
      last_command_ = "GRIP";
    }

    std::atomic<bool> feedback_running{true};
    std::thread feedback_thread([this, goal_handle, &feedback_running]() {
        while (feedback_running && rclcpp::ok()) {
          auto fb = std::make_shared<GripperCommand::Feedback>();
          {
            std::lock_guard<std::mutex> lk(mutex_);
            fb->position = angle_from_width(width_);
          }
          goal_handle->publish_feedback(fb);
          std::this_thread::sleep_for(100ms);
        }
      });

    const double target_width = width_from_angle(goal->command.position);
    const auto motion = start_motion_and_wait(
      target_width, [goal_handle]() { return goal_handle->is_canceling(); });

    feedback_running = false;
    feedback_thread.join();

    result->position = angle_from_width(
      std::isfinite(motion.final_width_m) ? motion.final_width_m : target_width);
    result->effort = motion.grip_detected ?
      (goal->command.max_effort > 0.0 ? goal->command.max_effort :
      get_parameter("grip_default_force_n").as_double()) : 0.0;
    result->stalled = motion.grip_detected;
    result->reached_goal = motion.settled &&
      std::fabs(result->position - goal->command.position) <=
      get_parameter("action_goal_angle_tol").as_double();

    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
    } else if (motion.settled) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  // --------- state publisher ------------------------------------------------
  // The fields of rg6_grip_bridge.status_payload(), character for character --
  // whoever renames something here makes the container a special case.
  //
  // "status" and "safety_failed" come from rg_get_status / rg_get_safety_failed
  // on the device.  The sim has no fault to report and therefore writes
  // 0 / false; that is not a claim about hardware but the honest statement
  // "in this simulation there is nothing to disturb".
  void publish_bridge_state()
  {
    std::ostringstream os;
    os << std::fixed << std::setprecision(6)
       << "{\"width_m\": " << width_
       << ", \"busy\": " << (moving_ ? "true" : "false")
       << ", \"grip_detected\": " << (grip_detected_ ? "true" : "false")
       << ", \"status\": 0"
       << ", \"safety_failed\": false"
       << ", \"last_command\": \"" << last_command_ << "\"}";
    std_msgs::msg::String out;
    out.data = os.str();
    bridge_state_pub_->publish(out);
  }

  rclcpp::CallbackGroup::SharedPtr blocking_cb_group_;
  rclcpp_action::Server<GripperCommand>::SharedPtr action_server_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bridge_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_, state_timer_;

  std::mutex mutex_;
  double width_{0.16};
  double target_width_{0.16};
  bool moving_{false};
  bool grip_detected_{false};
  const char * last_command_{"NONE"};

  std::mutex motion_mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RG6ControlSimNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
