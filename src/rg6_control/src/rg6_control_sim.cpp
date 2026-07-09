// rg6_control_sim: Simulations-Zwilling von rg6_control OHNE UR-Hardware.
//
// Bietet dieselbe ROS-Schnittstelle wie der Realtreiber:
//   Services rg6_control/{open,close,open_gripper,close_gripper} (Trigger),
//            rg6_control/grip (rg6_msgs/Grip),
//            rg6_control/set_force_preset, rg6_control/set_tool_power (SetBool)
//   Action   rg6_gripper_controller/gripper_cmd (control_msgs/GripperCommand)
//   Topic    rg6/state (rg6_msgs/GripperState)
// und publiziert zusaetzlich die 6 Greifergelenke als joint_states (ersetzt den
// frueheren rg6_joint_state_broadcaster_sim) -> Modell animiert in RViz/Foxglove,
// MoveIt-Integration ist damit komplett ohne Roboter testbar.
//
// Bewegungsmodell: Weite faehrt mit konstanter Geschwindigkeit auf die Zielweite.
// Mit sim_object_width_m > 0 stoppt das Schliessen an der Objektweite ->
// grip_detected=true (wie das Tool-DI0-Signal der echten Hardware).

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "control_msgs/action/gripper_command.hpp"
#include "rg6_msgs/msg/gripper_state.hpp"
#include "rg6_msgs/srv/grip.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using GripperCommand = control_msgs::action::GripperCommand;
using GoalHandleGripperCommand = rclcpp_action::ServerGoalHandle<GripperCommand>;

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

double map_clamped(double x, double x0, double x1, double y0, double y1)
{
  if (std::abs(x1 - x0) < 1e-12) {
    return y0;
  }
  const double y = y0 + (x - x0) * (y1 - y0) / (x1 - x0);
  return std::clamp(y, std::min(y0, y1), std::max(y0, y1));
}
}  // namespace

class RG6ControlSimNode : public rclcpp::Node
{
public:
  RG6ControlSimNode()
  : Node("rg6_control_node")  // gleicher Node-Name wie real -> identische Graph-Sicht
  {
    declare_parameter<double>("width_open_m", 0.160);
    declare_parameter<double>("width_closed_m", 0.0);
    declare_parameter<double>("angle_open", 0.0);
    declare_parameter<double>("angle_closed", 0.6);
    declare_parameter<double>("sim_speed_m_s", 0.16);       // voller Hub in ~1 s
    declare_parameter<double>("sim_object_width_m", 0.0);   // 0 = kein Objekt
    declare_parameter<double>("grip_default_force_n", 60.0);
    declare_parameter<double>("motion_timeout_s", 10.0);
    declare_parameter<double>("state_rate", 20.0);
    declare_parameter<double>("action_goal_angle_tol", 0.08);
    declare_parameter<std::string>("joint_prefix", "rg6_");

    width_ = get_parameter("width_open_m").as_double();
    target_width_ = width_;

    blocking_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    state_pub_ = create_publisher<rg6_msgs::msg::GripperState>("rg6/state", rclcpp::QoS(10));
    // 'joint_states' relativ -> per Launch-Remap auf das gewuenschte Topic legen
    // (a200-0553: manipulators/endeffectors/joint_states).
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(10));

    tick_timer_ = create_wall_timer(20ms, [this]() { tick(); });
    const double state_rate = get_parameter("state_rate").as_double();
    state_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, state_rate)),
      [this]() { publish_state(); });

    auto make_trigger = [this](const std::string & name, bool close_cmd) {
        return create_service<std_srvs::srv::Trigger>(
          name,
          [this, close_cmd](
            const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            handle_open_close(close_cmd, response);
          },
          rclcpp::ServicesQoS(), blocking_cb_group_);
      };
    open_service_ = make_trigger("rg6_control/open", false);
    close_service_ = make_trigger("rg6_control/close", true);
    open_alias_service_ = make_trigger("rg6_control/open_gripper", false);
    close_alias_service_ = make_trigger("rg6_control/close_gripper", true);

    grip_service_ = create_service<rg6_msgs::srv::Grip>(
      "rg6_control/grip",
      std::bind(&RG6ControlSimNode::handle_grip, this,
        std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), blocking_cb_group_);

    force_preset_service_ = create_service<std_srvs::srv::SetBool>(
      "rg6_control/set_force_preset",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        {
          std::lock_guard<std::mutex> lk(mutex_);
          high_force_preset_ = request->data;
        }
        response->success = true;
        response->message = request->data ? "Kraft-Preset: high (sim)" : "Kraft-Preset: low (sim)";
      },
      rclcpp::ServicesQoS(), blocking_cb_group_);

    tool_power_service_ = create_service<std_srvs::srv::SetBool>(
      "rg6_control/set_tool_power",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        {
          std::lock_guard<std::mutex> lk(mutex_);
          tool_power_on_ = request->data;
        }
        response->success = true;
        response->message = request->data ? "Tool-Spannung 24V an (sim)" : "Tool-Spannung aus (sim)";
      },
      rclcpp::ServicesQoS(), blocking_cb_group_);

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
      "RG6-SIM bereit (Weite %.0f mm offen). sim_object_width_m=%.3f",
      width_ * 1000.0, get_parameter("sim_object_width_m").as_double());
  }

private:
  // --------- Bewegungsmodell (50-Hz-Tick) ---------------------------------
  void tick()
  {
    const double dt = 0.02;
    const double speed = get_parameter("sim_speed_m_s").as_double();
    const double object_w = get_parameter("sim_object_width_m").as_double();
    std::lock_guard<std::mutex> lk(mutex_);
    if (!tool_power_on_) {
      moving_ = false;
      return;  // ohne Toolspannung bewegt sich nichts
    }
    double effective_target = target_width_;
    // Objekt im Weg? Schliessen stoppt an der Objektweite -> grip detected.
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
    // Alle 6 Gelenke explizit (robot_state_publisher wertet <mimic> nicht aus;
    // Faktoren wie in rg6_joint_state_broadcaster).
    const double t = map_clamped(
      width_,
      get_parameter("width_closed_m").as_double(),
      get_parameter("width_open_m").as_double(),
      get_parameter("angle_closed").as_double(),
      get_parameter("angle_open").as_double());
    const auto prefix = get_parameter("joint_prefix").as_string();
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = get_clock()->now();
    msg.name = {
      prefix + "finger_joint",
      prefix + "left_inner_knuckle_joint",
      prefix + "left_inner_finger_joint",
      prefix + "right_outer_knuckle_joint",
      prefix + "right_inner_knuckle_joint",
      prefix + "right_inner_finger_joint",
    };
    msg.position = {t, -t, t, -t, -t, t};
    joint_pub_->publish(msg);
  }

  double angle_from_width(double width_m) const
  {
    return map_clamped(
      width_m,
      get_parameter("width_closed_m").as_double(),
      get_parameter("width_open_m").as_double(),
      get_parameter("angle_closed").as_double(),
      get_parameter("angle_open").as_double());
  }

  double width_from_angle(double angle) const
  {
    return map_clamped(
      angle,
      get_parameter("angle_closed").as_double(),
      get_parameter("angle_open").as_double(),
      get_parameter("width_closed_m").as_double(),
      get_parameter("width_open_m").as_double());
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
        result.settled = tool_power_on_;  // ohne Spannung: nie "fertig"
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

  // --------- Services ------------------------------------------------------
  void handle_open_close(bool close_cmd, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::unique_lock<std::mutex> motion_lock(motion_mutex_, std::try_to_lock);
    if (!motion_lock.owns_lock()) {
      response->success = false;
      response->message = "RG6-SIM: Bewegung laeuft bereits";
      return;
    }
    {
      std::lock_guard<std::mutex> lk(mutex_);
      last_command_ = close_cmd ? rg6_msgs::msg::GripperState::COMMAND_CLOSE :
        rg6_msgs::msg::GripperState::COMMAND_OPEN;
    }
    const double target = close_cmd ? get_parameter("width_closed_m").as_double() :
      get_parameter("width_open_m").as_double();
    const auto result = start_motion_and_wait(target);
    response->success = result.settled;
    std::ostringstream msg;
    msg << (result.settled ? "Gripper motion settled (OK)" : "Gripper motion did not settle (TIMEOUT)");
    if (result.grip_detected) {
      msg << ", grip detected";
    }
    msg << ", width=" << result.final_width_m << " m [sim]";
    response->message = msg.str();
  }

  void handle_grip(
    const std::shared_ptr<rg6_msgs::srv::Grip::Request> request,
    std::shared_ptr<rg6_msgs::srv::Grip::Response> response)
  {
    std::unique_lock<std::mutex> motion_lock(motion_mutex_, std::try_to_lock);
    if (!motion_lock.owns_lock()) {
      response->success = false;
      response->message = "RG6-SIM: Bewegung laeuft bereits";
      return;
    }
    {
      std::lock_guard<std::mutex> lk(mutex_);
      last_command_ = rg6_msgs::msg::GripperState::COMMAND_GRIP;
    }
    if (!request->wait) {
      std::lock_guard<std::mutex> lk(mutex_);
      target_width_ = request->width;
      grip_detected_ = false;
      moving_ = true;
      response->success = true;
      response->final_width = kNaN;
      response->message = "grip-Kommando abgesetzt (sim, wait=false)";
      return;
    }
    const auto result = start_motion_and_wait(request->width);
    response->success = result.settled;
    response->grip_detected = result.grip_detected;
    response->final_width = result.final_width_m;
    response->message = result.settled ?
      (result.grip_detected ? "Grip OK (Objekt erkannt) [sim]" : "Grip OK (Zielweite erreicht) [sim]") :
      "Grip nicht abgeschlossen (TIMEOUT) [sim]";
  }

  // --------- GripperCommand-Action -----------------------------------------
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
      last_command_ = rg6_msgs::msg::GripperState::COMMAND_GRIP;
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

  // --------- Zustands-Publisher --------------------------------------------
  void publish_state()
  {
    rg6_msgs::msg::GripperState msg;
    msg.header.stamp = get_clock()->now();
    std::lock_guard<std::mutex> lk(mutex_);
    msg.width = width_;
    msg.width_raw = kNaN;
    msg.force_raw = kNaN;
    msg.busy = moving_;
    msg.grip_detected = grip_detected_;
    msg.io_states_received = true;
    msg.tool_data_received = true;
    msg.tool_power_on = tool_power_on_;
    msg.high_force_preset = high_force_preset_;
    msg.last_command = last_command_;
    state_pub_->publish(msg);
  }

  rclcpp::CallbackGroup::SharedPtr blocking_cb_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr open_service_, close_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr open_alias_service_, close_alias_service_;
  rclcpp::Service<rg6_msgs::srv::Grip>::SharedPtr grip_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr force_preset_service_, tool_power_service_;
  rclcpp_action::Server<GripperCommand>::SharedPtr action_server_;
  rclcpp::Publisher<rg6_msgs::msg::GripperState>::SharedPtr state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_, state_timer_;

  std::mutex mutex_;
  double width_{0.16};
  double target_width_{0.16};
  bool moving_{false};
  bool grip_detected_{false};
  bool tool_power_on_{true};  // sim: Greifer sofort "bestromt"
  bool high_force_preset_{false};
  uint8_t last_command_{rg6_msgs::msg::GripperState::COMMAND_NONE};

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
