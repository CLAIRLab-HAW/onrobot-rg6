// rg6_control_sim: Simulations-Zwilling von rg6_control OHNE UR-Hardware.
//
// Bietet dieselbe ROS-Schnittstelle wie der Realtreiber:
//   Services rg6_control/{open,close} (Trigger),
//            rg6_control/grip (rg6_msgs/Grip),
//            rg6_control/set_force_preset, rg6_control/set_tool_power (SetBool)
//   Action   rg6_gripper_controller/gripper_cmd (control_msgs/GripperCommand)
//   Topic    rg6/bridge_state (std_msgs/String, flaches JSON) -- DERSELBE
//            Name und dasselbe JSON wie rg6_grip_bridge am echten Roboter
//   Topic    rg6/state (rg6_msgs/GripperState) -- der alte Kanal, nur noch
//            fuer Betrachter, die ihn schon abonnieren; der plan_server liest
//            seit 2026-08-19 bridge_state
// und publiziert zusaetzlich das Treibergelenk als joint_states (ersetzt den
// frueheren rg6_joint_state_broadcaster_sim) -> Modell animiert in RViz/Foxglove,
// MoveIt-Integration ist damit komplett ohne Roboter testbar.  Die fuenf
// Folgegelenke haengen im rg6_v2 per <mimic> am Treiber und werden von
// robot_state_publisher und move_group selbst abgeleitet.
//
// Bewegungsmodell: Weite faehrt mit konstanter Geschwindigkeit auf die Zielweite.
// Mit sim_object_width_m > 0 stoppt das Schliessen an der Objektweite ->
// grip_detected=true (wie das Tool-DI0-Signal der echten Hardware).
//
// Analoger Rueckkanal: width_raw traegt an der Hardware die AI2-Spannung des
// Tool-Anschlusses, aus der der Realtreiber die Weite ueberhaupt erst gewinnt.
// Der Sim rechnet sie aus seiner Weite zurueck (analog_model.hpp) statt NaN zu
// melden -- sonst widerspricht er sich selbst: er setzt tool_data_received=true,
// behauptet also, die Tool-Daten seien da, und liefert dann keine.  Der
// Verfuegbarkeits-Guard der plan-bridge liest genau dieses Feld und feuerte
// darum im Container bei JEDEM Greifer-Kommando, obwohl der Sim die Ziele
// korrekt anfuhr.  Ein Guard, der immer feuert, ist Rauschen.
//
// force_raw bleibt bewusst NaN: das ist der Motorstrom (AI3), fuer den es hier
// kein ehrliches Modell gibt -- die Kraft haengt an Backengeometrie, Objekt und
// Preset.  Kein Verbraucher liest ihn; erfundene Zahlen waeren schlechter als
// ein offenes "nicht gemessen".
//
// Grenze: das macht den Greifer im Container benutzbar, mehr nicht.  Die echten
// RG6-Pathologien bleiben unabgedeckt (AI2 haengt bei zuen Backen auf 10 V, ein
// injizierter grip reisst ExternalControl ab).  Ein Erfolg aus diesem Node ist
// ueber das source-Feld in /twin/result ohnehin als Nicht-Hardware-Wahrheit
// gekennzeichnet.

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
#include "rg6_control/analog_model.hpp"
#include "rg6_control/finger_kinematics.hpp"
#include "rg6_msgs/msg/gripper_state.hpp"
#include "rg6_msgs/srv/grip.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using GripperCommand = control_msgs::action::GripperCommand;
using GoalHandleGripperCommand = rclcpp_action::ServerGoalHandle<GripperCommand>;

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

using rg6_control::analog::map_clamped;
}  // namespace

class RG6ControlSimNode : public rclcpp::Node
{
public:
  RG6ControlSimNode()
  : Node("rg6_control_node")  // gleicher Node-Name wie real -> identische Graph-Sicht
  {
    declare_parameter<double>("width_open_m", 0.160);
    declare_parameter<double>("width_closed_m", 0.0);
    declare_parameter<double>("sim_speed_m_s", 0.16);       // voller Hub in ~1 s
    declare_parameter<double>("sim_object_width_m", 0.0);   // 0 = kein Objekt
    declare_parameter<double>("grip_default_force_n", 60.0);
    declare_parameter<double>("motion_timeout_s", 10.0);
    declare_parameter<double>("state_rate", 20.0);
    declare_parameter<double>("action_goal_angle_tol", 0.08);
    declare_parameter<std::string>("joint_prefix", "rg6_");

    // Analog-Kalibrierung des Rueckkanals -- 1:1 die Defaults des Realtreibers
    // (analog_model.hpp).  Wer sie am Geraet nachjustiert, zieht sie hier mit.
    declare_parameter<double>("width_in_open", rg6_control::analog::kWidthInOpenV);
    declare_parameter<double>("width_in_closed", rg6_control::analog::kWidthInClosedV);
    // AI2 bei weggenommener Toolspannung.  Muss unter dead_input_threshold
    // (0,2 V) des Realtreibers liegen, sonst laesst sich der Totzustand im
    // Container nicht mehr provozieren -- das ist die Bedingung, unter der ein
    // simulierter Analogwert ueberhaupt vertretbar ist.
    declare_parameter<double>("sim_width_in_dead", rg6_control::analog::kSimDeadInputV);

    width_ = get_parameter("width_open_m").as_double();
    target_width_ = width_;

    blocking_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    state_pub_ = create_publisher<rg6_msgs::msg::GripperState>("rg6/state", rclcpp::QoS(10));
    // Derselbe Zustand als flaches JSON, unter dem Namen, den auch
    // rg6_grip_bridge am Roboter benutzt.  Ohne ihn liest der plan_server im
    // Container ins Leere -- genau der Fehler, der auf 'real' seit dem
    // rg6_control-Ruhestand bestand und am 2026-08-19 aufgefallen ist.
    bridge_state_pub_ = create_publisher<std_msgs::msg::String>(
      "rg6/bridge_state", rclcpp::QoS(10));
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
    // NUR das Treibergelenk -- genau wie rg6_grip_bridge am echten Roboter.
    //
    // Bis 2026-08-19 standen hier sechs Gelenke, fuenf davon mit den Namen des
    // alten Greifermodells (left_inner_knuckle_joint & Co.).  Die gibt es im
    // rg6_v2 nicht mehr; seine Folgegelenke heissen finger_joint_mirror,
    // gripper_finger_{1,2}_truss_arm_joint und _finger_tip_joint, und sie
    // haengen alle per <mimic> am Treiber -- robot_state_publisher und
    // move_group leiten sie selbst ab, ein zweiter Absender ist ueberfluessig.
    //
    // Und er war nicht bloss ueberfluessig:  ein RobotState, der einen der
    // toten Namen traegt, bringt move_group ueber
    // RobotModel::getVariableIndex zum ABSTURZ (std::terminate, SIGABRT -- am
    // 2026-08-19 zweimal reproduziert).  Jeder Verbraucher, der joint_states
    // liest und in eine MoveIt-Anfrage zurueckgibt, war damit eine
    // Abschussrampe.
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = get_clock()->now();
    msg.name = {get_parameter("joint_prefix").as_string() + "finger_joint"};
    msg.position = {angle_from_width(width_)};
    joint_pub_->publish(msg);
  }

  // Weite <-> Fingergelenk kommen aus der Tabelle, die aus dem GENERIERTEN
  // URDF des rg6_v2-Modells stammt (finger_kinematics.hpp, erzeugt von
  // tools/derive_finger_kinematics.py).
  //
  // Bis 2026-08-19 stand hier ``rg6_control::linkage`` -- die Kurbelschwinge
  // des ALTEN, selbstgebauten Greifermodells.  Sie hat zwei Dinge falsch
  // gerechnet, und beide waren am laufenden Container messbar:  ihr q=0 liegt
  // bei 93,4 mm statt bei den 153,2 mm des neuen Modells, und sie gibt fuer
  // die ganz offene Hand -0,93766 rad zurueck -- ein Wert AUSSERHALB der
  // Gelenkgrenzen des rg6_v2 (0,0 bis 1,25478).  Der Mock stellte den Greifer
  // damit in eine Stellung, die es nicht gibt.
  static double angle_from_width(double width_m)
  {
    return rg6_control::finger_kinematics::angle_from_width(width_m);
  }

  static double width_from_angle(double angle)
  {
    return rg6_control::finger_kinematics::width_from_angle(angle);
  }

  // Weite -> AI2-Spannung: der Weg, den nur der Sim geht.  Die Hardware misst
  // AI2 und rechnet vorwaerts (rg6_control.cpp: width_from_raw), der Sim kennt
  // die Weite und muss den Messwert daraus erzeugen.
  double raw_from_width(double width_m) const
  {
    return rg6_control::analog::raw_from_width(
      width_m,
      get_parameter("width_closed_m").as_double(),
      get_parameter("width_open_m").as_double(),
      get_parameter("width_in_closed").as_double(),
      get_parameter("width_in_open").as_double());
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
    // Stromlos faellt der Analogeingang unter die Totschwelle -- genau das Signal,
    // an dem die plan-bridge "Kommando kann nicht wirken" erkennt.  Bestromt
    // traegt er ueber den ganzen Hub die zurueckgerechnete Weite.
    msg.width_raw = tool_power_on_ ?
      raw_from_width(width_) : get_parameter("sim_width_in_dead").as_double();
    msg.force_raw = kNaN;  // Motorstrom AI3: kein ehrliches Modell, s. Dateikopf
    msg.busy = moving_;
    msg.grip_detected = grip_detected_;
    msg.io_states_received = true;
    msg.tool_data_received = true;
    msg.tool_power_on = tool_power_on_;
    msg.high_force_preset = high_force_preset_;
    msg.last_command = last_command_;
    state_pub_->publish(msg);
    publish_bridge_state_locked();
  }

  // Die Felder von rg6_grip_bridge.status_payload(), Zeichen fuer Zeichen --
  // wer hier etwas umbenennt, macht den Container zum Sonderfall.
  //
  // "status" und "safety_failed" kommen am Geraet aus rg_get_status /
  // rg_get_safety_failed.  Der Sim hat keine Stoerung zu melden und schreibt
  // deshalb 0 / false; das ist keine Behauptung ueber Hardware, sondern die
  // ehrliche Aussage "in dieser Simulation gibt es nichts zu stoeren".
  void publish_bridge_state_locked()
  {
    const char * cmd = "NONE";
    switch (last_command_) {
      case rg6_msgs::msg::GripperState::COMMAND_OPEN: cmd = "OPEN"; break;
      case rg6_msgs::msg::GripperState::COMMAND_CLOSE: cmd = "CLOSE"; break;
      case rg6_msgs::msg::GripperState::COMMAND_GRIP: cmd = "GRIP"; break;
      default: cmd = "NONE"; break;
    }
    std::ostringstream os;
    os << std::fixed << std::setprecision(6)
       << "{\"width_m\": " << width_
       << ", \"busy\": " << (moving_ ? "true" : "false")
       << ", \"grip_detected\": " << (grip_detected_ ? "true" : "false")
       << ", \"status\": 0"
       << ", \"safety_failed\": false"
       << ", \"last_command\": \"" << cmd << "\"}";
    std_msgs::msg::String out;
    out.data = os.str();
    bridge_state_pub_->publish(out);
  }

  rclcpp::CallbackGroup::SharedPtr blocking_cb_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr open_service_, close_service_;
  rclcpp::Service<rg6_msgs::srv::Grip>::SharedPtr grip_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr force_preset_service_, tool_power_service_;
  rclcpp_action::Server<GripperCommand>::SharedPtr action_server_;
  rclcpp::Publisher<rg6_msgs::msg::GripperState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bridge_state_pub_;
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
