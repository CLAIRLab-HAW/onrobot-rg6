// rg6_control: Vollausbau-Treiber fuer den OnRobot RG6 am UR-Tool-Anschluss
// (CB3, ohne Compute Box, URCap AUS) auf Basis des io_and_status_controller.
//
// Der RG6 bietet ueber die Tool-Schnittstelle folgende Hardware-Funktionen:
//   Tool-DO0 (ur_msgs-Pin 16): Level-Kommando  0 = auf Preset-Weite oeffnen,
//                                              1 = auf Preset-Weite schliessen
//   Tool-DO1 (ur_msgs-Pin 17): Kraft-Preset-Auswahl (low/high)
//   Tool-DI0 (ur_msgs-Pin 16): grip detected (Objekt gegriffen)
//   Tool-DI1 (ur_msgs-Pin 17): ready (high = Ruhe, low = Bewegung laeuft)
//   Tool-AI2: Greifweite (analog)   Tool-AI3: Kraft-/Stromsignal (analog)
//   24-V-Toolspannung: Stromversorgung des Greifers
//   + 17-Bit-Datenwort auf DO0(Takt)/DO1(Daten, invertiert): Ziel-Weite+Kraft
//     (das Protokoll des OnRobot-URCap; Quelle: community-dokumentiertes
//      RG2/RG6-URScript, python-urx PR#35). Laeuft nur mit URScript-Timing
//      (sync()-getaktet) -> wird hier per urscript_interface INJIZIERT.
//
// ACHTUNG URScript-Injektion (Service 'grip' / Zwischenweiten der Action):
// Sie ersetzt das laufende (ExternalControl-)Programm auf dem Controller.
// rg6_control ruft danach automatisch io_and_status_controller/
// resend_robot_program, damit der Arm wieder ROS-steuerbar ist.
//
// Alle Schwellen/Kalibrierwerte sind LIVE-Parameter (Foxglove/ros2 param set).
// Relative Topic-/Servicenamen -> im UR-Namespace starten
// (z. B. /a200_0553/manipulators).

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
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
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ur_msgs/msg/io_states.hpp"
#include "ur_msgs/msg/tool_data_msg.hpp"
#include "ur_msgs/srv/set_io.hpp"

using namespace std::chrono_literals;
using GripperCommand = control_msgs::action::GripperCommand;
using GoalHandleGripperCommand = rclcpp_action::ServerGoalHandle<GripperCommand>;

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// Lineare Abbildung x aus [x0,x1] auf [y0,y1], geklemmt auf den Zielbereich.
double map_clamped(double x, double x0, double x1, double y0, double y1)
{
  if (std::abs(x1 - x0) < 1e-12) {
    return y0;
  }
  const double y = y0 + (x - x0) * (y1 - y0) / (x1 - x0);
  const double lo = std::min(y0, y1);
  const double hi = std::max(y0, y1);
  return std::clamp(y, lo, hi);
}
}  // namespace

class RG6ControlNode : public rclcpp::Node
{
public:
  RG6ControlNode()
  : Node("rg6_control_node")
  {
    declare_parameters();

    // Blockierende Services/Action in eigener Reentrant-Gruppe, damit die
    // tool_data/io_states-Subscriptions (Default-Gruppe) waehrenddessen vom
    // MultiThreadedExecutor weiterverarbeitet werden.
    blocking_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    io_client_ = create_client<ur_msgs::srv::SetIO>(
      "io_and_status_controller/set_io",
      rclcpp::ServicesQoS(), blocking_cb_group_);
    resend_client_ = create_client<std_srvs::srv::Trigger>(
      "io_and_status_controller/resend_robot_program",
      rclcpp::ServicesQoS(), blocking_cb_group_);

    // Feedback: Analogwerte (Weite/Kraft) + Tool-DIs (busy/grip detected).
    tool_data_sub_ = create_subscription<ur_msgs::msg::ToolDataMsg>(
      "io_and_status_controller/tool_data", rclcpp::SensorDataQoS(),
      [this](const ur_msgs::msg::ToolDataMsg::ConstSharedPtr & msg) {
        std::lock_guard<std::mutex> lk(state_mutex_);
        width_raw_ = msg->analog_input2;
        force_raw_ = msg->analog_input3;
        have_tool_data_ = true;
      });
    io_states_sub_ = create_subscription<ur_msgs::msg::IOStates>(
      "io_and_status_controller/io_states", rclcpp::SensorDataQoS(),
      [this](const ur_msgs::msg::IOStates::ConstSharedPtr & msg) {
        const auto grip_pin = static_cast<uint8_t>(get_parameter("tool_di_grip_detected_pin").as_int());
        const auto ready_pin = static_cast<uint8_t>(get_parameter("tool_di_ready_pin").as_int());
        std::lock_guard<std::mutex> lk(state_mutex_);
        for (const auto & di : msg->digital_in_states) {
          if (di.pin == grip_pin) {
            di_grip_detected_ = di.state;
            have_io_states_ = true;
          } else if (di.pin == ready_pin) {
            di_ready_ = di.state;
            have_io_states_ = true;
          }
        }
      });

    // ExternalControl-Status: erst wenn das ROS-Programm auf dem UR laeuft, erreicht
    // set_io ueberhaupt die Tool-Hardware. Auf die STEIGENDE Flanke (Programm wird
    // aktiv - beim Boot oder wenn der Arm SPAETER eingeschaltet wird) hin bestromen
    // wir das Tool und "primen" den Greifer einmal, damit AI2 gueltig wird (siehe
    // on_external_control_ready). So braucht das spaete Einschalten keinen manuellen
    // set_tool_power/open-Schritt mehr.
    program_running_sub_ = create_subscription<std_msgs::msg::Bool>(
      "io_and_status_controller/robot_program_running", rclcpp::QoS(1),
      [this](const std_msgs::msg::Bool::ConstSharedPtr & msg) {
        const bool running = msg->data;
        const bool rising = running && program_running_state_ != 1;
        program_running_state_ = running ? 1 : 0;
        if (rising) {
          on_external_control_ready();
        }
      });

    state_pub_ = create_publisher<rg6_msgs::msg::GripperState>("rg6/state", rclcpp::QoS(10));
    urscript_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("urscript_topic").as_string(), rclcpp::QoS(1).reliable());

    const double state_rate = get_parameter("state_rate").as_double();
    state_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, state_rate)),
      [this]() { publish_state(); });

    // --- Services (Namen 'open'/'close' + deployte Aliasse '*_gripper') ----
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
      std::bind(&RG6ControlNode::handle_grip, this,
        std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), blocking_cb_group_);

    force_preset_service_ = create_service<std_srvs::srv::SetBool>(
      "rg6_control/set_force_preset",
      std::bind(&RG6ControlNode::handle_set_force_preset, this,
        std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), blocking_cb_group_);

    tool_power_service_ = create_service<std_srvs::srv::SetBool>(
      "rg6_control/set_tool_power",
      std::bind(&RG6ControlNode::handle_set_tool_power, this,
        std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), blocking_cb_group_);

    // --- GripperCommand-Action fuer MoveIt ---------------------------------
    // Name passend zum moveit_simple_controller_manager-Eintrag
    // 'manipulators/rg6_gripper_controller' + action_ns 'gripper_cmd'.
    action_server_ = rclcpp_action::create_server<GripperCommand>(
      this, "rg6_gripper_controller/gripper_cmd",
      std::bind(&RG6ControlNode::action_handle_goal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(&RG6ControlNode::action_handle_cancel, this, std::placeholders::_1),
      std::bind(&RG6ControlNode::action_handle_accepted, this, std::placeholders::_1),
      rcl_action_server_get_default_options(), blocking_cb_group_);

    // Tool-Spannung beim Start auf 24 V setzen (RG6-Stromversorgung). Der
    // io_and_status_controller kommt u. U. erst spaeter hoch -> aktiv pollen.
    auto attempts = std::make_shared<int>(0);
    const int max_attempts = static_cast<int>(get_parameter("startup_voltage_attempts").as_int());
    voltage_timer_ = create_wall_timer(
      2s,
      [this, attempts, max_attempts]() {
        if (io_client_->wait_for_service(100ms)) {
          const auto volts = get_parameter("tool_voltage").as_int();
          if (send_set_io(ur_msgs::srv::SetIO::Request::FUN_SET_TOOL_VOLTAGE, 0,
              static_cast<float>(volts), false))
          {
            std::lock_guard<std::mutex> lk(state_mutex_);
            tool_power_on_ = volts > 0;
          }
          RCLCPP_INFO(get_logger(), "RG6: Tool-Spannung auf %ldV gesetzt",
            static_cast<long>(volts));
          voltage_timer_->cancel();
          return;
        }
        if (++(*attempts) >= max_attempts) {
          RCLCPP_ERROR(get_logger(),
            "RG6: set_io nach %d Versuchen nicht verfuegbar - Tool-Spannung "
            "NICHT gesetzt (io_and_status_controller aktiv?)", max_attempts);
          voltage_timer_->cancel();
          return;
        }
        RCLCPP_WARN(get_logger(),
          "RG6: warte auf io_and_status_controller/set_io (Versuch %d/%d) ...",
          *attempts, max_attempts);
      });
  }

private:
  // ======================== Parameter =====================================
  void declare_parameters()
  {
    // UR-Tool-I/O-Belegung (ur_msgs-Pinnummern; Tool-Pins = 16/17).
    declare_parameter<int>("io_grip_pin", 16);           // Tool-DO0: auf/zu
    declare_parameter<int>("io_force_preset_pin", 17);   // Tool-DO1: Kraft-Preset
    declare_parameter<int>("tool_di_grip_detected_pin", 16);  // Tool-DI0
    declare_parameter<int>("tool_di_ready_pin", 17);          // Tool-DI1
    declare_parameter<int>("tool_voltage", 24);
    declare_parameter<int>("startup_voltage_attempts", 30);   // 30 * 2 s = 60 s
    // Prime beim Aktivwerden von ExternalControl: Tool bestromen und den Greifer
    // einmal oeffnen, damit AI2 gueltige Weiten liefert (sonst 0 V bis zum ersten
    // Kommando -> falscher Ist-Zustand + MoveIt-Startzustand vergiftet). Deckt auch
    // spaetes Einschalten des Arms ab (Programm-Flanke statt nur Startup-Timer).
    // prime_on_ready=false schaltet nur das Auto-Oeffnen ab (Bestromen bleibt).
    declare_parameter<bool>("prime_on_ready", true);
    declare_parameter<double>("prime_settle_s", 1.0);  // Wartezeit Tool-Power -> prime

    // Analog-Kalibrierung Tool-AI2 -> Weite (live nachjustierbar):
    //   ros2 topic echo .../io_and_status_controller/tool_data
    declare_parameter<double>("width_in_open", 10.0);    // [V] AI2 bei ganz offen
    declare_parameter<double>("width_in_closed", 0.56);  // [V] AI2 bei ganz zu
    declare_parameter<double>("width_open_m", 0.160);    // [m] RG6-Hub offen
    declare_parameter<double>("width_closed_m", 0.0);    // [m] zu

    // Gelenkwinkel-Konvention des Modells (rg6_finger_joint): 0 = offen, + = zu.
    // Muss zu rg6_joint_state_broadcaster (angle_open/angle_closed) passen.
    declare_parameter<double>("angle_open", 0.0);
    declare_parameter<double>("angle_closed", 0.6);

    // Bewegungserkennung ueber tool_data (Fallback, wenn io_states fehlt) und
    // Settle-Kriterium (deckt auch "auf Objekt geklemmt" als Erfolg ab).
    declare_parameter<double>("force_threshold", 3.1);   // [V] AI3: Bewegung laeuft
    declare_parameter<double>("move_eps", 0.10);         // [V] AI2-Delta = Bewegung
    declare_parameter<double>("settle_eps", 0.05);       // [V] AI2 stabil unterhalb
    declare_parameter<double>("settle_time_s", 0.4);
    declare_parameter<double>("motion_timeout_s", 10.0);
    // Startet innerhalb dieser Zeit KEINE Bewegung, gilt das Kommando als
    // "bereits in Zielzustand" (Erfolg) statt Timeout - wichtig fuer MoveIt-
    // Goals auf den aktuellen Zustand (open bei offenem Greifer usw.).
    declare_parameter<double>("no_motion_grace_s", 2.0);

    // Edge-Sicherung fuer Level-Kommandos (Tool-DO0): der RG6 reagiert auf die FLANKE
    // (Uebergang) auf DO0, NICHT auf einen statischen Level. Steht DO0 schon auf dem
    // Ziel-Level (z.B. Prime hatte DO0=0, dann nochmal 'open'), gibt es keine Flanke ->
    // keine Bewegung -> AI2 bleibt ungueltig. Daher kurz den Gegenspiegel setzen, dann
    // das Ziel (-> Flanke auf Ziel).
    declare_parameter<double>("edge_toggle_ms", 150.0);   // Dauer des Gegenspiels [ms]
    // AI2 unter diesem Wert gilt als "kein gueltiges Feedback" (Tool stromlos / vor
    // erstem Kommando). Pre-Check "bereits im Zielzustand" dann NICHT vertrauen, sondern
    // Edge erzwingen (bewegt den Greifer, macht AI2 danach gueltig). Muss unter
    // width_in_closed (0.56 V) liegen.
    declare_parameter<double>("dead_input_threshold", 0.2);  // [V]

    // Zustands-Publisher.
    declare_parameter<double>("state_rate", 20.0);

    // Backend fuer Ziel-Weite/-Kraft ('grip'-Service + Action-Zwischenweiten):
    //   'urscript' = 17-Bit-Protokoll per URScript-Injektion (voller Funktionsumfang)
    //   'io'       = nur Level-Kommandos -> 'grip' wird abgelehnt
    declare_parameter<std::string>("grip_backend", "urscript");
    declare_parameter<std::string>("urscript_topic", "urscript_interface/script_command");
    declare_parameter<bool>("resend_program_after_grip", true);
    declare_parameter<double>("resend_program_delay_s", 0.5);

    // 17-Bit-Protokoll (OnRobot-URCap-Kodierung; RG2-belegt, RG6-Defaults =
    // Datenblatt-Bereiche -> vor Produktivnutzung am Geraet verifizieren!):
    //   rg_data = floor(width_mm)*4 + floor(force_n/force_divisor)*4*width_steps
    declare_parameter<double>("grip_max_width_mm", 160.0);   // RG6-Hub
    declare_parameter<double>("grip_min_force_n", 25.0);     // RG6-Datenblatt
    declare_parameter<double>("grip_max_force_n", 120.0);    // RG6-Datenblatt
    declare_parameter<double>("grip_default_force_n", 60.0);
    declare_parameter<int>("rg_width_steps", 161);           // RG2: 111 (=max+1)
    declare_parameter<int>("rg_force_divisor", 2);
    declare_parameter<bool>("rg_slave", false);              // 2. Greifer (Dual)
    declare_parameter<int>("rg_slave_flag", 16384);
    declare_parameter<int>("rg_ready_timeout_sync", 400);    // sync-Zyklen (8 ms)
    declare_parameter<int>("rg_busy_start_timeout_sync", 20);

    // Action: Toleranz, ab der ein Zielwinkel als 'offen'/'zu' behandelt wird
    // (dann Level-Kommando statt URScript-Injektion).
    declare_parameter<double>("action_endpoint_angle_tol", 0.05);  // [rad]
    declare_parameter<double>("action_goal_angle_tol", 0.08);      // [rad]
  }

  // ======================== Hilfsfunktionen ================================
  double width_from_raw(double raw) const
  {
    return map_clamped(
      raw,
      get_parameter("width_in_closed").as_double(),
      get_parameter("width_in_open").as_double(),
      get_parameter("width_closed_m").as_double(),
      get_parameter("width_open_m").as_double());
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

  bool send_set_io(int8_t fun, int8_t pin, float state, bool wait_for_service = true)
  {
    if (wait_for_service && !io_client_->wait_for_service(1s)) {
      RCLCPP_ERROR(get_logger(),
        "RG6: io_and_status_controller/set_io nicht verfuegbar");
      return false;
    }
    auto req = std::make_shared<ur_msgs::srv::SetIO::Request>();
    req->fun = fun;
    req->pin = pin;
    req->state = state;
    auto future = io_client_->async_send_request(req);
    // Reentrant-Gruppe + MultiThreadedExecutor -> hier darf gewartet werden.
    if (future.wait_for(2s) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "RG6: set_io-Antwort steht aus (fun=%d pin=%d)", fun, pin);
      return true;  // Kommando ist raus; Bestaetigung kommt physisch via tool_data
    }
    return future.get()->success;
  }

  // Setzt Tool-DO0 auf `target` mit garantierter FLANKE: kurz den Gegenspiegel, dann
  // das Ziel. Der RG6 reagiert auf die Flanke (Uebergang) auf DO0, nicht auf einen
  // statischen Level - steht DO0 schon auf dem Ziel, bewegt sich ohne diesen Toggle
  // nichts (Edge-Sicherung, s. edge_toggle_ms).
  bool send_grip_do_edge(int8_t pin, float target)
  {
    const float opposite = (target > 0.5f) ? 0.0f : 1.0f;
    if (!send_set_io(ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT, pin, opposite)) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(
      static_cast<int>(get_parameter("edge_toggle_ms").as_double())));
    return send_set_io(ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT, pin, target);
  }

  struct MotionResult
  {
    bool settled{false};
    bool started{false};
    bool grip_detected{false};
    double final_width_m{kNaN};
    double final_angle{kNaN};
  };

  // Wartet auf Bewegungsende: primaer ueber Tool-DI1 (ready), sonst ueber das
  // Settle-Kriterium auf AI2. "Auf Objekt geklemmt" (Position stabil, Kraft
  // hoch) zaehlt bewusst als Erfolg (frueherer Bug: Greifen am Objekt -> TIMEOUT).
  MotionResult wait_motion_done(const std::function<bool()> & canceled = nullptr)
  {
    const double force_threshold = get_parameter("force_threshold").as_double();
    const double move_eps = get_parameter("move_eps").as_double();
    const double settle_eps = get_parameter("settle_eps").as_double();
    const auto settle_time = rclcpp::Duration::from_seconds(
      get_parameter("settle_time_s").as_double());
    const auto timeout = rclcpp::Duration::from_seconds(
      get_parameter("motion_timeout_s").as_double());

    const auto no_motion_grace = rclcpp::Duration::from_seconds(
      get_parameter("no_motion_grace_s").as_double());

    MotionResult result;
    const auto start = now();
    bool have_initial = false;
    double initial_pos = 0.0;
    double settle_ref = 0.0;
    auto last_change = now();
    bool saw_busy = false;

    while (rclcpp::ok() && (now() - start) < timeout) {
      if (canceled && canceled()) {
        break;
      }
      // Keine Bewegung angelaufen (weder DI-busy noch Analog-Delta)? Dann war
      // der Greifer schon im Zielzustand -> Erfolg statt Timeout.
      if (!result.started && (now() - start) > no_motion_grace) {
        std::lock_guard<std::mutex> lk(state_mutex_);
        if (have_tool_data_ || have_io_states_) {
          result.settled = true;
          result.grip_detected = have_io_states_ ? di_grip_detected_ : false;
          break;
        }
      }
      std::this_thread::sleep_for(20ms);

      double pos, force;
      bool have_analog, have_di, di_ready, di_grip;
      {
        std::lock_guard<std::mutex> lk(state_mutex_);
        pos = width_raw_;
        force = force_raw_;
        have_analog = have_tool_data_;
        have_di = have_io_states_;
        di_ready = di_ready_;
        di_grip = di_grip_detected_;
      }
      if (!have_analog && !have_di) {
        continue;
      }

      // Tool-DI1: ready-Signal des RG6 (low = Bewegung laeuft).
      if (have_di) {
        if (!di_ready) {
          saw_busy = true;
          result.started = true;
        } else if (saw_busy) {
          // busy -> ready-Flanke: Bewegung sicher abgeschlossen.
          result.settled = true;
          result.grip_detected = di_grip;
          break;
        }
      }

      if (have_analog) {
        if (!have_initial) {
          initial_pos = pos;
          settle_ref = pos;
          have_initial = true;
          last_change = now();
        }
        if (!result.started &&
          (force > force_threshold || std::fabs(pos - initial_pos) > move_eps))
        {
          result.started = true;
        }
        if (std::fabs(pos - settle_ref) > settle_eps) {
          settle_ref = pos;
          last_change = now();
        }
        if (result.started && (now() - last_change) > settle_time) {
          result.settled = true;
          result.grip_detected = have_di ? di_grip : false;
          break;
        }
      }
    }

    double raw;
    bool have_analog;
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      raw = width_raw_;
      have_analog = have_tool_data_;
      if (result.settled && have_io_states_) {
        result.grip_detected = di_grip_detected_;
      }
    }
    if (have_analog) {
      result.final_width_m = width_from_raw(raw);
      result.final_angle = angle_from_width(result.final_width_m);
    }
    return result;
  }

  // ======================== Level-Kommandos (Tool-DO0) =====================
  void handle_open_close(bool close_cmd, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::unique_lock<std::mutex> motion_lock(motion_mutex_, std::try_to_lock);
    if (!motion_lock.owns_lock()) {
      response->success = false;
      response->message = "RG6: Bewegung laeuft bereits (Kommando abgelehnt)";
      return;
    }
    RCLCPP_INFO(get_logger(), "RG6: %s (Level-Kommando Tool-DO0, edge-gesichert)", close_cmd ? "close" : "open");

    const auto pin = static_cast<int8_t>(get_parameter("io_grip_pin").as_int());
    const float target = close_cmd ? 1.0f : 0.0f;

    // Pre-Check bei gueltigem AI2: Greifer schon im Zielzustand -> kein Kommando noetig
    // (vermeidet Jitter durch das Edge-Toggle). Bei ungueltigem AI2 (Tool stromlos / vor
    // erstem Kommando) nicht pruefbar -> unten Edge erzwingen (bewegt den Greifer, macht
    // AI2 danach gueltig).
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      const double dead = get_parameter("dead_input_threshold").as_double();
      if (have_tool_data_ && width_raw_ >= dead) {
        const double w = width_from_raw(width_raw_);
        const double w_open = get_parameter("width_open_m").as_double();
        const double w_closed = get_parameter("width_closed_m").as_double();
        const double eps = 0.01;  // [m] Schwelle fuer "im Zielzustand"
        const bool already = close_cmd ? (w <= w_closed + eps) : (w >= w_open - eps);
        if (already) {
          last_command_ = close_cmd ? rg6_msgs::msg::GripperState::COMMAND_CLOSE :
            rg6_msgs::msg::GripperState::COMMAND_OPEN;
          response->success = true;
          response->message = "Greifer bereits in Zielzustand (OK, keine Bewegung)";
          RCLCPP_INFO(get_logger(), "RG6: %s", response->message.c_str());
          return;
        }
      }
    }

    // Edge sicherstellen: kurz Gegenspiegel, dann Ziel -> Flanke auf Ziel. (Der RG6
    // reagiert auf die Flanke, nicht auf den statischen Level; steht DO0 schon auf dem
    // Ziel, bewegt sich ohne diesen Toggle nichts.)
    if (!send_grip_do_edge(pin, target))
    {
      response->success = false;
      response->message = "UR IO service (io_and_status_controller/set_io) not available";
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return;
    }
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      last_command_ = close_cmd ? rg6_msgs::msg::GripperState::COMMAND_CLOSE :
        rg6_msgs::msg::GripperState::COMMAND_OPEN;
    }

    const auto result = wait_motion_done();
    response->success = result.settled;
    std::ostringstream msg;
    msg << (result.settled ?
      (result.started ? "Gripper motion settled (OK)" :
      "Gripper bereits in Zielzustand (OK, keine Bewegung)") :
      "Gripper motion did not settle (TIMEOUT)");
    if (result.settled && result.grip_detected) {
      msg << ", grip detected";
    }
    if (std::isfinite(result.final_width_m)) {
      msg << ", width=" << result.final_width_m << " m";
    }
    response->message = msg.str();
    RCLCPP_INFO(get_logger(), "RG6: %s", response->message.c_str());
  }

  // ======================== Kraft-Preset (Tool-DO1) ========================
  void handle_set_force_preset(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    const auto pin = static_cast<int8_t>(get_parameter("io_force_preset_pin").as_int());
    if (!send_set_io(ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT, pin,
        request->data ? 1.0f : 0.0f))
    {
      response->success = false;
      response->message = "UR IO service nicht verfuegbar";
      return;
    }
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      high_force_preset_ = request->data;
    }
    response->success = true;
    response->message = request->data ? "Kraft-Preset: high (Tool-DO1=1)" :
      "Kraft-Preset: low (Tool-DO1=0)";
    RCLCPP_INFO(get_logger(), "RG6: %s", response->message.c_str());
  }

  // ======================== Tool-Spannung (Power) ==========================
  void handle_set_tool_power(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    const auto volts = request->data ?
      static_cast<float>(get_parameter("tool_voltage").as_int()) : 0.0f;
    if (!send_set_io(ur_msgs::srv::SetIO::Request::FUN_SET_TOOL_VOLTAGE, 0, volts)) {
      response->success = false;
      response->message = "UR IO service nicht verfuegbar";
      return;
    }
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      tool_power_on_ = request->data;
      if (!request->data) {
        have_tool_data_ = false;  // Analogwerte sind ohne Spannung bedeutungslos
        primed_once_ = false;     // nach echtem Power-Off beim naechsten Hochlauf neu primen
      }
    }
    response->success = true;
    response->message = request->data ? "Tool-Spannung 24V an" : "Tool-Spannung aus (0V)";
    RCLCPP_INFO(get_logger(), "RG6: %s", response->message.c_str());
  }

  // ExternalControl ist (wieder) aktiv (steigende Flanke von robot_program_running)
  // -> Tool bestromen und den Greifer einmal primen, damit AI2 gueltige Weiten
  // liefert. Deckt normalen Boot UND spaetes Einschalten des Arms ab -> kein
  // manueller set_tool_power/open-Schritt noetig. Laeuft im eigenen Thread
  // (blockiert auf set_io + Bewegung); prime_in_progress_ verhindert Doppelstart.
  void on_external_control_ready()
  {
    // WICHTIG: nur beim ERSTEN Hochlaufen nach dem Einschalten bestromen+primen.
    // Spaetere Flanken sind i.d.R. der ExternalControl-Neustart NACH einer URScript-
    // Grip-Injektion (do_urscript_grip -> resend_robot_program). Da darf NICHT erneut
    // geoeffnet werden - das liesse ein gegriffenes Objekt fallen. Nach einem echten
    // Tool-Power-Off (handle_set_tool_power) wird primed_once_ zurueckgesetzt.
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      if (primed_once_ && tool_power_on_) {
        return;  // schon versorgt & geprimt (z.B. Grip-Resend) -> nichts tun
      }
    }
    bool expected = false;
    if (!prime_in_progress_.compare_exchange_strong(expected, true)) {
      return;  // laeuft bereits
    }
    std::thread{[this]() {
      RCLCPP_INFO(get_logger(),
        "RG6: ExternalControl aktiv -> Tool bestromen + Greifer primen (AI2 gueltig machen).");
      const auto volts = get_parameter("tool_voltage").as_int();
      if (send_set_io(ur_msgs::srv::SetIO::Request::FUN_SET_TOOL_VOLTAGE, 0,
          static_cast<float>(volts)))
      {
        std::lock_guard<std::mutex> lk(state_mutex_);
        tool_power_on_ = volts > 0;
      } else {
        RCLCPP_WARN(get_logger(),
          "RG6: Tool-Bestromung fehlgeschlagen (set_io nicht verfuegbar).");
      }
      bool do_prime = get_parameter("prime_on_ready").as_bool();
      {
        std::lock_guard<std::mutex> lk(state_mutex_);
        if (primed_once_) do_prime = false;  // seit dem Einschalten schon geprimt
      }
      if (do_prime) {
        // kurz warten, bis die Tool-Spannung anliegt, dann einmal oeffnen. Das
        // erste Kommando bringt den RG6 dazu, AI2 auf echte Werte zu treiben.
        std::this_thread::sleep_for(std::chrono::duration<double>(
            std::max(0.0, get_parameter("prime_settle_s").as_double())));
        auto resp = std::make_shared<std_srvs::srv::Trigger::Response>();
        handle_open_close(/*close_cmd=*/false, resp);  // greift motion_mutex_ selbst
        RCLCPP_INFO(get_logger(), "RG6: Prime-Open: %s", resp->message.c_str());
        std::lock_guard<std::mutex> lk(state_mutex_);
        primed_once_ = true;
      }
      prime_in_progress_ = false;
    }}.detach();
  }

  // ======================== Ziel-Weite/-Kraft (URScript) ===================
  // Baut das OnRobot-17-Bit-Kommando als injizierbares URScript-Programm.
  // Vorlage: community-dokumentiertes RG2/RG6-URScript (python-urx PR#35);
  // CB3-Flachnummerierung: digital_out 8/9 = Tool-DO0/DO1, digital_in 9 = Tool-DI1.
  std::string build_grip_urscript(double width_mm, double force_n) const
  {
    const int width_steps = static_cast<int>(get_parameter("rg_width_steps").as_int());
    const int force_div = std::max(1, static_cast<int>(get_parameter("rg_force_divisor").as_int()));
    const int ready_timeout = static_cast<int>(get_parameter("rg_ready_timeout_sync").as_int());
    const int busy_timeout = static_cast<int>(get_parameter("rg_busy_start_timeout_sync").as_int());

    long rg_data = static_cast<long>(std::floor(width_mm)) * 4L;
    rg_data += static_cast<long>(std::floor(force_n / force_div)) * 4L * width_steps;
    if (get_parameter("rg_slave").as_bool()) {
      rg_data += get_parameter("rg_slave_flag").as_int();
    }

    std::ostringstream s;
    s << "def rg6_ros_grip():\n"
      << "  textmsg(\"rg6_control: grip \", " << rg_data << ")\n"
      // warten, bis der Greifer bereit ist (Tool-DI1 high = Ruhe)
      << "  timeout = 0\n"
      << "  while get_digital_in(9) == False:\n"
      << "    if timeout > " << ready_timeout << ":\n"
      << "      break\n"
      << "    end\n"
      << "    timeout = timeout+1\n"
      << "    sync()\n"
      << "  end\n"
      // 17-Bit-Wort MSB-first auf DO8(Takt)/DO9(Daten, invertiert) takten
      << "  def bit(input):\n"
      << "    msb=65536\n"
      << "    local i=0\n"
      << "    local output=0\n"
      << "    while i<17:\n"
      << "      set_digital_out(8,True)\n"
      << "      if input>=msb:\n"
      << "        input=input-msb\n"
      << "        set_digital_out(9,False)\n"
      << "      else:\n"
      << "        set_digital_out(9,True)\n"
      << "      end\n"
      << "      if get_digital_in(8):\n"
      << "        out=1\n"
      << "      end\n"
      << "      sync()\n"
      << "      set_digital_out(8,False)\n"
      << "      sync()\n"
      << "      input=input*2\n"
      << "      output=output*2\n"
      << "      i=i+1\n"
      << "    end\n"
      << "    return output\n"
      << "  end\n"
      << "  bit(" << rg_data << ")\n"
      // Bewegungsstart (ready faellt) ...
      << "  timeout = 0\n"
      << "  while get_digital_in(9) == True:\n"
      << "    timeout = timeout+1\n"
      << "    sync()\n"
      << "    if timeout > " << busy_timeout << ":\n"
      << "      break\n"
      << "    end\n"
      << "  end\n"
      // ... und Bewegungsende (ready steigt) im Script abwarten, damit das
      // ExternalControl-Resend die Bewegung nicht unterbricht.
      << "  timeout = 0\n"
      << "  while get_digital_in(9) == False:\n"
      << "    timeout = timeout+1\n"
      << "    sync()\n"
      << "    if timeout > " << ready_timeout << ":\n"
      << "      break\n"
      << "    end\n"
      << "  end\n"
      << "  textmsg(\"rg6_control: grip done\")\n"
      << "end\n";
    return s.str();
  }

  // Fuehrt einen Weite/Kraft-Grip aus (blockierend). motion_mutex_ muss gehalten sein.
  MotionResult do_urscript_grip(double width_m, double force_n, std::string & error)
  {
    const double max_width_mm = get_parameter("grip_max_width_mm").as_double();
    const double min_force = get_parameter("grip_min_force_n").as_double();
    const double max_force = get_parameter("grip_max_force_n").as_double();
    if (force_n <= 0.0) {
      force_n = get_parameter("grip_default_force_n").as_double();
    }
    const double width_mm = std::clamp(width_m * 1000.0, 0.0, max_width_mm);
    force_n = std::clamp(force_n, min_force, max_force);

    if (urscript_pub_->get_subscription_count() == 0) {
      error = "urscript_interface nicht verbunden (Topic '" +
        std::string(urscript_pub_->get_topic_name()) +
        "' hat keinen Subscriber) - laeuft der urscript_interface-Node?";
      return MotionResult{};
    }

    std_msgs::msg::String script;
    script.data = build_grip_urscript(width_mm, force_n);
    RCLCPP_INFO(get_logger(),
      "RG6: grip width=%.1f mm force=%.0f N (URScript-Injektion, unterbricht "
      "ExternalControl)", width_mm, force_n);
    urscript_pub_->publish(script);
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      last_command_ = rg6_msgs::msg::GripperState::COMMAND_GRIP;
    }

    auto result = wait_motion_done();

    if (get_parameter("resend_program_after_grip").as_bool()) {
      std::this_thread::sleep_for(std::chrono::duration<double>(
          get_parameter("resend_program_delay_s").as_double()));
      if (resend_client_->wait_for_service(2s)) {
        auto fut = resend_client_->async_send_request(
          std::make_shared<std_srvs::srv::Trigger::Request>());
        if (fut.wait_for(5s) == std::future_status::ready && fut.get()->success) {
          RCLCPP_INFO(get_logger(), "RG6: ExternalControl wiederhergestellt (resend_robot_program)");
        } else {
          RCLCPP_WARN(get_logger(),
            "RG6: resend_robot_program fehlgeschlagen - Arm ggf. via "
            "ur_state_manager/ensure_ready reaktivieren");
        }
      } else {
        RCLCPP_WARN(get_logger(), "RG6: resend_robot_program-Service nicht verfuegbar");
      }
    }
    return result;
  }

  void handle_grip(
    const std::shared_ptr<rg6_msgs::srv::Grip::Request> request,
    std::shared_ptr<rg6_msgs::srv::Grip::Response> response)
  {
    response->final_width = kNaN;
    const auto backend = get_parameter("grip_backend").as_string();
    if (backend != "urscript") {
      response->success = false;
      response->message =
        "grip_backend='" + backend + "' kann keine Zielweite/-kraft; nutze "
        "open/close oder setze grip_backend=urscript";
      return;
    }
    std::unique_lock<std::mutex> motion_lock(motion_mutex_, std::try_to_lock);
    if (!motion_lock.owns_lock()) {
      response->success = false;
      response->message = "RG6: Bewegung laeuft bereits (Kommando abgelehnt)";
      return;
    }

    std::string error;
    if (!request->wait) {
      // Nur absetzen: Script raus, nicht auf Bewegung warten.
      const double max_width_mm = get_parameter("grip_max_width_mm").as_double();
      double force_n = request->force > 0.0 ? request->force :
        get_parameter("grip_default_force_n").as_double();
      force_n = std::clamp(force_n,
        get_parameter("grip_min_force_n").as_double(),
        get_parameter("grip_max_force_n").as_double());
      if (urscript_pub_->get_subscription_count() == 0) {
        response->success = false;
        response->message = "urscript_interface nicht verbunden";
        return;
      }
      std_msgs::msg::String script;
      script.data = build_grip_urscript(
        std::clamp(request->width * 1000.0, 0.0, max_width_mm), force_n);
      urscript_pub_->publish(script);
      {
        std::lock_guard<std::mutex> lk(state_mutex_);
        last_command_ = rg6_msgs::msg::GripperState::COMMAND_GRIP;
      }
      response->success = true;
      response->message = "grip-Kommando abgesetzt (wait=false; resend_robot_program "
        "NICHT automatisch gerufen)";
      return;
    }

    const auto result = do_urscript_grip(request->width, request->force, error);
    if (!error.empty()) {
      response->success = false;
      response->message = error;
      RCLCPP_ERROR(get_logger(), "RG6: %s", error.c_str());
      return;
    }
    response->success = result.settled;
    response->grip_detected = result.grip_detected;
    response->final_width = result.final_width_m;
    response->message = result.settled ?
      (result.grip_detected ? "Grip OK (Objekt erkannt)" : "Grip OK (Zielweite erreicht)") :
      "Grip nicht abgeschlossen (TIMEOUT)";
    RCLCPP_INFO(get_logger(), "RG6: %s", response->message.c_str());
  }

  // ======================== GripperCommand-Action (MoveIt) =================
  rclcpp_action::GoalResponse action_handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const GripperCommand::Goal> goal)
  {
    const double lo = std::min(get_parameter("angle_open").as_double(),
      get_parameter("angle_closed").as_double());
    const double hi = std::max(get_parameter("angle_open").as_double(),
      get_parameter("angle_closed").as_double());
    if (!std::isfinite(goal->command.position) ||
      goal->command.position < lo - 0.2 || goal->command.position > hi + 0.2)
    {
      RCLCPP_WARN(get_logger(), "RG6-Action: Zielposition %.3f ausserhalb [%.2f, %.2f]",
        goal->command.position, lo, hi);
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse action_handle_cancel(
    const std::shared_ptr<GoalHandleGripperCommand>)
  {
    // Der RG6 kennt kein Stop-Kommando; Cancel beendet nur das Warten.
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void action_handle_accepted(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
  {
    // Eigener Thread pro Goal (Standard-Pattern) -> Executor bleibt frei.
    std::thread{[this, goal_handle]() { action_execute(goal_handle); }}.detach();
  }

  void action_execute(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
  {
    auto result = std::make_shared<GripperCommand::Result>();
    const auto goal = goal_handle->get_goal();

    std::unique_lock<std::mutex> motion_lock(motion_mutex_, std::try_to_lock);
    if (!motion_lock.owns_lock()) {
      result->position = current_angle();
      result->reached_goal = false;
      result->stalled = false;
      goal_handle->abort(result);
      return;
    }

    const double angle_open = get_parameter("angle_open").as_double();
    const double angle_closed = get_parameter("angle_closed").as_double();
    const double endpoint_tol = get_parameter("action_endpoint_angle_tol").as_double();
    const double goal_tol = get_parameter("action_goal_angle_tol").as_double();
    const double target_angle = goal->command.position;
    const auto backend = get_parameter("grip_backend").as_string();

    // Feedback-Thread: 10 Hz Ist-Position, solange die Bewegung laeuft.
    std::atomic<bool> feedback_running{true};
    std::thread feedback_thread([this, goal_handle, &feedback_running]() {
        while (feedback_running && rclcpp::ok()) {
          auto fb = std::make_shared<GripperCommand::Feedback>();
          fb->position = current_angle();
          fb->effort = current_force_raw();
          fb->reached_goal = false;
          fb->stalled = false;
          goal_handle->publish_feedback(fb);
          std::this_thread::sleep_for(100ms);
        }
      });

    MotionResult motion;
    std::string error;
    bool sent = false;
    const auto canceled = [goal_handle]() { return goal_handle->is_canceling(); };

    if (std::fabs(target_angle - angle_open) <= endpoint_tol ||
      std::fabs(target_angle - angle_closed) <= endpoint_tol || backend != "urscript")
    {
      // Endlagen (MoveIt-Zustaende 'open'/'close') -> robustes Level-Kommando.
      const bool close_cmd =
        std::fabs(target_angle - angle_closed) <= std::fabs(target_angle - angle_open);
      if (backend != "urscript" &&
        std::fabs(target_angle - angle_open) > endpoint_tol &&
        std::fabs(target_angle - angle_closed) > endpoint_tol)
      {
        RCLCPP_WARN(get_logger(),
          "RG6-Action: Zwischenweite %.3f rad mit grip_backend=io nicht praezise "
          "erreichbar -> naechste Endlage (%s)", target_angle, close_cmd ? "close" : "open");
      }
      // Pre-Check: Greifer schon im Zielzustand -> kein Edge-Toggle noetig.
      // Vermeidet das "zu und wieder auf"-Jitter, das send_grip_do_edge sonst
      // erzwingt (der RG6 reagiert auf die Flanke auf DO0, nicht auf den
      // statischen Level -> ein redundantes open/close treibt ihn kurz in die
      // Gegenendlage). WICHTIG - Wake-up-Garantie: unterdrueckt wird NUR, wenn
      // der Greifer seit dem Einschalten bereits geprimt ist (primed_once_,
      // s. on_external_control_ready / README "RG6 treibt AI2 erst nach dem
      // ersten Kommando"). Vor dem Priming ist AI2 unzuverlaessig; also in
      // jedem Fall das Edge erzwingen -> einmal Bestromung/Flanke geben, damit
      // AI2 gueltig wird und keine Stale-Zustaende bleiben. primed_once_ wird
      // bei echtem Tool-Power-Off zurueckgesetzt (handle_set_tool_power) -> beim
      // Neustart des Arms greift das Wake-up wieder. Konsistent zu handle_open_close.
      bool already_at_target = false;
      {
        std::lock_guard<std::mutex> lk(state_mutex_);
        const bool primed = primed_once_;
        const double dead = get_parameter("dead_input_threshold").as_double();
        if (primed && have_tool_data_ && width_raw_ >= dead) {
          const double w = width_from_raw(width_raw_);
          const double w_open = get_parameter("width_open_m").as_double();
          const double w_closed = get_parameter("width_closed_m").as_double();
          const double eps = 0.01;  // [m] Schwelle fuer "im Zielzustand"
          already_at_target = close_cmd ? (w <= w_closed + eps) : (w >= w_open - eps);
        }
      }
      if (already_at_target) {
        RCLCPP_INFO(get_logger(), "RG6-Action: Greifer bereits in Zielzustand (%s) - kein Edge-Toggle",
          close_cmd ? "close" : "open");
        sent = true;
        {
          std::lock_guard<std::mutex> lk(state_mutex_);
          last_command_ = close_cmd ? rg6_msgs::msg::GripperState::COMMAND_CLOSE :
            rg6_msgs::msg::GripperState::COMMAND_OPEN;
        }
        // Keine Bewegung: direkt als settled mit Ist-Position melden, so dass
        // reached_goal wahr wird und das Goal succceed statt wait_motion_done.
        motion.settled = true;
        motion.final_angle = current_angle();
      } else {
        const auto pin = static_cast<int8_t>(get_parameter("io_grip_pin").as_int());
        // Edge-gesichert (s. handle_open_close): Flanke auf DO0, nicht nur statischer Level.
        sent = send_grip_do_edge(pin, close_cmd ? 1.0f : 0.0f);
        if (sent) {
          std::lock_guard<std::mutex> lk(state_mutex_);
          last_command_ = close_cmd ? rg6_msgs::msg::GripperState::COMMAND_CLOSE :
            rg6_msgs::msg::GripperState::COMMAND_OPEN;
        }
        if (sent) {
          motion = wait_motion_done(canceled);
        } else {
          error = "set_io nicht verfuegbar";
        }
      }
    } else {
      // Zwischenweite -> URScript-Grip mit Kraft aus max_effort.
      motion = do_urscript_grip(width_from_angle(target_angle), goal->command.max_effort, error);
      sent = error.empty();
    }

    feedback_running = false;
    feedback_thread.join();

    result->position = std::isfinite(motion.final_angle) ? motion.final_angle : current_angle();
    result->effort = current_force_raw();
    result->stalled = motion.grip_detected;
    result->reached_goal = motion.settled &&
      std::fabs(result->position - target_angle) <= goal_tol;

    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      return;
    }
    if (!sent) {
      RCLCPP_ERROR(get_logger(), "RG6-Action: %s", error.c_str());
      goal_handle->abort(result);
      return;
    }
    // Erfolg = Bewegung abgeschlossen: Zielweite erreicht ODER auf Objekt
    // geklemmt (stalled) - beides ist fuer MoveIt-Greifziele ein Erfolg.
    if (motion.settled) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  // ======================== Zustands-Publisher =============================
  double current_angle()
  {
    double raw;
    bool have;
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      raw = width_raw_;
      have = have_tool_data_;
    }
    if (!have) {
      return get_parameter("angle_open").as_double();
    }
    return angle_from_width(width_from_raw(raw));
  }

  double current_force_raw()
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    return have_tool_data_ ? force_raw_ : 0.0;
  }

  void publish_state()
  {
    rg6_msgs::msg::GripperState msg;
    msg.header.stamp = get_clock()->now();
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      msg.width_raw = width_raw_;
      msg.force_raw = force_raw_;
      msg.tool_data_received = have_tool_data_;
      msg.io_states_received = have_io_states_;
      msg.busy = have_io_states_ ? !di_ready_ : false;
      msg.grip_detected = have_io_states_ ? di_grip_detected_ : false;
      msg.tool_power_on = tool_power_on_;
      msg.high_force_preset = high_force_preset_;
      msg.last_command = last_command_;
    }
    msg.width = msg.tool_data_received ? width_from_raw(msg.width_raw) : kNaN;
    state_pub_->publish(msg);
  }

  // ======================== Member ========================================
  rclcpp::CallbackGroup::SharedPtr blocking_cb_group_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr open_service_, close_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr open_alias_service_, close_alias_service_;
  rclcpp::Service<rg6_msgs::srv::Grip>::SharedPtr grip_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr force_preset_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr tool_power_service_;
  rclcpp_action::Server<GripperCommand>::SharedPtr action_server_;

  rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr io_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr resend_client_;
  rclcpp::Subscription<ur_msgs::msg::ToolDataMsg>::SharedPtr tool_data_sub_;
  rclcpp::Subscription<ur_msgs::msg::IOStates>::SharedPtr io_states_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr program_running_sub_;
  rclcpp::Publisher<rg6_msgs::msg::GripperState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr urscript_pub_;
  rclcpp::TimerBase::SharedPtr voltage_timer_, state_timer_;

  std::mutex state_mutex_;
  double width_raw_{0.0};
  double force_raw_{0.0};
  bool have_tool_data_{false};
  bool have_io_states_{false};
  bool di_grip_detected_{false};
  bool di_ready_{true};
  bool tool_power_on_{false};
  bool high_force_preset_{false};
  uint8_t last_command_{rg6_msgs::msg::GripperState::COMMAND_NONE};

  int program_running_state_{-1};        // -1 unbekannt, 0 aus, 1 ExternalControl laeuft
  std::atomic<bool> prime_in_progress_{false};
  bool primed_once_{false};              // seit letztem Power-On schon bestromt+geprimt?

  std::mutex motion_mutex_;  // eine Greiferbewegung zur Zeit
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RG6ControlNode>();
  // MultiThreadedExecutor: blockierende Services/Action (Reentrant-Gruppe)
  // duerfen warten, waehrend tool_data/io_states weiterlaufen.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
