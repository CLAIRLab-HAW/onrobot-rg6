#include <algorithm>
#include <cmath>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "ur_msgs/msg/tool_data_msg.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// Mappt tool_data.analog_input2 (Greiferweite) linear auf den Treiber-Gelenkwinkel
// 'rg6_finger_joint' (Modell rg6_description) und publiziert ihn als
// joint_states -> Greifer-Animation. Konvention: 0 = offen, positiv = zu.
//
// WICHTIG: Im Modell sind ALLE Greifergelenke revolute. Der robot_state_publisher
// sendet die TF eines beweglichen Gelenks nur, wenn er einen joint_state dafuer
// bekommt. Daher publizieren wir 'rg6_finger_joint' KONTINUIERLICH per Timer (mit
// dem letzten bekannten Wert, Start = angle_closed). Sonst fehlt ohne tool_data die
// TF und die Fingerglieder fallen auf den Ursprung ("Teile liegen nebeneinander").
//
// Die 4 Mapping-Werte sind LIVE-Parameter (in Foxglove justierbar):
//   in_closed / in_open  : analog_input2 bei physisch zu / auf  (ros2 topic echo tool_data)
//   angle_closed/angle_open : zugehoeriger rg6_finger_joint-Winkel (0=offen, +=zu)
// Live tunen, z.B.:
//   ros2 param set /a200_0553/manipulators/tooldata_to_jointstate angle_closed 0.6
//   ros2 param set /a200_0553/manipulators/tooldata_to_jointstate angle_open   0.0
// Wenn passend: die Defaults hier eintragen.

using namespace std::chrono_literals;

class ToolDataToJointStateNode : public rclcpp::Node
{
public:
  ToolDataToJointStateNode() : Node("tooldata_to_jointstate")
  {
    // Startwerte = grobe Schaetzung; bitte live nachjustieren (siehe oben).
    this->declare_parameter<double>("in_closed", 0.56);
    this->declare_parameter<double>("in_open", 10.0);
    this->declare_parameter<double>("angle_closed", 0.6);
    this->declare_parameter<double>("angle_open", 0.0);
    // Totzonen-Schwelle: AI2 UNTER diesem Wert gilt als "kein gueltiges Feedback"
    // (der RG6 treibt die Analogleitung erst nach dem ersten Kommando; davor ~0 V).
    // Muss unter dem Zu-Wert in_closed liegen (0.56 V) und ueber der 0-V-Totlage.
    this->declare_parameter<double>("dead_input_threshold", 0.2);
    // Tool muss bestromt sein (tool_output_voltage == 24 V), bevor AI2 als Weite
    // vertraut wird: VOR der 24-V-Bestromung (Arm frisch oben, rg6_control setzt die
    // Tool-Spannung erst sekunden spaeter) liefert AI2 gelegentlich stale/spurious
    // Werte (z.B. ~8 V -> "offen"), die nicht der physischen Greiferstellung
    // entsprechen -> der JSB wuerde sonst auf einen Scheinwert festfahren und in RViz
    // "offen" zeigen, obwohl der Greifer zu ist. Default 24 (Feld ist uint8: 0 oder 24).
    this->declare_parameter<double>("min_tool_voltage", 24.0);
    // Nach der steigenden Flanke der Tool-Spannung (0 -> 24 V) kann AI2 kurz einen
    // power-on-Transienten schlagen (z.B. auf 10 V "offen"), der nicht der Realitaet
    // entspricht (Greifer kommuniziert noch nicht / Analog-Eingang faehrt hoch). In
    // diesem Settle-Fenster AI2 nicht vertrauen -> letzten Wert halten.
    this->declare_parameter<double>("power_on_settle_s", 3.0);

    // Start = zu (angle_closed). Bei stromlosem Tool (Arm aus / kein tool_data / AI2
    // ungueltig) liefert der Greifer kein Weiten-Feedback; er ruht in der
    // geschlossenen Stellung (RG6 haelt die Position, typischer Rest-/Letztzustand).
    // Start=offen zeigte ihn faelschlich als offen an. Jeder Wert haelt das Modell
    // zusammengebaut (TF via kontinuierlichem Timer); nach dem ersten gueltigen AI2-
    // Feedback trackt der Callback den Ist-Zustand (und haelt bei erneutem Stromverlust
    // den letzten gueltigen Wert, s.u.).
    position_ = this->get_parameter("angle_closed").as_double();

    // Relative Namen -> im Namespace aufloesbar (z.B. /a200_0553/manipulators).
    // 'joint_states' wird per Launch-Remap auf das Topic des Greifer-rendernden
    // robot_state_publisher gelegt (a200-0553: /a200_0553/platform/joint_states).
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    sub_ = this->create_subscription<ur_msgs::msg::ToolDataMsg>(
      "io_and_status_controller/tool_data", rclcpp::SensorDataQoS(),
      std::bind(&ToolDataToJointStateNode::tooldata_callback, this, std::placeholders::_1));

    // Kontinuierlich publizieren -> RSP hat immer eine TF fuer rg6_finger_joint.
    timer_ = this->create_wall_timer(
      50ms, std::bind(&ToolDataToJointStateNode::publish, this));
  }

private:
  void tooldata_callback(const ur_msgs::msg::ToolDataMsg::SharedPtr msg)
  {
    const double in_closed    = this->get_parameter("in_closed").as_double();
    const double in_open      = this->get_parameter("in_open").as_double();
    const double angle_closed = this->get_parameter("angle_closed").as_double();
    const double angle_open   = this->get_parameter("angle_open").as_double();

    const double analog_input = msg->analog_input2;
    const double volt = static_cast<double>(msg->tool_output_voltage);
    const double min_volt = this->get_parameter("min_tool_voltage").as_double();

    // Steigende Flanke der Tool-Spannung (0 -> >=min_volt) erkennen: kurz nach dem
    // Bestromen kann AI2 einen power-on-Transienten schlagen (z.B. auf 10 V "offen"),
    // der nicht der Realitaet entspricht. Settle-Fenster abwarten (s.u.).
    if (prev_tool_voltage_ < min_volt && volt >= min_volt) {
      power_on_time_ = this->now();
      have_power_on_time_ = true;
    }
    prev_tool_voltage_ = volt;

    // Spannungs-Gate: nur tracken, wenn das Tool bestromt ist. Vor der 24-V-
    // Bestromung kann AI2 stale/spurious Werte liefern (siehe min_tool_voltage).
    if (volt < min_volt) {
      if (have_valid_) {
        have_valid_ = false;  // Tool-Spannung weg -> letzter Wert bleibt, bis wieder 24 V
        RCLCPP_WARN(this->get_logger(),
          "RG6-JSB: tool_output_voltage=%.0f < %.0f V -> Tool nicht bestromt, AI2 ignoriert (halte letzten Wert).",
          volt, min_volt);
      }
      return;  // position_ unveraendert (Startwert = angle_closed)
    }

    // Settle-Fenster nach der Tool-Bestromung: AI2 noch nicht vertrauen (power-on-
    // Transient). Letzten Wert halten, bis das Fenster abgelaufen ist.
    const double settle = this->get_parameter("power_on_settle_s").as_double();
    if (have_power_on_time_ && (this->now() - power_on_time_).seconds() < settle) {
      if (have_valid_) {
        have_valid_ = false;
        RCLCPP_WARN(this->get_logger(),
          "RG6-JSB: Settle-Fenster nach Tool-Bestromung (%.1fs) -> AI2-Transient abwarten, halte letzten Wert.",
          settle);
      }
      return;  // position_ unveraendert
    }

    // Plausibilitaets-Gate: der RG6 treibt AI2 erst, NACHDEM er nach dem Einschalten
    // sein erstes Bewegungskommando bekommen hat - davor liegt AI2 bei ~0 V (unter
    // dem Zu-Wert in_closed=0.56). Diese 0 V sind KEINE Weite: linear abgebildet
    // landen sie durch die Klemmung auf 'closed' -> falscher Ist-Zustand UND, weil
    // move_group von hier seinen Startzustand liest, ein vergifteter MoveIt-Start
    // (Planung vom Scheinwert -> Greifer laesst sich in MoveIt nicht bewegen).
    // Darum unter der Totzone den letzten guten Wert HALTEN statt zu mappen.
    const double dead = this->get_parameter("dead_input_threshold").as_double();
    if (analog_input < dead) {
      if (have_valid_) {
        have_valid_ = false;  // hatten Feedback -> Tool jetzt vermutlich stromlos
        RCLCPP_WARN(this->get_logger(),
          "RG6-JSB: analog_input2=%.3f < %.3f V -> kein gueltiges Weiten-Feedback "
          "(Tool stromlos / vor erstem Kommando). Halte letzten Wert.",
          analog_input, dead);
      }
      return;  // position_ unveraendert (Startwert = angle_closed)
    }
    if (!have_valid_) {
      have_valid_ = true;
      RCLCPP_INFO(this->get_logger(),
        "RG6-JSB: gueltiges Weiten-Feedback (analog_input2=%.3f V) -> tracke Ist-Weite.",
        analog_input);
    }

    double pos = angle_closed;
    const double span_in = in_open - in_closed;
    if (std::abs(span_in) > 1e-9) {
      pos = angle_closed + (analog_input - in_closed) * (angle_open - angle_closed) / span_in;
    }

    // Klemmen: ein unerwarteter analog_input2-Wert (falsche Einheit/Bereich) darf
    // das Gelenk nicht ueber den gueltigen Bereich hinaus treiben.
    const double lo = std::min(angle_closed, angle_open);
    const double hi = std::max(angle_closed, angle_open);
    position_ = std::clamp(pos, lo, hi);
  }

  void publish()
  {
    // ALLE 6 Greifergelenke explizit publizieren. Der robot_state_publisher wertet
    // die <mimic>-Tags des Modells nicht aus -> wuerde 'rg6_finger_joint' allein nur
    // left_outer_knuckle setzen, der Rest faellt auf den Ursprung ("Teile liegen
    // nebeneinander"). Faktoren = die mimic-multiplier aus onrobot_rg6_model_macro:
    //   finger_joint (Treiber)        : +1
    //   *_inner_knuckle_joint         : -1
    //   *_inner_finger_joint          : +1
    //   right_outer_knuckle_joint     : -1
    const double t = position_;
    auto joint_msg = sensor_msgs::msg::JointState();
    joint_msg.header.stamp = this->get_clock()->now();
    joint_msg.name = {
      "rg6_finger_joint",
      "rg6_left_inner_knuckle_joint",
      "rg6_left_inner_finger_joint",
      "rg6_right_outer_knuckle_joint",
      "rg6_right_inner_knuckle_joint",
      "rg6_right_inner_finger_joint",
    };
    joint_msg.position = { t, -t, t, -t, -t, t };
    pub_->publish(joint_msg);
  }

  double position_{0.0};
  bool have_valid_{false};  // schon mind. ein plausibles AI2-Feedback gesehen?
  double prev_tool_voltage_{0.0};     // fuer steigende Flanke der Tool-Spannung
  rclcpp::Time power_on_time_{0, 0};  // Zeitpunkt der letzten 0->24V-Flanke
  bool have_power_on_time_{false};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Subscription<ur_msgs::msg::ToolDataMsg>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ToolDataToJointStateNode>());
  rclcpp::shutdown();
  return 0;
}
