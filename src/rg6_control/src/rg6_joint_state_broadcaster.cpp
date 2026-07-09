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
// dem letzten bekannten Wert, Start = angle_open). Sonst fehlt ohne tool_data die
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

    // Start = offen, damit das Modell sofort (auch ohne tool_data) zusammengebaut ist.
    position_ = this->get_parameter("angle_open").as_double();

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
      return;  // position_ unveraendert (Startwert = angle_open)
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
