#include <algorithm>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "ur_msgs/msg/tool_data_msg.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// Mappt tool_data.analog_input2 (Greiferweite) linear auf den Treiber-Gelenkwinkel
// 'rg6_finger_joint' (Modell onrobot_rg6_visualization) und publiziert ihn als
// joint_states -> Greifer-Animation. Konvention: 0 = offen, positiv = zu.
//
// Die 4 Mapping-Werte sind LIVE-Parameter (in Foxglove justierbar):
//   in_closed / in_open  : analog_input2 bei physisch zu / auf  (ros2 topic echo tool_data)
//   angle_closed/angle_open : zugehoeriger rg6_finger_joint-Winkel (0=offen, +=zu)
// Live tunen, z.B.:
//   ros2 param set /a200_0553/manipulators/tooldata_to_jointstate angle_closed 0.6
//   ros2 param set /a200_0553/manipulators/tooldata_to_jointstate angle_open   0.0
// Wenn passend: die Defaults hier eintragen.

class ToolDataToJointStateNode : public rclcpp::Node
{
public:
  ToolDataToJointStateNode() : Node("tooldata_to_jointstate")
  {
    // Startwerte = grobe Schaetzung; bitte live nachjustieren (siehe oben).
    in_closed_    = this->declare_parameter<double>("in_closed", 0.56);
    in_open_      = this->declare_parameter<double>("in_open", 10.0);
    angle_closed_ = this->declare_parameter<double>("angle_closed", 0.6);
    angle_open_   = this->declare_parameter<double>("angle_open", 0.0);

    // Relative Namen -> im Namespace aufloesbar (z.B. /a200_0553/manipulators).
    // 'joint_states' wird per Launch-Remap auf das Topic des Greifer-rendernden
    // robot_state_publisher gelegt (a200-0553: /a200_0553/platform/joint_states).
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    sub_ = this->create_subscription<ur_msgs::msg::ToolDataMsg>(
      "io_and_status_controller/tool_data", rclcpp::SensorDataQoS(),
      std::bind(&ToolDataToJointStateNode::tooldata_callback, this, std::placeholders::_1));
  }

private:
  void tooldata_callback(const ur_msgs::msg::ToolDataMsg::SharedPtr msg)
  {
    // Live-Parameter pro Sample lesen -> sofort wirksam beim Tunen.
    const double in_closed    = this->get_parameter("in_closed").as_double();
    const double in_open      = this->get_parameter("in_open").as_double();
    const double angle_closed = this->get_parameter("angle_closed").as_double();
    const double angle_open   = this->get_parameter("angle_open").as_double();

    const double analog_input = msg->analog_input2;
    double gripper_position = angle_closed;
    const double span_in = in_open - in_closed;
    if (std::abs(span_in) > 1e-9) {
      gripper_position = angle_closed +
        (analog_input - in_closed) * (angle_open - angle_closed) / span_in;
    }

    // Klemmen: ein unerwarteter analog_input2-Wert (falsche Einheit/Bereich) darf
    // das Gelenk nicht ueber den gueltigen Bereich hinaus treiben -> sonst
    // "zerpflueckt". Reihenfolge-unabhaengig (angle_closed/angle_open beliebig).
    const double lo = std::min(angle_closed, angle_open);
    const double hi = std::max(angle_closed, angle_open);
    gripper_position = std::clamp(gripper_position, lo, hi);

    auto joint_msg = sensor_msgs::msg::JointState();
    joint_msg.header.stamp = this->get_clock()->now();
    joint_msg.name = {"rg6_finger_joint"};
    joint_msg.position = {gripper_position};
    pub_->publish(joint_msg);
  }

  double in_closed_, in_open_, angle_closed_, angle_open_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Subscription<ur_msgs::msg::ToolDataMsg>::SharedPtr sub_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ToolDataToJointStateNode>());
  rclcpp::shutdown();
  return 0;
}
