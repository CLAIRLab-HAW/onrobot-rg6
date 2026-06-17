#include "rclcpp/rclcpp.hpp"
#include "ur_msgs/msg/tool_data_msg.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#define min_in 0.56
#define max_in 10.0
#define min_angle -0.62
#define max_angle 0.6

class ToolDataToJointStateNode : public rclcpp::Node
{
public:
  ToolDataToJointStateNode() : Node("tooldata_to_jointstate")
  {
    // Relative Namen -> aufloesbar im Namespace (z.B. /a200_0553/manipulators).
    // 'joint_states' (relativ) landet damit auf dem joint_states-Topic, das der
    // Manipulator-robot_state_publisher konsumiert -> Greifer wird mit animiert.
    // WICHTIG: setzt voraus, dass das Greifer-Gelenk im URDF REVOLUTE+mimic ist
    // und exakt so heisst wie joint_msg.name unten.
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    sub_ = this->create_subscription<ur_msgs::msg::ToolDataMsg>(
      "io_and_status_controller/tool_data", rclcpp::SensorDataQoS(),
      std::bind(&ToolDataToJointStateNode::tooldata_callback, this, std::placeholders::_1));
  }

private:
  void tooldata_callback(const ur_msgs::msg::ToolDataMsg::SharedPtr msg)
  {
    auto joint_msg = sensor_msgs::msg::JointState();
    
    auto analog_input = msg->analog_input2;
    double gripper_position = min_angle + (analog_input - min_in) * (max_angle - min_angle) / (max_in - min_in);

    // Klemmen: ein unerwarteter analog_input2-Wert (falsche Einheit/Bereich) darf
    // das Gelenk nicht ueber die URDF-Limits hinaus treiben -> Modell bliebe sonst
    // "zerpflueckt". min_angle < max_angle.
    if (gripper_position < min_angle) gripper_position = min_angle;
    if (gripper_position > max_angle) gripper_position = max_angle;

    joint_msg.header.stamp = this->get_clock()->now();
    joint_msg.name = {"rg6-l_out_joint"};
    joint_msg.position = {gripper_position};

    pub_->publish(joint_msg);
  }

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

