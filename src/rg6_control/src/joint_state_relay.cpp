#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// Leitet partielle joint_states von mehreren Quell-Topics VERBATIM auf EIN
// Ziel-Topic weiter, mit EXPLIZIT RELIABLE Publisher-QoS.
//
// Warum eigener Node statt `topic_tools relay`:
// topic_tools relay publiziert je nach Version mit best-effort/SensorDataQoS.
// move_group abonniert platform/joint_states aber RELIABLE -> ein best-effort-
// Publisher wird dort NICHT empfangen. Der robot_state_publisher abonniert
// best-effort und bekommt die Daten sehr wohl -> TF/Anzeige ok, ABER MoveIt
// bekommt den Arm-Zustand nicht -> Planning schlaegt fehl (klassisches Symptom
// "Zustand korrekt angezeigt, Planning failt"). Ein RELIABLE-Publisher bedient
// BEIDE: reliable move_group UND best-effort robot_state_publisher.

class JointStateRelay : public rclcpp::Node
{
public:
  JointStateRelay() : Node("joint_state_relay")
  {
    // Relative Namen -> im Node-Namespace (/a200_0553) aufloesbar.
    const std::vector<std::string> default_inputs = {
      "manipulators/joint_states",
      "manipulators/endeffectors/joint_states",
    };
    const auto inputs = this->declare_parameter("input_topics", default_inputs);
    const auto output = this->declare_parameter<std::string>("output_topic", "platform/joint_states");
    const int depth = this->declare_parameter<int>("depth", 20);

    // RELIABLE + VOLATILE + KEEP_LAST: kompatibel mit dem reliable move_group-
    // Subscriber UND dem best-effort robot_state_publisher-Subscriber.
    rclcpp::QoS qos(rclcpp::KeepLast(static_cast<std::size_t>(std::max(1, depth))));
    qos.reliable().durability_volatile();

    pub_ = this->create_publisher<sensor_msgs::msg::JointState>(output, qos);
    for (const auto & topic : inputs) {
      subs_.push_back(this->create_subscription<sensor_msgs::msg::JointState>(
        topic, qos,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { pub_->publish(*msg); }));
      RCLCPP_INFO(this->get_logger(), "relay %s -> %s", topic.c_str(), pub_->get_topic_name());
    }
  }

private:
  std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> subs_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateRelay>());
  rclcpp::shutdown();
  return 0;
}
