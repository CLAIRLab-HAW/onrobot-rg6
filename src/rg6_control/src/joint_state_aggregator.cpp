#include <algorithm>
#include <chrono>
#include <cstddef>
#include <map>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// Aggregiert mehrere PARTIELLE joint_states-Quellen (Raeder / Arm / Greifer) zu
// EINEM vollstaendigen Snapshot-Topic.
//
// Warum noetig: Der a200_0553 laeuft mit ZWEI controller_managern (Platform +
// manipulators) plus einem synthetischen Greifer-Node -> drei getrennte, jeweils
// partielle joint_states. ros2_control aggregiert nur INNERHALB eines CM
// automatisch; ueber CM-Grenzen hinweg gibt es keinen gemeinsamen Broadcaster.
//
// Dieser Node haelt je Gelenk den zuletzt gesehenen (position, velocity, effort)
// und published getaktet die VEREINIGUNG aller je gesehenen Gelenke. Er erhaelt
// velocity UND effort (anders als joint_state_publisher, der nur Position kann).
//
// GEDACHT ALS BEOBACHTUNGS-/RECORDING-TOPIC (rosbag/Foxglove), NICHT als Live-
// TF-Feed fuer den robot_state_publisher: ein Aggregator im TF-Pfad waere ein
// Single-Point-of-Failure, waehrend der gecachte platform/joint_states-Bus bei
// Ausfall einer Quelle graceful degradiert (nur deren Gelenke veralten).

using namespace std::chrono_literals;

class JointStateAggregator : public rclcpp::Node
{
public:
  JointStateAggregator() : Node("joint_state_aggregator")
  {
    // Relative Namen -> im Node-Namespace (/a200_0553) aufloesbar.
    const std::vector<std::string> default_sources = {
      "platform/joint_states",
      "manipulators/joint_states",
      "manipulators/endeffectors/joint_states",
    };
    const auto sources = this->declare_parameter("source_topics", default_sources);
    const auto output = this->declare_parameter<std::string>("output_topic", "joint_states");
    const double rate = this->declare_parameter<double>("publish_rate", 50.0);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "");

    pub_ = this->create_publisher<sensor_msgs::msg::JointState>(output, rclcpp::QoS(10));

    for (const auto & topic : sources) {
      subs_.push_back(this->create_subscription<sensor_msgs::msg::JointState>(
        topic, rclcpp::QoS(10),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { this->on_source(*msg); }));
      RCLCPP_INFO(this->get_logger(), "aggregiere Quelle: %s", topic.c_str());
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { this->publish(); });
  }

private:
  struct JointVal
  {
    double pos{0.0};
    double vel{0.0};
    double eff{0.0};
    bool has_vel{false};
    bool has_eff{false};
  };

  void on_source(const sensor_msgs::msg::JointState & msg)
  {
    // velocity/effort sind optional: nur uebernehmen, wenn die Arrays zur
    // Namensliste passen (der Greifer-Node z.B. liefert nur Position).
    const bool has_vel = msg.velocity.size() == msg.name.size();
    const bool has_eff = msg.effort.size() == msg.name.size();
    for (std::size_t i = 0; i < msg.name.size(); ++i) {
      auto & j = joints_[msg.name[i]];
      if (i < msg.position.size()) {
        j.pos = msg.position[i];
      }
      if (has_vel) {
        j.vel = msg.velocity[i];
        j.has_vel = true;
      }
      if (has_eff) {
        j.eff = msg.effort[i];
        j.has_eff = true;
      }
    }
  }

  void publish()
  {
    if (joints_.empty()) {
      return;  // noch keine Quelle gesehen -> nichts (Phantom-Nullen vermeiden)
    }

    // velocity/effort-Arrays nur ausgeben, wenn ueberhaupt eine Quelle sie liefert;
    // JointState verlangt Arrays gleicher Laenge wie name (oder leer).
    bool any_vel = false;
    bool any_eff = false;
    for (const auto & [name, j] : joints_) {
      (void)name;
      any_vel = any_vel || j.has_vel;
      any_eff = any_eff || j.has_eff;
    }

    sensor_msgs::msg::JointState out;
    out.header.stamp = this->get_clock()->now();
    out.header.frame_id = frame_id_;
    out.name.reserve(joints_.size());
    out.position.reserve(joints_.size());
    if (any_vel) {
      out.velocity.reserve(joints_.size());
    }
    if (any_eff) {
      out.effort.reserve(joints_.size());
    }

    for (const auto & [name, j] : joints_) {
      out.name.push_back(name);
      out.position.push_back(j.pos);
      if (any_vel) {
        out.velocity.push_back(j.vel);  // 0.0 fuer Quellen ohne velocity (z.B. Greifer)
      }
      if (any_eff) {
        out.effort.push_back(j.eff);
      }
    }
    pub_->publish(out);
  }

  std::map<std::string, JointVal> joints_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> subs_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string frame_id_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateAggregator>());
  rclcpp::shutdown();
  return 0;
}
