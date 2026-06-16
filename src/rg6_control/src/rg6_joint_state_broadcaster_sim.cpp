#include "rclcpp/rclcpp.hpp"
#include "ur_msgs/msg/io_states.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>
#define CALC_OFFSET 0.6
#define max_angle 0.6
#define min_angle -0.62

class RG6_SimNode : public rclcpp::Node
{
public:
  RG6_SimNode() : Node("RG6_SimNode")
  {
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/gripper/joint_states", 10);
    sub_ = this->create_subscription<ur_msgs::msg::IOStates>(
      "/io_and_status_controller/io_states", 10,
      std::bind(&RG6_SimNode::io_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RG6_SimNode::timer_callback, this));
    timer_->cancel();
  }

private:
  void io_callback(const ur_msgs::msg::IOStates::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto joint_msg = sensor_msgs::msg::JointState();
 
    bool state = msg->digital_out_states[16].state;
    if (last_state_ != state)
    {
      last_state_ = state;
      timer_->reset();
    }


    joint_msg.header.stamp = this->get_clock()->now();
    joint_msg.name = {"rg6-l_out_joint"};
    joint_msg.position = {gripper_position_};
    last_state_ = state;
    pub_->publish(joint_msg);
  }

  void timer_callback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gripper_position_ = last_state_ ? (std::max((gripper_position_ - 0.05), min_angle)) : (std::min((gripper_position_ + 0.05), max_angle));
    if (gripper_position_ == max_angle || gripper_position_ == min_angle)
    {
      timer_->cancel();
      timer_active_ = false;
    }
    RCLCPP_INFO(this->get_logger(), "simulating gripper %f", gripper_position_);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Subscription<ur_msgs::msg::IOStates>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double gripper_position_ = max_angle;
  bool last_state_ = false;
  bool timer_active_ = false;
  std::mutex mutex_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RG6_SimNode>());
  rclcpp::shutdown();
  return 0;
}

