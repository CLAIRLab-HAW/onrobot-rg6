#include <rclcpp/rclcpp.hpp>
#include "ur_msgs/srv/set_io.hpp"
#include "ur_msgs/msg/tool_data_msg.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#define min_angle -0.62
#define max_angle 0.6
#define angle_tolerance 0.005
#define gripper_open 0
#define gripper_close 1
#define io_fun_write 1
#define io_out_fun_pin 16
#define io_out_speed_pin 17
class RG6ControlNode : public rclcpp::Node{
  public:
    RG6ControlNode() : Node("rg6_control_node")
    {
      open_gripper_service_= this->create_service<std_srvs::srv::Trigger>("rg6_control/open_gripper", std::bind(&RG6ControlNode::open_callback, this, std::placeholders::_1, std::placeholders::_2));
      close_gripper_service_ = this->create_service<std_srvs::srv::Trigger>("rg6_control/close_gripper", std::bind(&RG6ControlNode::close_callback, this, std::placeholders::_1, std::placeholders::_2));
      io_client_ = this->create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");
    }
  private:
    void open_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      send_io_command(io_fun_write, io_out_fun_pin, gripper_open, response);
    }
    void close_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      send_io_command(io_fun_write, io_out_fun_pin, gripper_close, response);
    }
    void send_io_command(int fun, int pin, int state, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      // checking io service for availability and sending io request
      RCLCPP_INFO(this->get_logger(), "executing command %i started", state);
      auto req = std::make_shared<ur_msgs::srv::SetIO::Request>();
      req->fun = fun;
      req->pin = pin;
      req->state = state;
      if(!io_client_->wait_for_service(std::chrono::seconds(1)))
      {
        response->success = false; 
        response->message = "UR IO service is not available";
        return;
      }
      auto result = io_client_->async_send_request(req);
 
      // sub-node checking if gripper joint reached target position
      RCLCPP_INFO(this->get_logger(), "creating gripper_monitor node");
      rclcpp::Node::SharedPtr gripper_monitor = rclcpp::Node::make_shared("gripper_monitor_temp");
      std::mutex mutex;
      std::condition_variable cv;
      double target_angle = (state == gripper_open) ? max_angle : min_angle;
      bool target_reached = false;
      RCLCPP_INFO(this->get_logger(), "creating gripper_monitor subscription");
      auto sub = gripper_monitor->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, 
        [&gripper_monitor, target_angle, &mutex, &target_reached, &cv](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) 
        {
          auto it = std::find(msg->name.begin(), msg->name.end(), "rg6-l_out_joint");
          if (it != msg->name.end()) {
            double current_angle = msg->position[std::distance(msg->name.begin(), it)];
              //RCLCPP_INFO(gripper_monitor->get_logger(), "gripper_monitor: current_angle %f", current_angle);
            if (std::abs(current_angle - target_angle) < angle_tolerance && !target_reached) {
              //RCLCPP_INFO(gripper_monitor->get_logger(), "gripper_monitor: target reached current_angle %f", current_angle);
              target_reached = true;
              std::lock_guard<std::mutex> lock(mutex);

              cv.notify_all();
            }
          }
        });
      {
        auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor->add_node(gripper_monitor);
        
        // spinning gripper_monitor and chechking for timeout 
        std::unique_lock<std::mutex> lock(mutex);
        auto start = this->now();
        while (rclcpp::ok() && !target_reached && (this->now() - start) < rclcpp::Duration::from_seconds(10.0))
        {
          lock.unlock();
          executor->spin_once(std::chrono::milliseconds(20));  // Processes sub-node callbacks
          lock.lock();
          if (cv.wait_for(lock, std::chrono::milliseconds(20), [&target_reached]
          {
            return target_reached;
          }
            )) 
          {
            break;
          }
        }
      }
      gripper_monitor.reset();
      response->success = target_reached;
      response->message =  ("Gripper %s confirmed", target_reached ? "OK" : "TIMEOUT");
      RCLCPP_INFO(this->get_logger(), "Gripper %s confirmed", target_reached ? "OK" : "TIMEOUT");
    }
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr open_gripper_service_, close_gripper_service_;
    rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr io_client_;
    };
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RG6ControlNode>());
  rclcpp::shutdown();
  return 0;
}


