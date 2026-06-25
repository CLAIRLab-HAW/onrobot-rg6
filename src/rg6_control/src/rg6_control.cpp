#include <rclcpp/rclcpp.hpp>
#include "ur_msgs/srv/set_io.hpp"
#include "ur_msgs/msg/tool_data_msg.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <cmath>
#include <memory>
#include <mutex>

// --- UR Tool-IO Belegung des RG6 ---
#define io_fun_write 1        // ur_msgs SetIO: FUN_SET_DIGITAL_OUT
#define io_out_fun_pin 16     // Tool DO0: auf/zu
#define io_out_speed_pin 17   // Tool DO1: Kraft/Speed-Select (hier ungenutzt belassen)
#define gripper_open 0
#define gripper_close 1

// --- Bewegungserkennung (ueber tool_data) ---
// analog_input3 = Kraft/Strom, analog_input2 = Greifweite (roh).
// HINWEIS: Diese Schwellen sind hardware-abhaengig -> ggf. nachkalibrieren.
#define force_threshold 3.1   // ab hier gilt Bewegung/Kontakt als "gestartet"
#define move_eps 0.10         // Mindest-Positionsaenderung (roh), die als Bewegung zaehlt
#define settle_eps 0.05       // Position gilt als "stabil", wenn Aenderung < settle_eps
#define settle_time_s 0.4     // ... fuer mindestens diese Zeit
#define motion_timeout_s 10.0 // Gesamt-Timeout


class RG6ControlNode : public rclcpp::Node
{
public:
  RG6ControlNode() : Node("rg6_control_node")
  {
    // Relative Namen -> aufloesbar im jeweiligen Namespace (z.B. /a200_0553/manipulators)
    open_service_ = this->create_service<std_srvs::srv::Trigger>(
      "rg6_control/open",
      std::bind(&RG6ControlNode::open_callback, this, std::placeholders::_1, std::placeholders::_2));
    close_service_ = this->create_service<std_srvs::srv::Trigger>(
      "rg6_control/close",
      std::bind(&RG6ControlNode::close_callback, this, std::placeholders::_1, std::placeholders::_2));
    io_client_ = this->create_client<ur_msgs::srv::SetIO>("io_and_status_controller/set_io");

    // Tool-Spannung beim Start auf 24 V setzen (RG6-Stromversorgung). Der
    // io_and_status_controller wird vom ur_robot_driver-Stack hochgezogen und ist
    // u.U. erst spaeter verfuegbar -> aktiv pollen, bis set_io da ist (max. ~60 s),
    // statt nach einem einzigen Versuch aufzugeben.
    auto attempts = std::make_shared<int>(0);
    constexpr int max_attempts = 30;   // 30 * 2 s = 60 s
    voltage_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      [this, attempts, max_attempts]()
      {
        if (io_client_->wait_for_service(std::chrono::milliseconds(100)))
        {
          auto req = std::make_shared<ur_msgs::srv::SetIO::Request>();
          req->fun = 4;        // FUN_SET_TOOL_VOLTAGE
          req->pin = 0;
          req->state = 24.0f;
          io_client_->async_send_request(req);
          RCLCPP_INFO(this->get_logger(), "RG6: Tool-Spannung auf 24V gesetzt");
          voltage_timer_->cancel();
          return;
        }
        if (++(*attempts) >= max_attempts)
        {
          RCLCPP_ERROR(this->get_logger(),
            "RG6: set_io nach %d Versuchen nicht verfuegbar - Tool-Spannung (24V) "
            "NICHT gesetzt (io_and_status_controller aktiv?)", max_attempts);
          voltage_timer_->cancel();
          return;
        }
        RCLCPP_WARN(this->get_logger(),
          "RG6: warte auf io_and_status_controller/set_io (Versuch %d/%d) ...",
          *attempts, max_attempts);
      });
  }

private:
  void open_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    send_io_command(io_fun_write, io_out_fun_pin, gripper_open, response);
  }
  void close_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    send_io_command(io_fun_write, io_out_fun_pin, gripper_close, response);
  }

  void send_io_command(int fun, int pin, int state,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "RG6: command state=%d", state);

    auto req = std::make_shared<ur_msgs::srv::SetIO::Request>();
    req->fun = fun;
    req->pin = pin;
    req->state = static_cast<float>(state);

    if (!io_client_->wait_for_service(std::chrono::seconds(1)))
    {
      response->success = false;
      response->message = "UR IO service (io_and_status_controller/set_io) not available";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }
    // Fire-and-forget: die Bestaetigung kommt physisch ueber tool_data (unten).
    auto future = io_client_->async_send_request(req);
    (void)future;

    // Eigenes Monitor-Node + Executor, damit tool_data waehrend des (blockierenden)
    // Service-Callbacks verarbeitet wird, ohne den Haupt-Executor zu benoetigen.
    auto monitor = rclcpp::Node::make_shared("rg6_gripper_monitor");
    std::mutex m;
    bool have = false;
    double latest_pos = 0.0;
    double latest_force = 0.0;

    auto sub = monitor->create_subscription<ur_msgs::msg::ToolDataMsg>(
      "io_and_status_controller/tool_data", rclcpp::SensorDataQoS(),
      [&](const ur_msgs::msg::ToolDataMsg::ConstSharedPtr & msg)
      {
        std::lock_guard<std::mutex> lk(m);
        latest_pos = msg->analog_input2;
        latest_force = msg->analog_input3;
        have = true;
      });

    auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec->add_node(monitor);

    const auto start = this->now();
    bool have_initial = false;
    double initial_pos = 0.0;
    double settle_ref = 0.0;
    auto last_change = this->now();
    bool started = false;
    bool settled = false;

    while (rclcpp::ok() && (this->now() - start) < rclcpp::Duration::from_seconds(motion_timeout_s))
    {
      exec->spin_once(std::chrono::milliseconds(20));

      double pos, force;
      bool h;
      {
        std::lock_guard<std::mutex> lk(m);
        pos = latest_pos; force = latest_force; h = have;
      }
      if (!h) { continue; }

      if (!have_initial) { initial_pos = pos; settle_ref = pos; have_initial = true; last_change = this->now(); }

      // Bewegung gilt als gestartet, wenn Kraft steigt ODER sich die Position bewegt hat.
      if (!started && (force > force_threshold || std::fabs(pos - initial_pos) > move_eps))
      {
        started = true;
      }

      // Settling: solange sich die Position um mehr als settle_eps aendert, Timer zuruecksetzen.
      if (std::fabs(pos - settle_ref) > settle_eps)
      {
        settle_ref = pos;
        last_change = this->now();
      }

      // Fertig, sobald die Bewegung lief UND die Position lange genug stabil ist.
      // Das deckt BEIDES ab: freie Endlage erreicht ODER auf Objekt geklemmt (Position stabil,
      // Kraft hoch) -> in beiden Faellen Erfolg (frueherer Bug: Greifen am Objekt -> TIMEOUT).
      if (started && (this->now() - last_change) > rclcpp::Duration::from_seconds(settle_time_s))
      {
        settled = true;
        break;
      }
    }

    exec->remove_node(monitor);
    monitor.reset();

    response->success = settled;
    response->message = settled ? "Gripper motion settled (OK)"
                                : "Gripper motion did not settle (TIMEOUT)";
    RCLCPP_INFO(this->get_logger(), "RG6: %s", response->message.c_str());
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr open_service_, close_service_;
  rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr io_client_;
  rclcpp::TimerBase::SharedPtr voltage_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RG6ControlNode>());
  rclcpp::shutdown();
  return 0;
}
