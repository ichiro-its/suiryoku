#include <memory>
#include <string>

#include "suiryoku/locomotion/control/node/control_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "suiryoku/locomotion/locomotion.hpp"
#include "nlohmann/json.hpp"

using namespace std::chrono_literals;

class ControlPublisher : public rclcpp::Node
{
public:
  ControlPublisher() : Node("control_publisher"){
    timer_ = this->create_wall_timer(8ms, [this]() {
      run_locomotion_publisher = this->create_publisher<suiryoku_interfaces::msg::RunLocomotion>(
        "locomotion/control/run_locomotion", 1);
      suiryoku_interfaces::msg::RunLocomotion beziercommand;
      beziercommand.command = 9;
      nlohmann::json json_params;
      json_params["direction"] = -135;
      json_params["target"]["x"] = 700.0;
      json_params["target"]["y"] = 400.0;
      beziercommand.parameters = json_params.dump();
      run_locomotion_publisher->publish(beziercommand);
    });
  }
private:
  rclcpp::Publisher<suiryoku_interfaces::msg::RunLocomotion>::SharedPtr run_locomotion_publisher;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // std::shared_ptr<ControlPublisher> node;
  rclcpp::spin(std::make_shared<ControlPublisher>());
  rclcpp::shutdown();
  return 0;
}