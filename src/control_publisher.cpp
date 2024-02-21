#include <memory>
#include <string>

#include "suiryoku/locomotion/control/node/control_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "suiryoku/locomotion/locomotion.hpp"
#include "nlohmann/json.hpp"

class ControlPublisher : public rclcpp::Node
{
public:
  ControlPublisher() : Node("control_publisher"){
    run_locomotion_publisher = this->create_publisher<suiryoku_interfaces::msg::RunLocomotion>(
      "locomotion/control/run_locomotion", 1);
    suiryoku_interfaces::msg::RunLocomotion beziercommand;
    beziercommand.command = 9;
    nlohmann::json json_params;
    json_params["direction"] = -M_PI_2;
    json_params["target"]["x"] = 2000.0;
    json_params["target"]["y"] = 2000.0;
    beziercommand.parameters = json_params.dump();
    run_locomotion_publisher->publish(beziercommand);
  }
private:
  rclcpp::Publisher<suiryoku_interfaces::msg::RunLocomotion>::SharedPtr run_locomotion_publisher;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // std::shared_ptr<ControlPublisher> node;
  rclcpp::spin(std::make_shared<ControlPublisher>());
  rclcpp::shutdown();
  return 0;
}