#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <memory>

void publish_torque(
    const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr & publisher,
    const rclcpp::Time & start_time,
    const rclcpp::Node::SharedPtr& node
    ){
        std_msgs::msg::Float64 message;

        double passed_times =(node->now() - start_time).seconds();

         if (passed_times < 2.0) {
            message.data = 0.0;
        }
         else if (passed_times < 5.0) {
            message.data = 1.0;
        }
        else {
        message.data = 0.0;
        }

        publisher->publish(message);
      }

int main (int argc, char* argv[]){
    rclcpp::init(argc,argv);

    auto node = std::make_shared<rclcpp::Node>("torque_source");

    auto torque_publisher = node->create_publisher<std_msgs::msg::Float64>(
        "/motor/torque_cmd",
        10
    );

    auto start_time = node->now();

    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(2),
        [node, torque_publisher, start_time]() {
            publish_torque(torque_publisher, start_time, node);
        }
    );

    RCLCPP_INFO(node->get_logger(), "Torque source node started");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}