#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <chrono>
#include <cmath>
#include <random>

int main(int argc,char* argv[]){
    rclcpp::init(argc,argv);

    auto node = std::make_shared<rclcpp::Node>(
        "signal_generator"
    );

    auto publisher =node->create_publisher<std_msgs::msg::Float64>("/signal/noisy",10);

    const auto start_time = std::chrono::steady_clock::now();

    std::mt19937 rng(std::random_device{}());
    std::normal_distribution<double> noise(0.0, 0.01);

    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(1),
        [publisher,start_time,&rng,&noise](){
            const double passed_time = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start_time
            ).count();

            std_msgs::msg::Float64 message;
            message.data = 1*std::sin(2*3.141592653589793*20*passed_time)+noise(rng);
            publisher->publish(message);

        }

    );

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}