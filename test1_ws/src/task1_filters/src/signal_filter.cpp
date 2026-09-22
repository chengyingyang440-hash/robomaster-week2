#include <algorithm>
#include <deque>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("signal_filter");

    double low_pass = 0.0;
    auto low_pass_publisher = node->create_publisher<std_msgs::msg::Float64>(
        "/signal/low_pass", 10
    );

    std::deque<double> median_window;
    auto median_publisher = node->create_publisher<std_msgs::msg::Float64>(
        "/signal/median", 10
    );
    auto subscriber = node->create_subscription<std_msgs::msg::Float64>(
        "/signal/noisy", 10,
        [node, &low_pass, low_pass_publisher, &median_window, median_publisher](const std_msgs::msg::Float64::SharedPtr msg) {
            const double raw = msg->data;
            const double alpha = 0.2;
            low_pass = alpha * raw + (1.0 - alpha) * low_pass;
            std_msgs::msg::Float64 filtered_msg;
            filtered_msg.data = low_pass;
            low_pass_publisher->publish(filtered_msg);
            median_window.push_back(raw);
            if (median_window.size() > 5) {
                median_window.pop_front();
            }
            if (median_window.size() == 5) {
                std::vector<double> sorted_values(median_window.begin(), median_window.end());
                std::sort(sorted_values.begin(), sorted_values.end());

                std_msgs::msg::Float64 median_msg;
                median_msg.data = sorted_values[2];
                median_publisher->publish(median_msg);
            }
        }
    );
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
