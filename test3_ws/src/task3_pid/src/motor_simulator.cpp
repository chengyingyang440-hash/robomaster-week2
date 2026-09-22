#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class MotorSimulatorNode : public rclcpp::Node
{
public:
  MotorSimulatorNode()
  : rclcpp::Node("motor_simulator")
  {
    torque_subscription_ =
      create_subscription<std_msgs::msg::Float64>(
      "/motor/torque_cmd",
      10,
      [this](const std_msgs::msg::Float64 & message) {
        torque_callback(message);
      });


      update_timer_ = create_wall_timer(
        std::chrono::milliseconds(1),
        [this](){
          update_motor_state();
        }
      );

      angle_publisher_ =
        create_publisher<std_msgs::msg::Float64>(
          "/motor/angle",
          10
        );

      angular_velocity_publisher_ =
        create_publisher<std_msgs::msg::Float64>(
         "/motor/angular_velocity",
          10
        );
  }

private:
  void torque_callback(const std_msgs::msg::Float64 & message)
  {
    torque_ = message.data;
  }

  void update_motor_state()
  {
    const double angular_acceleration =
      (torque_ - load_torque_ -
      damping_coefficient_ * angular_velocity_) /
      moment_of_inertia_;

    angle_ += angular_velocity_ * time_step_;
    angular_velocity_ += angular_acceleration * time_step_;

    std_msgs::msg::Float64 angular_velocity_message;
    angular_velocity_message.data = angular_velocity_;

    angular_velocity_publisher_->publish(angular_velocity_message);

    std_msgs::msg::Float64 angle_message;
    angle_message.data = angle_;

    angle_publisher_->publish(angle_message);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      500,
      "torque: %.3f N*m | velocity: %.3f rad/s | angle: %.3f rad",
      torque_,
      angular_velocity_,
      angle_);
  }

  double torque_{0.0};
  double angular_velocity_{0.0};
  double angle_{0.0};

  double moment_of_inertia_{0.01};
  double damping_coefficient_{0.1};
  double load_torque_{0.0};
  double time_step_{0.001};

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr torque_subscription_;

  rclcpp::TimerBase::SharedPtr update_timer_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angular_velocity_publisher_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MotorSimulatorNode>();

  RCLCPP_INFO(node->get_logger(), "Motor simulator node started");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
