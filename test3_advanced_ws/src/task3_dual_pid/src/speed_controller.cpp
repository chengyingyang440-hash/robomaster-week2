#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class PidController
{
public:
  PidController(double kp, double ki, double kd)
  {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  double update(double target, double current, double dt)
  {
    double error = target - current;

    double proportional = calculate_proportional(error);
    double integral = calculate_integral(error, dt);
    double derivative = calculate_derivative(error, dt);

    return proportional + integral + derivative;
  }

private:
  double calculate_proportional(double error)
  {
    return kp_ * error;
  }

  double calculate_integral(double error, double dt)
  {
    integral_error_ += error * dt;
    return ki_ * integral_error_;
  }

  double calculate_derivative(double error, double dt)
  {
    if (dt <= 0.0) {
      return 0.0;
    }

    if (first_update_) {
      previous_error_ = error;
      first_update_ = false;
      return 0.0;
    }

    double derivative_error = error - previous_error_;
    previous_error_ = error;
    return kd_ * derivative_error / dt;
  }

  double kp_;
  double ki_;
  double kd_;
  double integral_error_{0.0};
  double previous_error_{0.0};
  bool first_update_{true};
};

class SpeedControllerNode : public rclcpp::Node
{
public:
  SpeedControllerNode()
  : rclcpp::Node("speed_controller"), pid_(0.2, 0.5, 0.0)
  {
    velocity_subscription_ = create_subscription<std_msgs::msg::Float64>(
      "/motor/angular_velocity",
      10,
      [this](const std_msgs::msg::Float64 & message) {
        velocity_callback(message);
      });

    target_velocity_subscription_ = create_subscription<std_msgs::msg::Float64>(
      "/motor/velocity_cmd",
      10,
      [this](const std_msgs::msg::Float64 & message) {
        target_velocity_callback(message);
      });

    torque_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "/motor/torque_cmd",
      10);

    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(2),
      [this]() {
        control_update();
      });
  }

private:
  void velocity_callback(const std_msgs::msg::Float64 & message)
  {
    current_velocity_ = message.data;
  }

  void target_velocity_callback(const std_msgs::msg::Float64 & message)
  {
    target_velocity_ = message.data;
  }

  void control_update()
  {
    control_torque_ = pid_.update(
      target_velocity_, current_velocity_, control_dt_);

    std_msgs::msg::Float64 torque_message;
    torque_message.data = control_torque_;
    torque_publisher_->publish(torque_message);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      500,
      "target velocity: %.3f | current velocity: %.3f | torque: %.3f",
      target_velocity_, current_velocity_, control_torque_);
  }

  PidController pid_;

  double target_velocity_{0.0};
  double current_velocity_{0.0};
  double control_dt_{0.002};
  double control_torque_{0.0};

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_velocity_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr torque_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SpeedControllerNode>();
  RCLCPP_INFO(node->get_logger(), "Speed controller node started");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
