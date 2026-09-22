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

class AngleControllerNode : public rclcpp::Node
{
public:
  AngleControllerNode()
  : rclcpp::Node("angle_controller"), pid_(1.0, 0.0, 0.0)
  {
    angle_subscription_ = create_subscription<std_msgs::msg::Float64>(
      "/motor/angle",
      10,
      [this](const std_msgs::msg::Float64 & message) {
        angle_callback(message);
      });

    velocity_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "/motor/velocity_cmd",
      10);

    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(2),
      [this]() {
        control_update();
      });
  }

private:
  void angle_callback(const std_msgs::msg::Float64 & message)
  {
    current_angle_ = message.data;

    if (first_angle_) {
      choose_major_arc();
      first_angle_ = false;
    }
  }

  void choose_major_arc()
  {
    double pi = 3.1415926;
    double error = target_angle_ - current_angle_;

    while (error > 2.0 * pi) {
      error -= 2.0 * pi;
    }
    while (error < -2.0 * pi) {
      error += 2.0 * pi;
    }

    if (error > 0.0 && error < pi) {
      error -= 2.0 * pi;
    } else if (error < 0.0 && error > -pi) {
      error += 2.0 * pi;
    }

    target_angle_ = current_angle_ + error;
    RCLCPP_INFO(get_logger(), "major arc: %.3f rad", error);
  }

  void control_update()
  {
    if (first_angle_) {
      return;
    }

    target_velocity_ = pid_.update(
      target_angle_, current_angle_, control_dt_);

    if (target_velocity_ > 4.0) {
      target_velocity_ = 4.0;
    }
    if (target_velocity_ < -4.0) {
      target_velocity_ = -4.0;
    }

    std_msgs::msg::Float64 velocity_message;
    velocity_message.data = target_velocity_;
    velocity_publisher_->publish(velocity_message);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      500,
      "target angle: %.3f | current angle: %.3f | target velocity: %.3f",
      target_angle_, current_angle_, target_velocity_);
  }

  PidController pid_;

  double target_angle_{1.5 * 3.1415926};
  double current_angle_{0.0};
  double target_velocity_{0.0};
  double control_dt_{0.002};
  bool first_angle_{true};

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AngleControllerNode>();
  RCLCPP_INFO(node->get_logger(), "Angle controller node started");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
