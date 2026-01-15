#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher")
    {
      // Corrected the template type to match Float32MultiArray
      publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/camera_angles", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
      
      // Seed for random number generation
      std::srand(std::time(nullptr));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::Float32MultiArray();
      
      // Logic to populate 6 random integers between 0 and 180
      for(int i = 0; i < 6; i++) {
        message.data.push_back(static_cast<float>(std::rand() % 181));
      }

      RCLCPP_INFO(this->get_logger(), "Publishing array of 6 random values");
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    // Changed String to Float32MultiArray to match the message type
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}