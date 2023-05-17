// pub.cpp
// C++ Header files
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime> // Change

// Import the rclcpp client library and std_msgs header
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Include our custom message
#include "custom_msg/msg/log_tf.hpp"    // Change

using namespace std::chrono_literals;

// PublisherNode Composition
//We create a PublisherNode class that inherits from the rclcpp::Node
class PublisherNode: public rclcpp::Node
{
  public:
    PublisherNode()
    : Node("publisher"), count_(0)
    {
      // Create a Publisher with String message type and queue size is 10
      publisher_ = this->create_publisher<custom_msg::msg::LogTf>("string_msg", 10); // Change

      //Create a timer with 500ms delay
      timer_ = this->create_wall_timer(
      500ms, std::bind(&PublisherNode::callback, this));
    }

  private:
    void callback()
    {
      custom_msg::msg::LogTf message = custom_msg::msg::LogTf(); // Change
      time_t now = time(0); // Change
      message.time_frame = ctime(&now); // Change
      message.log_msg = "Hello, world! " + std::to_string(count_++); 
      
      // Publish the message
      publisher_->publish(message);
    }

    // Timer objects allow a node to perform a specific action at a specified rate or at a specific time in the future
    rclcpp::TimerBase::SharedPtr timer_;

    // Publisher
    rclcpp::Publisher<custom_msg::msg::LogTf>::SharedPtr publisher_; // Change

    // Variable to count number of message published
    size_t count_;
};

int main(int argc, char * argv[])
{
  // Initialize the ROS2 communication
  rclcpp::init(argc, argv);

  // Create a default single-threaded executor and spin the PublisherNode node
  rclcpp::spin(std::make_shared<PublisherNode>());

  // Shutdown the ROS2 communication
  rclcpp::shutdown();
  return 0;
}
