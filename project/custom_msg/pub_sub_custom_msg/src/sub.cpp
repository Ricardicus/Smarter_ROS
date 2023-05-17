// sub.cpp
// C++ Header files
#include <memory>

// Import the rclcpp client library and std_msgs header
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Include our custom message
#include "custom_msg/msg/log_tf.hpp" //change

// creation of function objects with placeholder arguments
using std::placeholders::_1;

// SubscriberNode Composition
//We create a SubscriberNode class that inherits from the rclcpp::Node
class SubscriberNode : public rclcpp::Node
{
  public:
    SubscriberNode()
    : Node("subscriber")
    {
      // Create a subscriber with String message type and queue size is 10
      subscription_ = this->create_subscription<custom_msg::msg::LogTf>(
      "string_msg", 10, std::bind(&SubscriberNode::callback, this, _1)); //change
    }

  private:
    // The moment the message available in the queue and 
    // the call back execute the print statement
    void callback(const custom_msg::msg::LogTf::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->time_frame.c_str()); //change
      RCLCPP_INFO(this->get_logger(), "%s", msg->log_msg.c_str()); //change
    }

    // Subscriber
    rclcpp::Subscription<custom_msg::msg::LogTf>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  // Initialize the ROS2 communication
  rclcpp::init(argc, argv);

  // Create a default single-threaded executor and spin the SubscriberNode node
  rclcpp::spin(std::make_shared<SubscriberNode>());

  // Shutdown the ROS2 communication
  rclcpp::shutdown();
  
  return 0;
}
