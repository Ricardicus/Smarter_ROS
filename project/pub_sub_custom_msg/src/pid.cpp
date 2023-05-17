#include "rclcpp/rclcpp.hpp"
#include "custom_msg/msg/signal_reference.hpp"
#include "custom_msg/msg/signal_true.hpp"
#include "std_msgs/msg/float64.hpp"
#include "control_toolbox/pid.hpp"

class PIController : public rclcpp::Node
{
public:
    PIController(double p_gain, double i_gain) : Node("pi_controller"), pid_controller(p_gain, i_gain, 0.0, 0.0, 0.0)
    {
        control_pub = this->create_publisher<std_msgs::msg::Float64>("control_signal", 10);
        reference_sub = this->create_subscription<custom_msg::msg::SignalReference>(
            "signal_reference", 10, std::bind(&PIController::referenceCallback, this, std::placeholders::_1));
        true_sub = this->create_subscription<custom_msg::msg::SignalTrue>(
            "signal_true", 10, std::bind(&PIController::trueCallback, this, std::placeholders::_1));
    }

private:
    void referenceCallback(const custom_msg::msg::SignalReference::SharedPtr msg)
    {
        reference = msg->reference;
        computeControl();
    }

    void trueCallback(const custom_msg::msg::SignalTrue::SharedPtr msg)
    {
        true_value = msg->true_value;
        computeControl();
    }

    void computeControl()
    {
        double error = reference - true_value;
        rclcpp::Duration dt = this->get_clock()->now() - last_time;
        last_time = this->get_clock()->now();

        double control_signal = pid_controller.computeCommand(error, dt.nanoseconds());

        auto message = std_msgs::msg::Float64();
        message.data = control_signal;
        control_pub->publish(message);
    }

    rclcpp::Subscription<custom_msg::msg::SignalReference>::SharedPtr reference_sub;
    rclcpp::Subscription<custom_msg::msg::SignalTrue>::SharedPtr true_sub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_pub;
    control_toolbox::Pid pid_controller;
    double reference;
    double true_value;
    rclcpp::Time last_time;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    double p_gain = 1.0;
    double i_gain = 0.5;
    auto node = std::make_shared<PIController>(p_gain, i_gain);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

