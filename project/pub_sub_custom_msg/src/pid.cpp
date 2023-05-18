#include "rclcpp/rclcpp.hpp"
#include "custom_msg/msg/controller_output.hpp"
#include "custom_msg/msg/controller_input.hpp"
#include "control_toolbox/pid.hpp"

class PIController : public rclcpp::Node
{
public:
    PIController(double p_gain, double i_gain) : Node("pi_controller"), pid_controller(p_gain, i_gain, 0.0, 0.0, 0.0)
    {
        control_pub = this->create_publisher<custom_msg::msg::ControllerOutput>("controller_output", 10);
        control_sub = this->create_subscription<custom_msg::msg::ControllerInput>(
            "controller_input", 10, std::bind(&PIController::inputCallback, this, std::placeholders::_1));
        last_time = this->get_clock()->now();
    }

private:
    void inputCallback(const custom_msg::msg::ControllerInput::SharedPtr msg)
    {
        reference = msg->reference;
        true_value = msg->true_value;
        computeControl();
    }

    void computeControl()
    {
        double error = true_value - reference;
        rclcpp::Duration dt = this->get_clock()->now() - last_time;
        last_time = this->get_clock()->now();

        double control_value = pid_controller.computeCommand(error, dt.nanoseconds());

        auto message = custom_msg::msg::ControllerOutput();
        message.control_value = control_value;
        control_pub->publish(message);
    }

    rclcpp::Subscription<custom_msg::msg::ControllerInput>::SharedPtr control_sub;
    rclcpp::Publisher<custom_msg::msg::ControllerOutput>::SharedPtr control_pub;
    control_toolbox::Pid pid_controller;
    double reference;
    double true_value;
    rclcpp::Time last_time;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    double p_gain = 50.0;
    double i_gain = 100.0;
    auto node = std::make_shared<PIController>(p_gain, i_gain);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

