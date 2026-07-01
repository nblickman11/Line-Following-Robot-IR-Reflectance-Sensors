#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <gpiod.hpp>
#include <chrono>
#include <pigpio.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

class MotorNode : public rclcpp::Node
{
public:
    MotorNode() : Node("motor_node")
    {
        // Initialize GPIO chip (gpiochip0 is standard for Raspberry Pi)
        chip = gpiod::chip("gpiochip0");

        // Set BCM GPIO pin numbers used to control motor driver inputs
	in1_pin = 18; // Motor A forward
        in2_pin = 23; // Motor A back
        in3_pin = 17; // Motor B back
        in4_pin = 27; // Motor B forward

        // Get handles to GPIO lines
        in1 = chip.get_line(in1_pin);
        in2 = chip.get_line(in2_pin);
        in3 = chip.get_line(in3_pin);
        in4 = chip.get_line(in4_pin);

	// Request each line as output with initial value LOW (0)
        // The request() function takes:
        //   - a consumer name ("motor_ctrl" helps identify who owns the pin)
        //   - a flag that this is a direction OUTPUT
        //   - initial value (0 = LOW)
        // If this fails, it throws an exception (e.g., if already in use)
        in1.request({"motor_ctrl", gpiod::line_request::DIRECTION_OUTPUT, 0});
        in2.request({"motor_ctrl", gpiod::line_request::DIRECTION_OUTPUT, 0});
        in3.request({"motor_ctrl", gpiod::line_request::DIRECTION_OUTPUT, 0});
        in4.request({"motor_ctrl", gpiod::line_request::DIRECTION_OUTPUT, 0});

        left_pwm_pin = 13;  // Motor PMWA Enable/Speed
        right_pwm_pin = 12; // Motor MPWB Enable/Speed

	// Initialize pigpio
	if (gpioInitialise() < 0) {
    		RCLCPP_ERROR(this->get_logger(), "Failed to initialize pigpio!");
    		rclcpp::shutdown();
	}
        
	// Subscribe to the /cmd_vel topic (standard ROS topic for velocity commands)
        // Queue size of 1; uses std::bind to attach the callback method
        // Prevents overcorrection! Only need to run latest command.
	subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&MotorNode::cmdVelCallback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Motor controller node started. Listening to /cmd_vel...");
    }
    ~MotorNode()
    {
        // Stop motors on shutdown
        in1.set_value(0);
        in2.set_value(0);
        in3.set_value(0);
        in4.set_value(0);
    }

private:
    gpiod::chip chip;
    gpiod::line in1, in2, in3, in4, in5, in6;
    int in1_pin, in2_pin, in3_pin, in4_pin, left_pwm_pin, right_pwm_pin;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

    // This callback is called every time a new message is received on /cmd_vel
    // Twist messages contain:
    //   - linear.x for forward/backward velocity
    //   - angular.z for rotation (turning left/right)
    void setMotorPins(bool a1, bool a2, bool b1, bool b2)
    {
        in1.set_value(a1);
        in2.set_value(a2);
        in3.set_value(b1);
        in4.set_value(b2);
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double x = msg->linear.x;
        double z = msg->angular.z;
	
	// x=1, z=0. so x-z = 1. then 1 * 255 * scale.5 = 128 so half speed.
	// // Combine forward/backward and turning
	//left_pwm  = (x - z) * base_speed * speed_factor;
	//right_pwm = (x + z) * base_speed * speed_factor;

	// Set speed with duty cycle (0–255)
	gpioPWM(left_pwm_pin, 120);   // 
	gpioPWM(right_pwm_pin,120);  //

	// Simple directional logic
        if (x > 0.1) {
            // Forward
            setMotorPins(1, 0, 0, 1);
            RCLCPP_INFO(this->get_logger(), "Moving forward");
        }
        else if (x < -0.1) {
            // Backward
            setMotorPins(0, 1, 1, 0);
            RCLCPP_INFO(this->get_logger(), "Moving backward");
        }
        else if (z > 0.1) {
            // Turn left
	    setMotorPins(0, 1, 0, 1);
            RCLCPP_INFO(this->get_logger(), "Turning left");
	    //setMotorPins(0, 0, 0, 0);
        }
        else if (z < -0.1) {
		// Turn right
            setMotorPins(1, 0, 1, 0);
            RCLCPP_INFO(this->get_logger(), "Turning right");
	    // Then Stop!
	    //setMotorPins(0, 0, 0, 0);
        }
        else {
            // Stop
            setMotorPins(0, 0, 0, 0);
            RCLCPP_INFO(this->get_logger(), "Stopped");
        }
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorNode>();
    rclcpp::spin(node);  // spin will stop on Ctrl+C
    rclcpp::shutdown();
    return 0;
}


