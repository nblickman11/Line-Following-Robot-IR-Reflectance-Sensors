#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <gpiod.hpp>
#include <chrono>

class SensorDriverNode : public rclcpp::Node
{
public:
    SensorDriverNode()
    : Node("sensor_driver_node")
    {
        // GPIO setup
        chip_ = gpiod::chip("/dev/gpiochip0");  // Change if your GPIO chip differs
        left_line_ = chip_.get_line(4);        // left sensor
        center_line_ = chip_.get_line(3);        // left sensor
    	right_line_ = chip_.get_line(2);       // right sensor

        left_line_.request({"line_follower", gpiod::line_request::DIRECTION_INPUT});
        center_line_.request({"line_follower", gpiod::line_request::DIRECTION_INPUT});
	right_line_.request({"line_follower", gpiod::line_request::DIRECTION_INPUT});

        // Publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SensorDriverNode::loop, this)
        );

        RCLCPP_INFO(this->get_logger(), "Sensor Driver Node started using libgpiod");
    }

private:
    void loop()
    {
        int left = left_line_.get_value();
	int center = center_line_.get_value();
        int right = right_line_.get_value();

        geometry_msgs::msg::Twist cmd;

        RCLCPP_INFO(this->get_logger(), "Left: %d | Center: %d | Right: %d", left, center, right);

	// 1 means we are on the black line.
        // Line following logic
	if (center == 1)
        {
            cmd.linear.x = 0.2;
            cmd.angular.z = 0.0;
        }
        else if (left == 0)
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = -0.4;  // turn right
        }
        else if (right == 0)
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.4;   // turn left
        }
        else
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;   // stop
        }

        cmd_vel_pub_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    gpiod::chip chip_;
    gpiod::line left_line_;
    gpiod::line center_line_;
    gpiod::line right_line_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorDriverNode>());
    rclcpp::shutdown();
    return 0;
}

