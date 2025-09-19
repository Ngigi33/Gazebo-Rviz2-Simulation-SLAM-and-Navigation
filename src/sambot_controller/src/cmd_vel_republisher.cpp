#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class CmdVelConverter : public rclcpp::Node {
public:
    CmdVelConverter() : Node("cmd_vel_converter") {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&CmdVelConverter::cmd_callback, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/sambot_diffdrive_controller/cmd_vel", 10
        );
    }

private:
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        geometry_msgs::msg::TwistStamped stamped_msg;
        stamped_msg.header.stamp = this->now();
        stamped_msg.header.frame_id = "base_link";  // or any appropriate frame
        stamped_msg.twist = *msg;

        publisher_->publish(stamped_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdVelConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
