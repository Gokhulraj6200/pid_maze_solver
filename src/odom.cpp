#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

class OdomListener : public rclcpp::Node {
public:
  OdomListener() : Node("odom_listener") {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/rosbot_xl_base_controller/odom", 10,
        std::bind(&OdomListener::odom_callback, this, std::placeholders::_1));
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    curr_x = msg->pose.pose.position.x;
    curr_y = msg->pose.pose.position.y;
    curr_z = msg->pose.pose.position.z;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    RCLCPP_INFO(get_logger(), "x: %f, y: %f, z: %f", msg->pose.pose.position.x,
                msg->pose.pose.position.y, msg->pose.pose.position.z);
    // RCLCPP_INFO(get_logger(), "x: %f, y: %f, z: %f, w: %f",
    // msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
    // msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    RCLCPP_INFO(get_logger(), "R: %f, P:%f, ANGLE: %f", roll, pitch, yaw);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  double curr_x = 0;
  double curr_y = 0;
  double curr_z = 0;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomListener>());
  return 0;
}