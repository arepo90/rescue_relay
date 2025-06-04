#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class OdometryNode : public rclcpp::Node
{
public:
    OdometryNode() : Node("odometry_node"), x_(0.0), y_(0.0), theta_(0.0), last_time_(this->now())
    {        
        bad_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/bad_odom", 10, std::bind(&OdometryNode::bad_odom_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&OdometryNode::imu_callback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        timer_ = this->create_wall_timer(20ms, std::bind(&OdometryNode::update, this));
    }

private:
    void bad_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        left_speed_ = msg->twist.twist.linear.x - (msg->twist.twist.angular.z * wheel_base_ / 2.0);
        right_speed_ = msg->twist.twist.linear.x + (msg->twist.twist.angular.z * wheel_base_ / 2.0);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        theta_ = yaw;
    }

    void update()
    {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        double linear_velocity = (left_speed_ + right_speed_) / 2.0;

        x_ += linear_velocity * std::cos(theta_) * dt;
        y_ += linear_velocity * std::sin(theta_) * dt;

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom.pose.pose.orientation = tf2::toMsg(q);

        odom.twist.twist.linear.x = linear_velocity;
        odom.twist.twist.angular.z = (right_speed_ - left_speed_) / wheel_base_;

        odom_pub_->publish(odom);

        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = current_time;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_link";

        odom_tf.transform.translation.x = x_;
        odom_tf.transform.translation.y = y_;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(odom_tf);
    }

    double x_, y_, theta_;
    double left_speed_ = 0.0, right_speed_ = 0.0;
    const double wheel_base_ = 0.5;

    rclcpp::Time last_time_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr bad_odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    std::cout << "starting node...";
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryNode>());
    rclcpp::shutdown();
    return 0;
}
