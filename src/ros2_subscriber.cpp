/*
    Subscriber test program
    Robotec 2025

    Couldn't be bothered to write any of this, so thank deepseek
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include <iostream>
#include <vector>
#include <sstream>
#include <iomanip>

#define IMU_TOPIC "imu_data"
#define ENCODER_TOPIC "encoder_position"
#define GAS_TOPIC "mq2_gas"
#define TRACK_TOPIC "track_velocity"
#define THERMAL_TOPIC "thermal_image"
#define SENSOR_TOPIC "sensor_topic"
#define JOINT1_TOPIC "joint_base"
#define JOINT2_TOPIC "joint_shouler"
#define JOINT3_TOPIC "joint_elbow"
#define JOINT4_TOPIC "joint_hand"

// Use placeholders for std::bind (like _1)
using namespace std::placeholders;

class TestSubscriber : public rclcpp::Node {
public:
    TestSubscriber() : Node("test_subscriber") {
        RCLCPP_INFO(this->get_logger(), "Test Subscriber Node Started. Listening...");

        // Create subscriptions for each topic
        gas_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            GAS_TOPIC, 10, std::bind(&TestSubscriber::gas_callback, this, _1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            IMU_TOPIC, 10, std::bind(&TestSubscriber::imu_callback, this, _1));

        thermal_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            THERMAL_TOPIC, 10, std::bind(&TestSubscriber::thermal_callback, this, _1));

        track_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            TRACK_TOPIC, 10, std::bind(&TestSubscriber::track_callback, this, _1));

        encoder_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            ENCODER_TOPIC, 10, std::bind(&TestSubscriber::encoder_callback, this, _1));

        sensor_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            SENSOR_TOPIC, 10, std::bind(&TestSubscriber::sensor_callback, this, _1));

        joint1_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            JOINT1_TOPIC, 10, std::bind(&TestSubscriber::joint1_callback, this, _1));

        joint2_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            JOINT2_TOPIC, 10, std::bind(&TestSubscriber::joint2_callback, this, _1));

        joint3_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            JOINT3_TOPIC, 10, std::bind(&TestSubscriber::joint3_callback, this, _1));

        joint4_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            JOINT4_TOPIC, 10, std::bind(&TestSubscriber::joint4_callback, this, _1));
    }

private:
    // --- Callback functions ---
    void gas_callback(const std_msgs::msg::Float32::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "Received [%s]: %.2f", GAS_TOPIC, msg->data);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "Received [%s]: Orientation(x=%.3f, y=%.3f, z=%.3f, w=%.3f)",
                    IMU_TOPIC,
                    msg->orientation.x,
                    msg->orientation.y,
                    msg->orientation.z,
                    msg->orientation.w);
        // Optionally print other IMU fields if needed (they are 0 in the publisher)
        // RCLCPP_INFO(this->get_logger(), "  AngVel(x=%.3f, y=%.3f, z=%.3f), LinAcc(x=%.3f, y=%.3f, z=%.3f)",
        //             msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
        //             msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    }

    void thermal_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2); // Format floats nicely
        ss << "[";
        bool first = true;
        for (const auto& val : msg->data) {
            if (!first) {
                ss << ", ";
            }
            ss << val;
            first = false;
        }
        ss << "]";
        RCLCPP_INFO(this->get_logger(), "Received [%s]: %zu elements %s",
                    THERMAL_TOPIC, msg->data.size(), ss.str().c_str());
    }

    void track_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "Received [%s]: x=%.2f, y=%.2f, z=%.2f",
                    TRACK_TOPIC, msg->x, msg->y, msg->z);
    }

    void encoder_callback(const std_msgs::msg::Float32::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "Received [%s]: %.2f degrees", ENCODER_TOPIC, msg->data);
    }

    void sensor_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const {
         std::stringstream ss;
        ss << std::fixed << std::setprecision(3); // Format floats nicely
        ss << "[";
        bool first = true;
        for (const auto& val : msg->data) {
            if (!first) {
                ss << ", ";
            }
            ss << val;
            first = false;
        }
        ss << "]";
        RCLCPP_INFO(this->get_logger(), "Received [%s]: %zu elements %s",
                    SENSOR_TOPIC, msg->data.size(), ss.str().c_str());
    }

    void joint1_callback(const std_msgs::msg::Float32::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "Received [%s]: %.2f degrees", JOINT1_TOPIC, msg->data);
    }

    void joint2_callback(const std_msgs::msg::Float32::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "Received [%s]: %.2f degrees", JOINT2_TOPIC, msg->data);
    }

    void joint3_callback(const std_msgs::msg::Float32::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "Received [%s]: %.2f degrees", JOINT3_TOPIC, msg->data);
    }

    void joint4_callback(const std_msgs::msg::Float32::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "Received [%s]: %.2f degrees", JOINT4_TOPIC, msg->data);
    }


    // --- Subscription Member Variables ---
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gas_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr thermal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr track_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr encoder_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sensor_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr joint1_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr joint2_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr joint3_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr joint4_sub_;
};

int main(int argc, char* argv[]) {
    std::cout << "Starting Test Subscriber Node...\n";
    rclcpp::init(argc, argv);
    // Create the node and spin it to keep it alive and processing callbacks
    rclcpp::spin(std::make_shared<TestSubscriber>());
    rclcpp::shutdown();
    std::cout << "Test Subscriber Node Shutdown.\n";
    return 0;
}