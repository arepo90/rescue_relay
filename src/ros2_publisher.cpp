/*
    Publisher test program
    Robotec 2025
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <random>
#include <cmath>

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

#define M_PI 3.14159265358979323846

class TestPublisher : public rclcpp::Node{
public:
    TestPublisher() : Node("test_publisher"){
        gas_publisher_ = this->create_publisher<std_msgs::msg::Float32>(GAS_TOPIC, 10);
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(IMU_TOPIC, 10);
        thermal_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(THERMAL_TOPIC, 10);
        track_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>(TRACK_TOPIC, 10);
        encoder_publisher_ = this->create_publisher<std_msgs::msg::Float32>(ENCODER_TOPIC, 10);
        sensor_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(SENSOR_TOPIC, 10);
        joint1_publisher_ = this->create_publisher<std_msgs::msg::Float32>(JOINT1_TOPIC, 10);
        joint2_publisher_ = this->create_publisher<std_msgs::msg::Float32>(JOINT2_TOPIC, 10);
        joint3_publisher_ = this->create_publisher<std_msgs::msg::Float32>(JOINT3_TOPIC, 10);
        joint4_publisher_ = this->create_publisher<std_msgs::msg::Float32>(JOINT4_TOPIC, 10);


        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(250),
            std::bind(&TestPublisher::publish_messages, this)
        );
        std::cout << "setup done\n";
    }

private:
    bool inc = true;
    void publish_messages(){
        static float gas_value = 0;
        static float joint1_value = 0;
        static float joint2_value = 0;
        static float joint3_value = 0;
        static float joint4_value = 0;

        std_msgs::msg::Float32 gas_msg;
        gas_msg.data = gas_value;
        gas_publisher_->publish(gas_msg);
        gas_value += 5;
        if(gas_value > 100) 
            gas_value = 0;

        std_msgs::msg::Float32 joint1_msg;
        joint1_msg.data = joint1_value;
        joint1_publisher_->publish(joint1_msg);
        joint1_value += 15;
        if(joint1_value > 90) 
            joint1_value = 0;
        
        std_msgs::msg::Float32 joint2_msg;
        joint2_msg.data = joint2_value;
        joint2_publisher_->publish(joint2_msg);
        joint2_value += 15;
        if(joint2_value > 90) 
            joint2_value = 0;

        std_msgs::msg::Float32 joint3_msg;
        joint3_msg.data = joint3_value;
        joint3_publisher_->publish(joint3_msg);
        joint3_value += 15;
        if(joint3_value > 90) 
            joint3_value = 0;

        std_msgs::msg::Float32 joint4_msg;
        joint4_msg.data = joint4_value;
        joint4_publisher_->publish(joint4_msg);
        joint4_value += 15;
        if(joint4_value > 90) 
            joint4_value = 0;

        static float imu_angle = -45.0;
        sensor_msgs::msg::Imu imu_msg;
        float roll = imu_angle * M_PI / 180.0;
        float pitch = imu_angle * M_PI / 180.0;
        float yaw = imu_angle * M_PI / 180.0;

        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
        imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
        imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
        imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;

        imu_publisher_->publish(imu_msg);

        //std::cout << "imu euler: " << imu_angle << "\n";
        //std::cout << "imu quat: " << imu_msg.orientation.x << " " << imu_msg.orientation.y << " " << imu_msg.orientation.z << " " << imu_msg.orientation.w << "\n";

        if(inc) imu_angle += 15.0;
        else imu_angle -= 15.0;
        if(imu_angle > 45.0)
            inc = false;
        else if(imu_angle < -45.0)
            inc = true;

        // Thermal topic: 64 length vector with random numbers between 20 to 40
        std_msgs::msg::Float32MultiArray thermal_msg;
        thermal_msg.data.resize(64);
        for(auto &value : thermal_msg.data){
            value = random_float(20.0, 40.0);
        }
        thermal_publisher_->publish(thermal_msg);

        // Track topic: 2 numbers between -1 and 1 increasing by 0.25
        static float track_value = -1.0;
        geometry_msgs::msg::Vector3 track_msg;
        track_msg.x = track_value;
        track_msg.y = track_value;
        track_publisher_->publish(track_msg);
        track_value += 0.25;
        if(track_value > 1.0) track_value = -1.0;

        // Encoder topic: 2 numbers between 0 to 2*pi increasing by pi/4
        static float encoder_value = 0.0;
        std_msgs::msg::Float32 encoder_msg;
        encoder_msg.data =  encoder_value;
        encoder_publisher_->publish(encoder_msg);
        encoder_value += 30.0;
        if(encoder_value > 360.0)
            encoder_value = 0.0;

        // Sensor topic: 3 random numbers between -1 and 1
        std_msgs::msg::Float32MultiArray sensor_msg;
        sensor_msg.data = {random_float(-1.0, 1.0), random_float(-1.0, 1.0), random_float(-1.0, 1.0)};
        sensor_publisher_->publish(sensor_msg);

    }

    float random_float(float min, float max){
        std::uniform_real_distribution<float> dist(min, max);
        return dist(rng_);
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gas_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr joint1_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr joint2_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr joint3_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr joint4_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr thermal_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr track_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr encoder_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr sensor_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::default_random_engine rng_;
};

int main(int argc, char *argv[]){
    std::cout << "hi\n";
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestPublisher>());
    rclcpp::shutdown();
    return 0;
}
