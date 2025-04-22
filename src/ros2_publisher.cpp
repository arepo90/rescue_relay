/*
    Publisher test program
    Robotec 2025
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#define TOPIC_NAME "rescue_topic"

struct BasePacket{
    float body_x = 0;
    float body_y = 0;
    float body_z = 0;
    float arm_l = 0;
    float arm_r = 0;
    float art_1 = 0;
    float art_2 = 0;
    float art_3 = 0;
    float art_4 = 0;
    float track_l = 0;
    float track_r = 0;
    float armtrack_l = 0;
    float armtrack_r = 0;
};

class PublisherNode : public rclcpp::Node{
public:
    PublisherNode() : Node("publisher"){
        std::cout << "[i] Starting publisher...\n";
        float_publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>(TOPIC_NAME, 10);
        timer = this->create_wall_timer(std::chrono::milliseconds(1000), [this](){
            std_msgs::msg::Float32MultiArray message = std_msgs::msg::Float32MultiArray();
            message.data = { packet.body_x++, packet.body_y, packet.body_z, packet.arm_l, packet.arm_r, packet.art_1, packet.art_2, packet.art_3, packet.art_4, packet.track_l, packet.track_r, packet.armtrack_l, packet.armtrack_r };
            std::cout <<  "[i] Publishing... " << packet.body_x << "\n";
            float_publisher->publish(message);
        });
        std::cout << "[i] Setup done\n";
    }
    ~PublisherNode(){
        std::cout << "[i] Bye\n";
    }
private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr float_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr img_publisher;
    rclcpp::TimerBase::SharedPtr timer;
    BasePacket packet;
};

int main(int argc, char *argv[]){
    std::cout << "[i] Hi\n";
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherNode>());
    rclcpp::shutdown();
    return 0;
}