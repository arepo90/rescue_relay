#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#define TOPIC_NAME "rescue_topic"

class SubscriberNode : public rclcpp::Node{
public:
    SubscriberNode() : Node("subscriber"){
        std::cout << "[i] Starting subscriber...\n";
        subscription = this->create_subscription<std_msgs::msg::Float32MultiArray>(TOPIC_NAME, 10, [this](std_msgs::msg::Float32MultiArray::UniquePtr msg){
            std::cout << "msg size: " << msg->data.size() << " first val: " << msg->data[0] << "\n";
        });
        std::cout << "[i] Setup done\n";
    }
    ~SubscriberNode(){
        std::cout << "[i] Bye\n";
    }
private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription;
};

int main(int argc, char * argv[]){
    std::cout << "[i] Hi\n";
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberNode>());
    rclcpp::shutdown();
    return 0;
}