/*
    Relay program - 9Linux version
    Robotec 2025

    ** may or may not work/be stable **
*/

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <atomic>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

// --- Initial settings --
#define M_PI 3.14159265358979323846
#define SERVER_IP "127.0.0.1"   //"192.168.0.131"
#define SERVER_PORT 9000
#define MAX_UDP_PACKET_SIZE 65507   // 65507 bytes
#define FRAGMENTATION_FLAG 0x8000   // RTP header flag
int exit_code = 0;

std::vector<std::pair<int, std::string>> cam_info;

struct BasePacket{
    float body_x = 0;
    float body_y = 0;
    float body_z = 0;
    float arms = 0;
    float art_1 = 0;
    float art_2 = 0;
    float art_3 = 0;
    float art_4 = 0;
    float art_5 = 0;
    float art_6 = 0;
    float track_l = 0;
    float track_r = 0;
    float magnetometer_x = 0;
    float magnetometer_y = 0;
    float magnetometer_z = 0;
};

struct RTPHeader{
    uint16_t cc:4;
    uint16_t x:1;
    uint16_t p:1;
    uint16_t version:2;
    uint16_t pt:1;
    uint16_t m;
    uint16_t seq;
    uint16_t timestamp;
    uint16_t ssrc;
};

enum class PayloadType : uint8_t {
    VIDEO_MJPEG = 97,
    AUDIO_PCM = 98,
    ROS2_ARRAY = 99
};

// Linux-compatible RTPStreamHandler class
class RTPStreamHandler{    
public:
    RTPStreamHandler(int port, std::string address, PayloadType type){
        // --- Stream info ---
        stream = new Stream;
        stream->ssrc = 0;
        stream->seq_num = 0 & 0xFFFF;
        stream->timestamp = 0;
        stream->payload_type = type;
        stream->port = port;

        // --- UDP Socket init ---
        // -- send --
        send_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (send_socket < 0) {
            std::cout << "[e] Failed to create send socket. Error: " << errno << std::endl;
            return;
        }
        
        memset(&send_socket_address, 0, sizeof(send_socket_address));
        send_socket_address.sin_family = AF_INET;
        send_socket_address.sin_port = htons(port);
        inet_pton(AF_INET, address.c_str(), &send_socket_address.sin_addr);
        
        // -- recv --  
        recv_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (recv_socket < 0) {
            std::cout << "[e] Failed to create recv socket. Error: " << errno << std::endl;
            return;
        }
        
        int recv_buff_size = 1024 * 1024;   // 1MB
        setsockopt(recv_socket, SOL_SOCKET, SO_RCVBUF, (char*)&recv_buff_size, sizeof(recv_buff_size));
        
        memset(&recv_socket_address, 0, sizeof(recv_socket_address));
        recv_socket_address.sin_family = AF_INET;
        recv_socket_address.sin_port = htons(port + 1);
        recv_socket_address.sin_addr.s_addr = INADDR_ANY;
        
        if (bind(recv_socket, (struct sockaddr*)&recv_socket_address, socket_address_size) < 0) {
            std::cout << "[e] Failed to bind recv socket. Error: " << errno << std::endl;
            return;
        }

        std::cout << "[i] Channel created, bound to ports (" << port << ", " << port + 1 << ")" << std::endl;
    }
    
    ~RTPStreamHandler(){
        std::cout << "[i] Closing channel (" << stream->port << ", " << stream->port + 1 << ")" << std::endl;
        shutdown(recv_socket, SHUT_RDWR);
        close(send_socket);
        close(recv_socket);
    }

    void destroy(){
        delete this;
    }
    
    template <typename T> void sendPacket(std::vector<T> data, int marker = 0){
        // --- Initial settings ---
        int max_size = MAX_UDP_PACKET_SIZE - sizeof(RTPHeader);
        int num_fragments = ((data.size()*sizeof(T)) + max_size - 1) / max_size;
        // -- (Pseudo)random ssrc --
        thread_local uint16_t ssrc = 1;
        ssrc ^= ssrc << 7;
        ssrc ^= ssrc >> 9;
        ssrc ^= ssrc << 8;

        // --- Fragment setup ---
        for(int i = 0; i < num_fragments; i++){
            // -- RTP header info --
            RTPHeader header;
            header.version = 2;
            header.p = 0;
            header.x = 0;
            header.cc = 0;
            header.m = (uint16_t)num_fragments;
            header.pt = 0;
            header.timestamp = 0;
            header.ssrc = ssrc;
            header.seq = (uint16_t)i;
            if(num_fragments > 1)
                header.seq |= FRAGMENTATION_FLAG;
            // -- Merge header + packet --
            int current_size = (max_size < ((data.size()*sizeof(T)) - (i*max_size)) ? max_size : (data.size()*sizeof(T)) - (i*max_size));
            std::vector<char> packet(current_size + sizeof(RTPHeader));
            std::memcpy(packet.data(), &header, sizeof(RTPHeader));
            std::memcpy(packet.data() + sizeof(RTPHeader), data.data() + (i*max_size), current_size);

            // --- Simulated network degradation (lowkey trash implementation, sorry) ---
            /*
            this_thread::sleep_for(std::chrono::milliseconds(rand() % 150));    // ~100ms latency
            if(rand() % 10 == 0) return;    // ~10% packet loss
            */

            if(sendto(send_socket, (const char*)packet.data(), packet.size(), 0, (struct sockaddr*)&send_socket_address, socket_address_size) < 0){
                int error = errno;
                if(error == 9) return;
                std::cout << "[w] Packet send failed on fragment " << i << ". Error: " << error << std::endl;
            }
        } 
    }
    
    std::vector<int> recvPacket(){
        // --- Receive (non-fragmented) packet ---
        std::vector<char> packet(4096);
        int bytes_received = recvfrom(recv_socket, packet.data(), packet.size(), 0, (struct sockaddr*)&recv_socket_address, &socket_address_size);
        if(bytes_received < 0){
            int error = errno;
            if(error != EAGAIN && error != EWOULDBLOCK) 
                std::cout << "[e] Packet recv failed. Error: " << error << std::endl;
            return {}; 
        }
        else if(bytes_received <= (int)sizeof(RTPHeader)){
            if(bytes_received != 0)
                std::cout << "[w] Empty packet received, size: " << bytes_received << std::endl;
            return {};
        }

        // --- Parse header and data ---
        RTPHeader* header = new RTPHeader;
        std::memcpy(header, packet.data(), sizeof(RTPHeader));
        std::vector<int> data((bytes_received - sizeof(RTPHeader)) / sizeof(int));
        std::memcpy(data.data(), packet.data() + sizeof(RTPHeader), bytes_received - sizeof(RTPHeader));
        return data;
    }
    
private:
    struct Stream{
        uint32_t ssrc;
        uint16_t seq_num;
        uint32_t timestamp;
        int port;
        PayloadType payload_type;
    };
    Stream* stream;
    int send_socket;
    int recv_socket;
    sockaddr_in send_socket_address;
    sockaddr_in recv_socket_address;
    socklen_t socket_address_size = sizeof(send_socket_address);
};

class RelayNode : public rclcpp::Node{
public:
    RelayNode() : Node("local_relay_node"){
        // --- Full startup ---

        image_sub1 = this->create_subscription<sensor_msgs::msg::CompressedImage>("/camera0/image_raw/compressed", 10, [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg){
            cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
            std::lock_guard<std::mutex> lock(cam1_mutex);
            frame1 = frame;
        });
        image_sub2 = this->create_subscription<sensor_msgs::msg::CompressedImage>("/camera1/image_raw/compressed", 10, [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg){
            cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
            std::lock_guard<std::mutex> lock(cam2_mutex);
            frame2 = frame;
        });
        std::cout << "[i] Starting stream handlers..." << std::endl;

        // -- video --
        for(int i = 0; i < 2; i++){
            SocketStruct socket_struct;
            socket_struct.target_socket = new RTPStreamHandler(SERVER_PORT + (2*i), SERVER_IP, PayloadType::VIDEO_MJPEG);
            video_sockets.push_back(std::move(socket_struct));
        }
        for(int i = 0; i < video_sockets.size(); i++){
            video_sockets[i].is_recv_running.store(true);
            video_sockets[i].is_send_running.store(true);
            video_sockets[i].is_active.store(false);        
        }

        // --- Threads startup - Program begins ---
        std::cout << "[i] Initializing threads..." << std::endl;

        // -- video --
        for(int i = 0; i < video_sockets.size(); i++){
            video_sockets[i].send_thread = std::thread([i, this](){
                while(video_sockets[i].is_send_running.load()){
                    if(video_sockets[i].is_active.load()){
                        cv::Mat frame;
                        std::vector<unsigned char> compressed_data;
                        if(i == 0){
                            std::lock_guard<std::mutex> lock(cam1_mutex);
                            frame = frame1;
                        }
                        else{
                            std::lock_guard<std::mutex> lock(cam2_mutex);
                            frame = frame2;
                        }
                        if(frame.empty()){
                            std::this_thread::sleep_for(std::chrono::milliseconds(500));
                            continue;
                        }
                        cv::imencode(".jpg", frame, compressed_data, {cv::IMWRITE_JPEG_QUALITY, 100});
                        video_sockets[i].target_socket->sendPacket(compressed_data);
                    }
                    else 
                        std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
            });
            video_sockets[i].recv_thread = std::thread([i, this](){
                while(video_sockets[i].is_recv_running.load()){
                    std::vector<int> data = video_sockets[i].target_socket->recvPacket();
                    if(data.size() <= 1) continue;
                    video_sockets[i].is_active.store(data[1]);
                }
            });
        }
        
        std::cout << "[i] Setup done" << std::endl;
    }
    ~RelayNode(){
        // --- Stop & join threads + destroy objects ---
        std::cout << "[i] Closing program..." << std::endl;
        
        for(int i = 0; i < video_sockets.size(); i++){
            video_sockets[i].is_active.store(false);
            video_sockets[i].is_recv_running.store(false);
            video_sockets[i].is_send_running.store(false);
        }

        // -- video --
        for(int i = 0; i < video_sockets.size(); i++){
            video_sockets[i].target_socket->destroy();
            if(video_sockets[i].send_thread.joinable()) 
                video_sockets[i].send_thread.join();
            if(video_sockets[i].recv_thread.joinable()) 
                video_sockets[i].recv_thread.join();
        }
        std::cout << "[i] Video channels closed\n[i] Bye" << std::endl;
    }
private:
    struct SocketStruct{
        RTPStreamHandler* target_socket;
        std::thread send_thread;
        std::thread recv_thread;
        std::atomic<bool> is_send_running;
        std::atomic<bool> is_recv_running;
        std::atomic<bool> is_active;
        // --- Unecessarily complicated implementation to transfer std::thread ownership ---
        SocketStruct() : target_socket(nullptr){}
        SocketStruct(SocketStruct&& other) noexcept
            : recv_thread(std::move(other.recv_thread)),
              send_thread(std::move(other.send_thread)),
              target_socket(std::move(other.target_socket)){}
        SocketStruct& operator=(SocketStruct&& other) noexcept {
            if(this != &other){
                recv_thread = std::move(other.recv_thread);
                send_thread = std::move(other.send_thread);
                target_socket = std::move(other.target_socket);
            }
            return *this;
        }
        SocketStruct(const SocketStruct&) = delete;
        SocketStruct& operator=(const SocketStruct&) = delete;
    };
    void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg, int id){
        try{
            cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
            if(id == 0){
                std::lock_guard<std::mutex> lock(cam1_mutex);
                frame1 = frame;
            }
            else{
                std::lock_guard<std::mutex> lock(cam2_mutex);
                frame2 = frame;
            }
        } 
        catch (cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub1;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub2;


    std::mutex cam1_mutex;
    std::mutex cam2_mutex;
    cv::Mat frame1;
    cv::Mat frame2;
    std::vector<SocketStruct> video_sockets;
};

int main(int argc, char** argv){
    std::cout << "[i] Hi Linux" << std::endl;
    // --- RelayNode handles everything ---
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RelayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return exit_code;
}
