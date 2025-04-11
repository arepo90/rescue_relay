/*
    Relay program - Linux version
    Robotec 2025
*/

#ifndef UNIX
    #error "Unsupported OS: Windows required"
#endif

#include <portaudio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>
#include <cstdint>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <opus/opus.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#define TOPIC_NAME "rescue_topic"
#define SERVER_IP "127.0.0.1"
#define SERVER_PORT 8000
#define AUDIO_SAMPLE_RATE 48000     // 48 kHz
#define AUDIO_FRAME_SIZE 2880       // 2880 bytes = 120 ms
#define VIDEO_WIDTH 1280
#define VIDEO_HEIGHT 720
#define VIDEO_FPS 30

std::vector<int> cam_ports = {0};

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

struct RTPHeader{
    uint16_t cc:4;
    uint16_t x:1;
    uint16_t p:1;
    uint16_t version:2;
    uint16_t pt:7;
    uint16_t m:1;
    uint16_t seq;
    uint32_t timestamp;
    uint32_t ssrc;
};

enum class PayloadType : uint8_t {
    VIDEO_MJPEG = 97,
    AUDIO_PCM = 98,
    ROS2_ARRAY = 99
};

std::vector<int> scanWebcams(int num_ports = 5){
    std::vector<int> ports;
    for(int i = 0; i < num_ports; i++){
        cv::VideoCapture cap(i);
        if(cap.isOpened()){
            ports.push_back(i);
            cap.release();
        }
    }
    return ports;
}

class RTPStreamHandler{    
public:
    RTPStreamHandler(int port, std::string address, PayloadType type){
        // ---- Stream info ---
        stream = new Stream;
        stream->ssrc = 0;
        stream->seq_num = 0 & 0xFFFF;
        stream->timestamp = 0;
        stream->payload_type = type;
        stream->port = port;
        
        // --- UDP Socket init ---
        // --- send ---
        send_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        send_socket_address.sin_family = AF_INET;
        send_socket_address.sin_port = htons(port);
        inet_pton(AF_INET, address.c_str(), &send_socket_address.sin_addr);
        // -- recv --
        recv_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        int recv_buff_size = 1024 * 1024;   // 1MB
        setsockopt(recv_socket, SOL_SOCKET, SO_RCVBUF, (char*)&recv_buff_size, sizeof(recv_buff_size));
        recv_socket_address.sin_family = AF_INET;
        recv_socket_address.sin_port = htons(port + 1);
        recv_socket_address.sin_addr.s_addr = INADDR_ANY;
        ::bind(recv_socket, (struct sockaddr*)&recv_socket_address, sizeof(recv_socket_address));
        
        std::cout << "[i] Channel created, bound to ports (" << port << ", " << port + 1 << ")\n";
    }
    ~RTPStreamHandler(){
        std::cout << "[i] Closing channel (" << stream->port << ", " << stream->port + 1 << ")\n";
        shutdown(recv_socket, SHUT_RDWR);
        close(send_socket);
        close(recv_socket);
    }

    void destroy(){
        delete this;
    }

    template <typename T> void sendPacket(std::vector<T> data){
        // --- RTP header info ---
        RTPHeader header;
        header.version = 2;
        header.p = 0;
        header.x = 0;
        header.cc = 0;
        header.m = 1;
        header.pt = static_cast<uint8_t>(stream->payload_type);
        header.seq = stream->seq_num++;
        header.timestamp = stream->timestamp;
        header.ssrc = stream->ssrc;
        stream->timestamp += 100;

        // --- Prepare packet for send ---
        std::vector<char> packet((data.size() * sizeof(T)) + sizeof(RTPHeader));
        std::memcpy(packet.data(), &header, sizeof(RTPHeader));
        std::memcpy(packet.data() + sizeof(RTPHeader), data.data(), data.size() * sizeof(T));

        // --- Simulated network degradation (lowkey trash implementation but idc) ---
        /*
        this_thread::sleep_for(std::chrono::milliseconds(rand() % 150));    // ~100ms latency
        if(rand() % 10 == 0) return;    // ~10% packet loss
        */

        if(sendto(send_socket, packet.data(), packet.size(), 0, (struct sockaddr*)&send_socket_address, sizeof(send_socket_address)) == -1)
            std::cout << "[w] Packet send failed\n";
    }

    std::vector<int> recvPacket(){
        // --- Receive packet ---
        std::vector<char> packet(4096);
        int bytes_received = recvfrom(recv_socket, packet.data(), packet.size(), 0, (struct sockaddr*)&recv_socket_address, (socklen_t*)&socket_address_size);
        if(bytes_received == -1){
            std::cout << "[e] Packet recv failed\n";
            return {};
        }
        else if(bytes_received <= (int)sizeof(RTPHeader)){
            std::cout << "[w] Empty packet received\n";
            return {};
        }

        // --- Parse header and relevant data ---
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
    int socket_address_size = sizeof(send_socket_address);
};

class RelayNode : public rclcpp::Node{
public:
    RelayNode() : Node("relay_node"){
        // --- Sockets + ROS2 startup ---
        std::cout << "[i] Starting stream handlers...\n";
        // -- base + audio --
        base_socket.target_socket = new RTPStreamHandler(SERVER_PORT, SERVER_IP, PayloadType::ROS2_ARRAY);
        base_socket.is_recv_running.store(true);
        base_socket.is_send_running.store(true);
        audio_socket.is_recv_running.store(true);
        audio_socket.is_send_running.store(true);
        audio_socket.target_socket = new RTPStreamHandler(SERVER_PORT + 2, SERVER_IP, PayloadType::AUDIO_PCM);
        // -- video --
        for(int i = 0; i < cam_ports.size(); i++){
            SocketStruct socket_struct;
            socket_struct.target_socket = new RTPStreamHandler(SERVER_PORT + (2*i) + 4, SERVER_IP, PayloadType::VIDEO_MJPEG);
            video_sockets.push_back(std::move(socket_struct));
        }
        for(int i = 0; i < video_sockets.size(); i++){
            video_sockets[i].is_send_running.store(true);
            video_sockets[i].is_recv_running.store(true);
        }
        
        // --- Opus + PortAudio startup ---
        int opus_error;
        opus_encoder = opus_encoder_create(AUDIO_SAMPLE_RATE, 1, OPUS_APPLICATION_AUDIO, &opus_error);
        Pa_Initialize();
        Pa_OpenDefaultStream(&stream, 1, 0, paInt16, AUDIO_SAMPLE_RATE, AUDIO_FRAME_SIZE, audioCallback, this);
        Pa_StartStream(stream);

        // --- Threads startup - Program begins ---
        std::cout << "[i] Initializing threads...\n";
        // -- base --
        base_socket.send_thread = std::thread([this](){
            while(base_socket.is_send_running.load()){
                std::vector<float> data(sizeof(BasePacket) / sizeof(float));
                {
                    std::lock_guard<std::mutex> lock(ros2_mutex);
                    std::memcpy(data.data(), &base_packet, sizeof(BasePacket));
                }
                base_socket.target_socket->sendPacket(data);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Immediate updates are not necessary
            }
        });
        base_socket.recv_thread = std::thread([this](){
            while(base_socket.is_recv_running.load()){
                std::vector<int> data = base_socket.target_socket->recvPacket();
                if(data.size() == 0 || data[0] != 0) continue;
                else if(data[1] == -1){
                    std::cout << "[i] Shutdown received from GUI";
                    this->destroy();
                    break;
                }
                // --- Mutex locks for thread-safe updates ---
                std::lock_guard<std::mutex> lock(gui_mutex);
                gui_data = data;
            }
        });
        // -- audio --
        audio_socket.recv_thread = std::thread([this](){
            while(audio_socket.is_recv_running.load()){
                std::vector<int> data = audio_socket.target_socket->recvPacket();
                if(data.size() == 0 || data[0] != 0) continue;
                audio_data = data;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
        audio_socket.send_thread = std::thread([this](){
            while(audio_socket.is_send_running.load()){
                std::vector<int> data(audio_data.size());
                std::lock_guard<std::mutex> lock(audio_mutex);
                std::memcpy(data.data(), audio_data.data(), audio_data.size());
                audio_socket.target_socket->sendPacket(data);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        });
        // -- ros2 --
        ros2_subscription = this->create_subscription<std_msgs::msg::Float32MultiArray>(TOPIC_NAME, 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg){
            ros2_data.clear();
            ros2_data = msg->data;
            // --- Mutex locks for thread-safe updates ---
            std::lock_guard<std::mutex> lock(ros2_mutex);
            std::memcpy(&base_packet, ros2_data.data(), sizeof(BasePacket));
            std::cout << "received first: " << base_packet.body_x << "\n";
        });
        std::cout << "[i] Setup done\n";
    }

    ~RelayNode(){
        // --- Stop & join threads + destroy objects ---
        std::cout << "[i] Closing program...\n";
        audio_socket.is_active.store(false);
        audio_socket.is_recv_running.store(false);
        audio_socket.is_send_running.store(false);
        base_socket.is_recv_running.store(false);
        base_socket.is_send_running.store(false);
        for(int i = 0; i < video_sockets.size(); i++){
            video_sockets[i].is_active.store(false);
            video_sockets[i].is_recv_running.store(false);
            video_sockets[i].is_send_running.store(false);
        }
        // -- audio --
        Pa_StopStream(stream);
        Pa_CloseStream(stream);
        Pa_Terminate();
        opus_encoder_destroy(opus_encoder);
        audio_socket.target_socket->destroy();
        if(audio_socket.recv_thread.joinable()) 
            audio_socket.recv_thread.join();
        if(audio_socket.send_thread.joinable())
            audio_socket.send_thread.join();
        // -- base --
        base_socket.target_socket->destroy();
        if(base_socket.send_thread.joinable()) 
            base_socket.send_thread.join();
        if(base_socket.recv_thread.joinable()) 
            base_socket.recv_thread.join();
        // -- video --
        for(int i = 0; i < video_sockets.size(); i++){
            video_sockets[i].target_socket->destroy();
            if(video_sockets[i].send_thread.joinable()) 
                video_sockets[i].send_thread.join();
            if(video_sockets[i].recv_thread.joinable()) 
                video_sockets[i].recv_thread.join();
        }
        std::cout << "[i] Bye\n";
    }
    void destroy(){
        delete this;
    }
private:
    static int audioCallback(const void* input, void* output, unsigned long frameCount, const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags, void* userData) {
        // --- PortAudio callback requires a static function pointer, so this is needed as a middleman ---
        RelayNode* self = static_cast<RelayNode*>(userData);
        return self->audioProcess(input, output, frameCount, timeInfo, statusFlags);
    }
    int audioProcess(const void* input, void* output, unsigned long frameCount, const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags){
        // --- Callback runs again after paContinue is returned, so no loop required ---
        if(!audio_socket.is_send_running.load()) return paComplete;
        if(!input || !audio_socket.is_active.load()){
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            return paContinue;
        }
        // --- Opus encode + send ---
        unsigned char encoded_data[4096];
        int encoded_size = opus_encode(opus_encoder, (const opus_int16*)input, AUDIO_FRAME_SIZE, encoded_data, sizeof(encoded_data));
        if(encoded_size > 0){
            std::vector<unsigned char> packet(encoded_size);
            std::memcpy(packet.data(), encoded_data, encoded_size);
            std::cout << "sending audio\n";
            audio_socket.target_socket->sendPacket(packet);
        }
        else 
            std::cout << "[w] Empty encoded audio\n";
        return paContinue;
    }
    struct SocketStruct{
        RTPStreamHandler* target_socket;
        std::thread send_thread;
        std::thread recv_thread;
        std::atomic<bool> is_send_running;
        std::atomic<bool> is_recv_running;
        std::atomic<bool> is_active;
        // --- Thingamajig to transfer std::thread ownership ---
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
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr ros2_subscription;
    BasePacket base_packet;
    std::vector<float> ros2_data;
    std::vector<int> gui_data;
    std::mutex ros2_mutex;
    std::mutex gui_mutex;
    PaStream* stream;
    PaError err;
    OpusEncoder* opus_encoder;
    WSADATA wsa_data;
    SocketStruct audio_socket;
    SocketStruct base_socket;
    std::vector<SocketStruct> video_sockets;
};

int main(int argc, char** argv){
    std::cout << "[i] Hi\n";
    //cam_ports = scanWebcams();
    // --- RelayNode handles everything ---
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RelayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}