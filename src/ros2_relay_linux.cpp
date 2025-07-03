/*
    Relay program - Linux version
    Robotec 2025

    ** may or may not work/be stable **
*/

#include <portaudio.h>
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
#include <opus/opus.h>
#include <unistd.h>
#include <fcntl.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3.hpp"

// --- Initial settings ---
#define M_PI 3.14159265358979323846
#define CONTROLLER_TOPIC "joy"
#define SETTINGS_TOPIC "settings"
#define ESTOP_TOPIC "estop"
#define IMU_TOPIC "imu_data"
#define ENCODER_TOPIC "encoder_position"
#define GAS_TOPIC "mq2_gas"
#define TRACK_TOPIC "track_velocity"
#define THERMAL_TOPIC "thermal_image"
#define SENSOR_TOPIC "magnetometer"
#define JOINT1_TOPIC "joint_base"
#define JOINT2_TOPIC "joint_shouler"
#define JOINT3_TOPIC "joint_elbow"
#define JOINT4_TOPIC "joint_hand"
#define SERVER_IP "192.168.0.131"      //"192.168.0.131"
#define SERVER_PORT 8000
#define AUDIO_SAMPLE_RATE 16000     // 16 kHz
#define AUDIO_FRAME_SIZE 960        // 960 bytes
#define VIDEO_WIDTH 1280
#define VIDEO_HEIGHT 720
#define MAX_UDP_PACKET_SIZE 65507   // 65507 bytes
#define FRAGMENTATION_FLAG 0x8000   // RTP header flag
int exit_code = 0;

std::vector<int> cam_ports = {0};
std::vector<std::string> cam_names = {"aaaa", "bbbb", "cccc", "dddd", "eeeee"};
int mic_port = -1;

std::vector<std::pair<int, std::string>> cam_info;

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
    float magnetometer_x = 0;
    float magnetometer_y = 0;
    float magnetometer_z = 0;
    float gas_ppm = 0;
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

void scanPorts(bool full_scan = false){
    std::cout << "[i] Checking video sources..." << std::endl;
    if(full_scan){
        cam_ports.clear();
        for(int i = 0; i < 5; i++){
            cv::VideoCapture cap(i);
            if(cap.isOpened())
                cam_ports.push_back(i);
            cap.release();
        }
    }
    if(cam_ports.empty())
        std::cout << "[i] No video sources found" << std::endl;
    else
        std::cout << "[i] Found " << cam_ports.size() << " video sources on ports { ";
    for(int i = 0; i < cam_ports.size(); i++){
        cam_info.push_back(std::make_pair(cam_ports[i], cam_names[i]));
        std::cout << cam_ports[i] << (i < cam_ports.size()-1 ? ", " : " }\n") << std::flush;
    }
    int max_checks = Pa_GetDeviceCount();
    const PaDeviceInfo* mic_info;
    std::cout << "[i] Checking " << max_checks << " audio sources..." << std::endl;
    for(int i = 0; i < max_checks; i++) {
        mic_info = Pa_GetDeviceInfo(i);
        if(mic_info->maxInputChannels > 0){
            std::cout << "[i] Found microphone on port " << i << ", name: " << mic_info->name << std::endl;
            if(std::string(mic_info->name) == "default"){
                mic_port = i;
                break;
            }
        }
    }
    if(mic_port == -1){
        std::cout << "[w] No default microphone found. Assigning port 0" << std::endl;
        mic_port = 0;
    }
    else
        std::cout << "[i] Default microphone found on port " << mic_port << std::endl;
}

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
    RelayNode() : Node("relay_node"){
        // --- Full startup ---
        // -- portaudio --
        int stderr_backup = -1;
        int dev_null = -1;
        fflush(stderr);
        stderr_backup = dup(STDERR_FILENO);
        dev_null = open("/dev/null", O_WRONLY);
        if(dev_null != -1 && stderr_backup != -1){
            dup2(dev_null, STDERR_FILENO);
            close(dev_null);
        }
        std::cout << "[i] Initializing PortAudio (stderr silenced)..." << std::endl;

        Pa_Initialize();
        if(stderr_backup != -1){
            fflush(stderr);
            dup2(stderr_backup, STDERR_FILENO);
            close(stderr_backup);
        }
        std::cout << "[i] Scanning device ports..." << std::endl;
        scanPorts(true);
        std::cout << "[i] Starting stream handlers..." << std::endl;
        // -- base + audio --
        base_socket.is_recv_running.store(true);
        base_socket.is_send_running.store(true);
        base_socket.target_socket = new RTPStreamHandler(SERVER_PORT, SERVER_IP, PayloadType::ROS2_ARRAY);
        audio_socket.is_recv_running.store(true);
        audio_socket.is_send_running.store(true);
        audio_socket.is_active.store(false);
        audio_socket.target_socket = new RTPStreamHandler(SERVER_PORT + 2, SERVER_IP, PayloadType::AUDIO_PCM);
        // -- controller --
        controller_socket.is_recv_running.store(true);
        controller_socket.is_send_running.store(false);
        controller_socket.target_socket = new RTPStreamHandler(SERVER_PORT + 4, SERVER_IP, PayloadType::ROS2_ARRAY);
        // -- video --
        for(int i = 0; i < cam_info.size(); i++){
            SocketStruct socket_struct;
            socket_struct.target_socket = new RTPStreamHandler(SERVER_PORT + (2*i) + 6, SERVER_IP, PayloadType::VIDEO_MJPEG);
            video_sockets.push_back(std::move(socket_struct));
        }
        for(int i = 0; i < video_sockets.size(); i++){
            video_sockets[i].is_recv_running.store(true);
            video_sockets[i].is_send_running.store(true);
            video_sockets[i].is_active.store(false);        
        }

        // --- Opus + PortAudio startup ---
        int opus_error;
        opus_encoder = opus_encoder_create(AUDIO_SAMPLE_RATE, 1, OPUS_APPLICATION_AUDIO, &opus_error);
        PaStreamParameters stream_params;
        stream_params.device = mic_port;
        stream_params.channelCount = 1;
        stream_params.sampleFormat = paInt16;
        stream_params.suggestedLatency = Pa_GetDeviceInfo(mic_port)->defaultLowInputLatency;
        stream_params.hostApiSpecificStreamInfo = nullptr;
        Pa_OpenStream(&stream, &stream_params, nullptr, AUDIO_SAMPLE_RATE, AUDIO_FRAME_SIZE, paClipOff, audioCallback, this);
        //Pa_OpenDefaultStream(&stream, 1, 0, paInt16, AUDIO_SAMPLE_RATE, AUDIO_FRAME_SIZE, audioCallback, this);

        // --- ros2 publishers ---
        estop_publisher = this->create_publisher<std_msgs::msg::Bool>(ESTOP_TOPIC, 10);
        controller_publisher = this->create_publisher<sensor_msgs::msg::Joy>(CONTROLLER_TOPIC, 10);
        this->declare_parameter<bool>("launched", false);

        // --- Threads startup - Program begins ---
        std::cout << "[i] Initializing threads..." << std::endl;

        // -- controller --
        controller_socket.recv_thread = std::thread([this](){
            while(controller_socket.is_recv_running.load()){
                std::vector<int> data = controller_socket.target_socket->recvPacket();
                if(data.size() != 20) continue;
                std::vector<float> axes;
                std::vector<int> buttons(data.end()-14, data.end());
                for(int i = 0; i < 6; i++){
                    if(i < 4) 
                        axes.push_back(float(data[i]) / 255.0);
                    else
                        axes.push_back((float(data[i])/255.0) * 2.0 - 1.0);
                }
                sensor_msgs::msg::Joy msg;
                msg.header.stamp = this->now();
                msg.header.frame_id = "joy_frame";
                msg.axes = axes;
                msg.buttons = buttons;
                controller_publisher->publish(msg);
            }
        });

        // -- base --
        base_socket.send_thread = std::thread([this](){
            //std_msgs::msg::Int32MultiArray settings_msg;
            //settings_msg.data = {};
            while(base_socket.is_send_running.load()){
                //settings_publisher->publish(settings_msg);
                std::vector<float> orientation, tracks, sensor, joints, thermal;
                float gas, arms;
                // --- Mutex locks for thread-safe access ---
                {
                    std::lock_guard<std::mutex> lock(sensor_mutex);
                    sensor = sensor_data;
                }
                {
                    std::lock_guard<std::mutex> lock(imu_mutex);
                    orientation = imu_data;
                }
                {
                    std::lock_guard<std::mutex> lock(gas_mutex);
                    gas = gas_data;
                }
                {
                    std::lock_guard<std::mutex> lock(track_mutex);
                    tracks = track_data;
                }
                {
                    std::lock_guard<std::mutex> lock(encoder_mutex);
                    arms = encoder_data;
                }
                {
                    std::lock_guard<std::mutex> lock(joint1_mutex);
                    joints.push_back(joint1_data);
                }
                {
                    std::lock_guard<std::mutex> lock(joint2_mutex);
                    joints.push_back(joint2_data);
                }
                {
                    std::lock_guard<std::mutex> lock(joint3_mutex);
                    joints.push_back(joint3_data);
                }
                {
                    std::lock_guard<std::mutex> lock(joint4_mutex);
                    joints.push_back(joint4_data);
                }
                {
                    std::lock_guard<std::mutex> lock(thermal_mutex);
                    thermal = thermal_data;
                }
                if(sensor.empty())
                    sensor = {0, 0, 0};
                if(orientation.empty())
                    orientation = {0, 0, 0};
                if(tracks.empty())
                    tracks = {0, 0};
                if(joints.size() != 4)
                    joints = {0, 0, 0, 0};
                if(thermal.empty())
                    thermal = std::vector<float>(64, 0);

                BasePacket base_packet = {
                    orientation[0],
                    orientation[1],
                    orientation[2],
                    arms,
                    arms,
                    joints[0],
                    joints[1],
                    joints[2],
                    joints[3],
                    tracks[0],
                    tracks[1],
                    sensor[0],
                    sensor[1],
                    sensor[2],
                    gas
                };
                std::vector<char> packet(sizeof(BasePacket)+sizeof(float)), thermal_encoded(64*sizeof(float));
                float temp = static_cast<float>(cam_info.size());
                std::memcpy(packet.data(), &temp, sizeof(float));
                std::memcpy(packet.data()+sizeof(float), &base_packet, sizeof(BasePacket));
                for(int i = 0; i < cam_info.size(); i++){
                    int str_size = cam_info[i].second.length();
                    std::vector<char> fragment(sizeof(int)+str_size);
                    std::memcpy(fragment.data(), &str_size, sizeof(int));
                    std::memcpy(fragment.data()+sizeof(int), cam_info[i].second.data(), str_size);
                    packet.insert(packet.end(), fragment.begin(), fragment.end());
                }
                std::memcpy(thermal_encoded.data(), thermal.data(), 64*sizeof(float));
                packet.insert(packet.end(), thermal_encoded.begin(), thermal_encoded.end());
                base_socket.target_socket->sendPacket(packet);
                std::this_thread::sleep_for(std::chrono::milliseconds(33));
            }
        });
        base_socket.recv_thread = std::thread([this](){
            while(base_socket.is_recv_running.load()){
                std::vector<int> data = base_socket.target_socket->recvPacket();
                if(data.size() == 0 || data[0] != 0) continue;
                else if(data[1] == 0)
                    std::cout << "[i] GUI connected" << std::endl;
                else if(data[1] == 1)
                    std::cout << "[i] GUI disconnected" << std::endl;
                else if(data[1] == -1){
                    std::cout << "[i] E-Stop called" << std::endl;
                    std_msgs::msg::Bool estop_msg;
                    estop_msg.data = true;
                    estop_publisher->publish(estop_msg);
                }
                else if(data[1] == -2){
                    if(this->get_parameter("launched").as_bool()){
                        std::cout << "[i] Restart called" << std::endl;
                        exit_code = 1;
                        rclcpp::shutdown();
                    }
                    else
                        std::cout << "[w] Restart called but node wasn't launched. Skipping..." << std::endl;
                    break;
                }
                else if(data[1] != 2) 
                    std::cout << "[w] Invalid base packet received" << std::endl;

                if(data[1] != 2){
                    audio_socket.is_active.store(false);
                    for(int i = 0; i < video_sockets.size(); i++){
                        video_sockets[i].is_active.store(false);
                    }
                }
                // --- Mutex lock for thread-safe updates ---
                std::lock_guard<std::mutex> lock(gui_mutex);
                gui_data = data;
            }
        });
        // -- audio --
        audio_socket.recv_thread = std::thread([this](){
            while(audio_socket.is_recv_running.load()){
                std::vector<int> data = audio_socket.target_socket->recvPacket();
                if(data.size() == 0 || data[0] != 0) continue;
                audio_socket.is_active.store((bool)data[1]);
                if((bool)data[1])
                    Pa_StartStream(stream);
                else
                    Pa_StopStream(stream);
            }
        });
        // -- video --
        for(int i = 0; i < video_sockets.size(); i++){
            video_sockets[i].send_thread = std::thread([i, this](){
                video_sockets[i].cap = cv::VideoCapture(cam_info[i].first, cv::CAP_V4L2);
                video_sockets[i].cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
                video_sockets[i].cap.set(cv::CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH);
                video_sockets[i].cap.set(cv::CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT);
                if(!video_sockets[i].cap.isOpened()){
                    std::cout << "[e] Could not open webcam on port " << cam_info[i].first << " for video socket " << i << std::endl;
                    return;
                }
                std::cout << "[i] Camera on port " << cam_info[i].first << " reserved" << std::endl;
                cv::Mat frame;
                std::vector<unsigned char> compressed_data;
                while(video_sockets[i].is_send_running.load()){
                    if(video_sockets[i].is_active.load()){
                        // --- ~35ms from frame capture to send, works as a frame rate limiter (max ~30 fps) ---
                        video_sockets[i].cap >> frame;
                        if(frame.empty()){
                            std::cout << "[w] Empty frame captured on camport " << cam_ports[i] << std::endl;
                            continue;
                        }
                        cv::imencode(".jpg", frame, compressed_data, {cv::IMWRITE_JPEG_QUALITY, 40});
                        video_sockets[i].target_socket->sendPacket(compressed_data);
                    }
                    else 
                        std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
                video_sockets[i].cap.release();
            });
            video_sockets[i].recv_thread = std::thread([i, this](){
                while(video_sockets[i].is_recv_running.load()){
                    std::vector<int> data = video_sockets[i].target_socket->recvPacket();
                    if(data.size() <= 1) continue;
                    if(data[0] == 0)
                        video_sockets[i].is_active.store(data[1]);
                    else{
                        std::cout << "[i] Resolution change received for camport: " << cam_info[i].first << " to: " << data[1] << "x" << data[2] << std::endl;
                        video_sockets[i].is_active.store(false);                        
                        video_sockets[i].cap.release();
                        video_sockets[i].cap = cv::VideoCapture(cam_info[i].first, cv::CAP_V4L2);
                        video_sockets[i].cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
                        video_sockets[i].cap.set(cv::CAP_PROP_FRAME_WIDTH, data[1]);
                        video_sockets[i].cap.set(cv::CAP_PROP_FRAME_HEIGHT, data[2]);
                        if(!video_sockets[i].cap.isOpened()){
                            std::cout << "[e] Could not reopen webcam on port " << cam_info[i].first << " for video socket " << i << ". Resetting..." << std::endl;
                            video_sockets[i].cap.release();
                            video_sockets[i].cap = cv::VideoCapture(cam_info[i].first, cv::CAP_V4L2);
                            video_sockets[i].cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
                            video_sockets[i].cap.set(cv::CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH);
                            video_sockets[i].cap.set(cv::CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT);
                        }
                        
                        video_sockets[i].is_active.store(true);
                    }
                }
            });
        }
        // -- ros2 subscribers --
        gas_subscription = this->create_subscription<std_msgs::msg::Float32>(GAS_TOPIC, 10, [this](const std_msgs::msg::Float32 msg){
            // --- Mutex locks for thread-safe updates ---
            std::lock_guard<std::mutex> lock(gas_mutex);
            gas_data = msg.data;
        });
        imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>(IMU_TOPIC, 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg){
            std::vector<float> orientation;
            double roll, pitch, yaw;
            double sinr_cosp = 2 * (msg->orientation.w * msg->orientation.x + msg->orientation.y * msg->orientation.z);
            double cosr_cosp = 1 - 2 * (msg->orientation.x * msg->orientation.x + msg->orientation.y * msg->orientation.y);
            double sinp = 2 * (msg->orientation.w * msg->orientation.y - msg->orientation.z * msg->orientation.x);
            double siny_cosp = 2 * (msg->orientation.w * msg->orientation.z + msg->orientation.x * msg->orientation.y);
            double cosy_cosp = 1 - 2 * (msg->orientation.y * msg->orientation.y + msg->orientation.z * msg->orientation.z);
            roll = std::atan2(sinr_cosp, cosr_cosp) * 180.0 / M_PI;
            if(std::abs(sinp) >= 1)
                pitch = std::copysign(90.0, sinp);
            else
                pitch = std::asin(sinp) * 180.0 / M_PI;
            yaw = std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
            orientation.push_back(static_cast<float>(roll));
            orientation.push_back(static_cast<float>(pitch));
            orientation.push_back(static_cast<float>(yaw));

            // --- Mutex locks for thread-safe updates ---
            std::lock_guard<std::mutex> lock(imu_mutex);
            imu_data = orientation;
        });
        thermal_subscription = this->create_subscription<std_msgs::msg::Float32MultiArray>(THERMAL_TOPIC, 10, [this](const std_msgs::msg::Float32MultiArray msg){
            // --- Mutex locks for thread-safe updates ---
            std::lock_guard<std::mutex> lock(thermal_mutex);
            thermal_data = msg.data;
        });
        track_subscription = this->create_subscription<geometry_msgs::msg::Vector3>(TRACK_TOPIC, 10, [this](const geometry_msgs::msg::Vector3 msg){
            std::vector<float> velocities;
            velocities.push_back(msg.x);
            velocities.push_back(msg.y);
            // --- Mutex locks for thread-safe updates ---
            std::lock_guard<std::mutex> lock(track_mutex);
            track_data = velocities;
        });
        encoder_subscription = this->create_subscription<std_msgs::msg::Float32>(ENCODER_TOPIC, 10, [this](const std_msgs::msg::Float32 msg){
            // --- Mutex locks for thread-safe updates ---
            std::lock_guard<std::mutex> lock(encoder_mutex);
            encoder_data = msg.data;
        });
        sensor_subscription = this->create_subscription<geometry_msgs::msg::Vector3>(SENSOR_TOPIC, 10, [this](const geometry_msgs::msg::Vector3 msg){
            std::vector<float> sensor;
            sensor.push_back(msg.x);
            sensor.push_back(msg.y);
            sensor.push_back(msg.z);
            // --- Mutex locks for thread-safe updates ---
            std::lock_guard<std::mutex> lock(sensor_mutex);
            sensor_data = sensor;
        });
        joint1_subscription = this->create_subscription<std_msgs::msg::Float32>(JOINT1_TOPIC, 10, [this](const std_msgs::msg::Float32 msg){
            // --- Mutex locks for thread-safe updates ---
            std::lock_guard<std::mutex> lock(joint1_mutex);
            joint1_data = msg.data;
        });
        joint2_subscription = this->create_subscription<std_msgs::msg::Float32>(JOINT2_TOPIC, 10, [this](const std_msgs::msg::Float32 msg){
            // --- Mutex locks for thread-safe updates ---
            std::lock_guard<std::mutex> lock(joint2_mutex);
            joint2_data = msg.data;
        });
        joint3_subscription = this->create_subscription<std_msgs::msg::Float32>(JOINT3_TOPIC, 10, [this](const std_msgs::msg::Float32 msg){
            // --- Mutex locks for thread-safe updates ---
            std::lock_guard<std::mutex> lock(joint3_mutex);
            joint3_data = msg.data;
        });
        joint4_subscription = this->create_subscription<std_msgs::msg::Float32>(JOINT4_TOPIC, 10, [this](const std_msgs::msg::Float32 msg){
            // --- Mutex locks for thread-safe updates ---
            std::lock_guard<std::mutex> lock(joint4_mutex);
            joint4_data = msg.data;
        });
        std::cout << "[i] Setup done" << std::endl;
    }
    ~RelayNode(){
        // --- Stop & join threads + destroy objects ---
        std::cout << "[i] Closing program..." << std::endl;
        audio_socket.is_active.store(false);
        audio_socket.is_recv_running.store(false);
        audio_socket.is_send_running.store(false);
        base_socket.is_recv_running.store(false);
        base_socket.is_send_running.store(false);
        controller_socket.is_recv_running.store(false);
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
        std::cout << "[i] Audio channel closed" << std::endl;
        // -- base --
        base_socket.target_socket->destroy();
        if(base_socket.send_thread.joinable()) 
            base_socket.send_thread.join();
        if(base_socket.recv_thread.joinable()) 
            base_socket.recv_thread.join();
        std::cout << "[i] Base channel closed" << std::endl;
        // -- controller --
        controller_socket.target_socket->destroy();
        if(controller_socket.recv_thread.joinable())
            controller_socket.recv_thread.join();
        std::cout << "[i] Controller channel closed" << std::endl;
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
    static int audioCallback(const void* input, void* output, unsigned long frameCount, const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags, void* userData) {
        // --- PortAudio callback requires a static function pointer, so this is needed as a middleman ---
        RelayNode* self = static_cast<RelayNode*>(userData);
        return self->audioProcess(input, output, frameCount, timeInfo, statusFlags);
    }
    int audioProcess(const void* input, void* output, unsigned long frameCount, const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags){
        // --- Callback runs again after paContinue is returned, so no loop required ---
        if(!audio_socket.is_send_running.load()) return paComplete;
        //audio_socket.is_active.store(true);
        if(!input || !audio_socket.is_active.load()) return paContinue;

        // --- Opus encode + send ---
        unsigned char encoded_data[4096];
        int encoded_size = opus_encode(opus_encoder, (const opus_int16*)input, AUDIO_FRAME_SIZE, encoded_data, sizeof(encoded_data));
        if(encoded_size > 0){
            std::vector<unsigned char> packet(encoded_size);
            std::memcpy(packet.data(), encoded_data, encoded_size);
            audio_socket.target_socket->sendPacket(packet);
        }
        else 
            std::cout << "[w] Empty encoded audio" << std::endl;
        return paContinue;
    }
    struct SocketStruct{
        cv::VideoCapture cap;
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

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gas_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr encoder_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr joint1_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr joint2_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr joint3_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr joint4_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr thermal_subscription;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sensor_subscription;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr track_subscription;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_publisher;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr settings_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr controller_publisher;
    float gas_data = 0;
    float encoder_data = 0;
    float joint1_data = 0;
    float joint2_data = 0;
    float joint3_data = 0;
    float joint4_data = 0;
    bool estop_data = false;
    std::vector<float> imu_data = {};
    std::vector<float> thermal_data = {};
    std::vector<float> track_data = {};
    std::vector<float> sensor_data = {};
    std::vector<int> controller_data = {};
    std::vector<int> gui_data = {};
    std::mutex estop_mutex;
    std::mutex imu_mutex;
    std::mutex gas_mutex;
    std::mutex thermal_mutex;
    std::mutex track_mutex;
    std::mutex encoder_mutex;
    std::mutex sensor_mutex;
    std::mutex gui_mutex;
    std::mutex joint1_mutex;
    std::mutex joint2_mutex;
    std::mutex joint3_mutex;
    std::mutex joint4_mutex;
    PaStream* stream;
    PaError err;
    OpusEncoder* opus_encoder;
    SocketStruct audio_socket;
    SocketStruct base_socket;
    SocketStruct controller_socket;
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
