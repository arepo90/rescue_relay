/*
    Relay program - Windows version
    Robotec 2025

    ** may or may not be long-term stable **
*/

#ifndef NOMINMAX
    #define NOMINMAX
#endif

#include <portaudio.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>
#include <cstdint>
#include <cstdlib>
#include <stdlib.h>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opus/opus.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#pragma comment(lib, "ws2_32.lib")
#define M_PI 3.14159265358979323846

// --- Initial settings ---
#define IMU_TOPIC "imu_data"
#define ENCODER_TOPIC "encoder_position"
#define GAS_TOPIC "mq2_gas"
#define TRACK_TOPIC "track_velocity"
#define THERMAL_TOPIC "thermal_image" 
#define SENSOR_TOPIC "magnetometer"
#define JOINT1_TOPIC "joint_base"
#define JOINT2_TOPIC "joint_shoulder"
#define JOINT3_TOPIC "joint_elbow"
#define JOINT4_TOPIC "joint_hand"
#define SERVER_IP "127.0.0.1"
#define SERVER_PORT 8000
#define FRAGMENTATION_FLAG 0x8000   // RTP header flag
#define MAX_UDP_PACKET_SIZE 65507   // 65507 bytes
#define AUDIO_SAMPLE_RATE 16000     // 16 kHz
#define AUDIO_FRAME_SIZE 960        // 960 bytes
#define VIDEO_WIDTH 1280            // 1280 pixels
#define VIDEO_HEIGHT 720            // 720 pixels

std::vector<int> cam_ports = {0, 1};
std::vector<std::string> cam_names = {"Front camera", "Arm camera"};

int mic_port = -1;
std::map<int, std::string> cam_info;

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
    if(full_scan){
        cam_ports.clear();
        for(int i = 0; i < 5; i++){
            cv::VideoCapture cap(i);
            if(cap.isOpened()){
                std::cout << "[i] FOUND VIDEO SOURCE ON PORT " << i << "\n";
                cam_ports.push_back(i);
            }
            cap.release();
        }
    }
    for(int i = 0; i < cam_ports.size(); i++){
        cam_info[i] = cam_names[i];
    }
    std::cout << "[i] FOUND " << cam_ports.size() << " VIDEO SOURCES\n";
    int max_checks = Pa_GetDeviceCount();
    const PaDeviceInfo* mic_info;
    for(int i = 0; i < max_checks; i++) {
        mic_info = Pa_GetDeviceInfo(i);
        if(mic_info->maxInputChannels > 0){
            std::cout << "[i] FOUND DEFAULT MICROPHONE ON PORT " << i << ", name: " << mic_info->name << "\n";
            if(std::string(mic_info->name) == "Default")
                mic_port = i;
        }
    }
    if(mic_port == -1){
        std::cout << "[w] No default microphone found. Assigning port 0\n";
        mic_port = 0;
    }
}

cv::Mat thermalAdaptiveGradient(cv::Mat frame){
    double min_val, max_val;
    cv::Mat grad_x, grad_y, output;
    cv::resize(frame, frame, cv::Size(64, 64), 0, 0, cv::INTER_CUBIC);
    cv::minMaxLoc(frame, &min_val, &max_val);
    cv::Sobel(frame, grad_x, CV_32F, 1, 0, 3);
    cv::Sobel(frame, grad_y, CV_32F, 0, 1, 3);
    cv::magnitude(grad_x, grad_y, output);
    cv::resize(output, output, cv::Size(64, 64), 0, 0, cv::INTER_LINEAR);
    cv::normalize(output, output, 0, 1, cv::NORM_MINMAX);
    for(int i = 0; i < frame.rows; i++){
        for(int j = 0; j < frame.cols; j++){
            output.at<float>(i, j) = frame.at<float>(i, j) * (1.0f + output.at<float>(i, j) * 0.3f);
        }
    }
    cv::normalize(output, output, min_val, max_val, cv::NORM_MINMAX);
    cv::normalize(output, output, 0, 255, cv::NORM_MINMAX);
    output.convertTo(output, CV_8U);
    cv::applyColorMap(output, output, cv::COLORMAP_JET);
    cv::resize(output, output, cv::Size(1920, 1080), cv::INTER_NEAREST);
    return output;
}

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
        ::bind(recv_socket, (struct sockaddr*)&recv_socket_address, socket_address_size);

        std::cout << "[i] Channel created, bound to ports (" << port << ", " << port + 1 << ")\n";
    }
    ~RTPStreamHandler(){
        std::cout << "[i] Closing channel (" << stream->port << ", " << stream->port + 1 << ")\n";
        shutdown(recv_socket, SD_BOTH);
        closesocket(send_socket);
        closesocket(recv_socket);
    }
    void destroy(){
        delete this;
    }
    template <typename T> void sendPacket(std::vector<T> data){
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

            if(sendto(send_socket, (const char*)packet.data(), packet.size(), 0, (struct sockaddr*)&send_socket_address, socket_address_size) == SOCKET_ERROR){
                std::cout << "[w] Packet send failed on fragment " << i << ". Winsock error: " << WSAGetLastError() << "\n";
            }
        } 
    }
    std::vector<int> recvPacket(){
        // --- Receive (non-fragmented) packet ---
        std::vector<char> packet(4096);
        int bytes_received = recvfrom(recv_socket, packet.data(), packet.size(), 0, (struct sockaddr*)&recv_socket_address, &socket_address_size);
        if(bytes_received == SOCKET_ERROR){
            int error = WSAGetLastError(); 
            if(error != 10004) 
                std::cout << "[e] Packet recv failed. Winsock error: " << error << "\n";
            return {};
        }
        else if(bytes_received <= static_cast<int>(sizeof(RTPHeader))){
            std::cout << "[w] Empty packet received\n";
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
    SOCKET send_socket;
    SOCKET recv_socket;
    sockaddr_in send_socket_address;
    sockaddr_in recv_socket_address;
    int socket_address_size = sizeof(send_socket_address);
};

class RelayNode : public rclcpp::Node{
public:
    RelayNode() : Node("relay_node"){
        // --- Sockets + ROS2 startup ---
        std::cout << "[i] Starting stream handlers...\n";
        WSAStartup(MAKEWORD(2, 2), &wsa_data);
        // -- base + audio --
        base_socket.target_socket = new RTPStreamHandler(SERVER_PORT, SERVER_IP, PayloadType::ROS2_ARRAY);
        base_socket.is_recv_running.store(true);
        base_socket.is_send_running.store(true);
        audio_socket.is_recv_running.store(true);
        audio_socket.is_send_running.store(true);
        audio_socket.target_socket = new RTPStreamHandler(SERVER_PORT + 2, SERVER_IP, PayloadType::AUDIO_PCM);
        // -- video --
        for(int i = 0; i < cam_ports.size()+1; i++){
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
        PaStreamParameters stream_params;
        stream_params.device = mic_port;
        stream_params.channelCount = 1;
        stream_params.sampleFormat = paInt16;
        stream_params.suggestedLatency = Pa_GetDeviceInfo(mic_port)->defaultLowInputLatency;
        stream_params.hostApiSpecificStreamInfo = nullptr;
        Pa_OpenStream(&stream, &stream_params, nullptr, AUDIO_SAMPLE_RATE, AUDIO_FRAME_SIZE, paClipOff, audioCallback, this);
        //Pa_OpenDefaultStream(&stream, 1, 0, paInt16, AUDIO_SAMPLE_RATE, AUDIO_FRAME_SIZE, audioCallback, this);

        // --- Threads startup - Program begins ---
        std::cout << "[i] Initializing threads...\n";
        // -- base --
        base_socket.send_thread = std::thread([this](){
            while(base_socket.is_send_running.load()){
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
                for(auto it = cam_info.begin(); it != cam_info.end(); it++){
                    int str_size = it->second.length();
                    std::vector<char> fragment(sizeof(int)+str_size);
                    std::memcpy(fragment.data(), &str_size, sizeof(int));
                    std::memcpy(fragment.data()+sizeof(int), &(it->second), str_size);
                    packet.insert(packet.end(), fragment.begin(), fragment.end());
                }
                std::memcpy(thermal_encoded.data(), thermal.data(), 64*sizeof(float));
                packet.insert(packet.end(), thermal_encoded.begin(), thermal_encoded.end());
                base_socket.target_socket->sendPacket(packet);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        });
        base_socket.recv_thread = std::thread([this](){
            while(base_socket.is_recv_running.load()){
                std::vector<int> data = base_socket.target_socket->recvPacket();
                if(data.size() == 0 || data[0] != 0) continue;
                else if(data[1] == 0)
                    std::cout << "[i] GUI connected\n";
                else if(data[1] == -1)
                    std::cout << "[i] GUI disconnected\n";
                audio_socket.is_active.store(false);
                Pa_StopStream(stream);
                for(int i = 0; i < video_sockets.size(); i++){
                    video_sockets[i].is_active.store(false);
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
                audio_socket.is_active.store(static_cast<bool>(data[1]));
                if(static_cast<bool>(data[1]))
                    Pa_StartStream(stream);
                else    
                    Pa_StopStream(stream);
            }
        });
        // -- video --
        for(int i = 0; i < video_sockets.size(); i++){
            video_sockets[i].send_thread = std::thread([i, this](){
                cv::VideoCapture cap;
                if(i < cam_ports.size()){
                    cap.open(cam_ports[i]);
                    cap.set(cv::CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH);
                    cap.set(cv::CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT);
                    if(!cap.isOpened()){
                        std::cout << "[e] Could not open webcam on port " << cam_ports[i] << " for video socket " << i << "\n";
                        return;
                    }
                    std::cout << "[i] Camera on port " << cam_ports[i] << " reserved\n";
                }
                cv::Mat frame;
                std::vector<unsigned char> compressed_data;
                while(video_sockets[i].is_send_running.load()){
                    if(video_sockets[i].is_active.load()){
                        // --- ~35ms from frame capture to send, works as a frame rate limiter (max ~30 fps) ---
                        if(i < cam_ports.size())
                            cap >> frame;
                        else{
                            std::vector<float> data;
                            {
                                std::lock_guard<std::mutex> lock(thermal_mutex);
                                data = thermal_data;
                            }
                            if(data.size() != 64){
                                std::cout << "[w] SUBSECTION FILTER THERMAL | Invalid payload: missing pixels";
                                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                                continue;
                            }
                            frame = cv::Mat(8, 8, CV_32F);
                            std::memcpy(frame.data, data.data(), data.size()*sizeof(float));
                            frame = thermalAdaptiveGradient(frame);
                            /*
                            cv::resize(frame, frame, cv::Size(64, 64), 0, 0, cv::INTER_CUBIC);
                            double minVal, maxVal;
                            cv::minMaxLoc(frame, &minVal, &maxVal);
                            cv::Mat gradX, gradY, gradMag;
                            cv::Sobel(frame, gradX, CV_32F, 1, 0, 3);
                            cv::Sobel(frame, gradY, CV_32F, 0, 1, 3);
                            cv::magnitude(gradX, gradY, gradMag);
                            cv::Mat upGradient;
                            cv::resize(gradMag, upGradient, cv::Size(64, 64), 0, 0, cv::INTER_LINEAR);
                            cv::normalize(upGradient, upGradient, 0, 1, cv::NORM_MINMAX);
                            cv::Mat enhanced = frame.clone();
                            for (int y = 0; y < enhanced.rows; y++) {
                                for (int x = 0; x < enhanced.cols; x++) {
                                    float gradientValue = upGradient.at<float>(y, x);
                                    // Apply stronger enhancement in areas of high thermal gradient
                                    enhanced.at<float>(y, x) = frame.at<float>(y, x) * (1.0f + gradientValue * 0.3f);
                                }
                            }
                            cv::normalize(enhanced, enhanced, minVal, maxVal, cv::NORM_MINMAX);
                            cv::Mat normalized;
                            cv::normalize(enhanced, normalized, 0, 255, cv::NORM_MINMAX);
                            normalized.convertTo(normalized, CV_8U);
                            cv::Mat colormap;
                            cv::applyColorMap(normalized, colormap, cv::COLORMAP_JET);
                            frame = colormap.clone();
                            */
                            /*
                            cv::normalize(frame, frame, 0, 255, cv::NORM_MINMAX);
                            frame.convertTo(frame, CV_8U);
                            cv::applyColorMap(frame, frame, cv::COLORMAP_JET);
                            cv::resize(frame, frame, cv::Size(1920, 1080), cv::INTER_NEAREST);
                            */
                            std::this_thread::sleep_for(std::chrono::milliseconds(35));
                        }
                        if(frame.empty()){
                            std::cout << "[w] Empty frame captured on camport " << cam_ports[i] << "n";
                            break;
                        }
                        cv::imencode(".jpg", frame, compressed_data, {cv::IMWRITE_JPEG_QUALITY, 40});
                        video_sockets[i].target_socket->sendPacket(compressed_data);
                    }
                    else 
                        std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
                cap.release();
            });
            video_sockets[i].recv_thread = std::thread([i, this](){
                while(video_sockets[i].is_recv_running.load()){
                    std::vector<int> data = video_sockets[i].target_socket->recvPacket();
                    if(data.size() == 0) continue;
                    video_sockets[i].is_active.store(data[1]);
                }
            });
        }
        // -- ros2 topics --
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
        std::cout << "[i] Audio channel closed\n";
        // -- base --
        base_socket.target_socket->destroy();
        if(base_socket.send_thread.joinable()) 
            base_socket.send_thread.join();
        if(base_socket.recv_thread.joinable()) 
            base_socket.recv_thread.join();
        std::cout << "[i] Base channel closed\n";
        // -- video --
        for(int i = 0; i < video_sockets.size(); i++){
            video_sockets[i].target_socket->destroy();
            if(video_sockets[i].send_thread.joinable()) 
                video_sockets[i].send_thread.join();
            if(video_sockets[i].recv_thread.joinable()) 
                video_sockets[i].recv_thread.join();
        }
        std::cout << "[i] Video channels closed\n";
        WSACleanup();
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
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr thermal_subscription;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sensor_subscription;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr track_subscription;
    float gas_data = 0;
    float encoder_data = 0;
    float joint1_data = 0;
    float joint2_data = 0;
    float joint3_data = 0;
    float joint4_data = 0;
    std::vector<float> imu_data = {};
    std::vector<float> thermal_data = {50.5025, 56.9884, 61.9211, 64.6447, 64.6447, 61.9211, 56.9884, 50.5025, 56.9884, 64.6447, 70.8452, 74.5049, 74.5049, 70.8452, 64.6447, 56.9884, 61.9211, 70.8452, 78.7868, 84.1886, 84.1886, 78.7868, 70.8452, 61.9211, 64.6447, 74.5049, 84.1886, 92.9289, 92.9289, 84.1886, 74.5049, 64.6447, 64.6447, 74.5049, 84.1886, 92.9289, 92.9289, 84.1886, 74.5049, 64.6447, 61.9211, 70.8452, 78.7868, 84.1886, 84.1886, 78.7868, 70.8452, 61.9211, 56.9884, 64.6447, 70.8452, 74.5049, 74.5049, 70.8452, 64.6447, 56.9884, 50.5025, 56.9884, 61.9211, 64.6447, 64.6447, 61.9211, 56.9884, 50.5025};
    std::vector<float> track_data = {};
    std::vector<float> sensor_data = {};
    std::vector<int> gui_data = {};
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
    WSADATA wsa_data;
    SocketStruct audio_socket;
    SocketStruct base_socket;
    std::vector<SocketStruct> video_sockets;
};

int main(int argc, char** argv){
    std::cout << "[i] Hi Windows\n";
    Pa_Initialize();
    scanPorts(true);
    // --- RelayNode handles everything ---
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RelayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}