/*
    Tests n stuff
    Not actual subscriber implementation
*/

#include <zlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
using namespace std;
using namespace chrono;
using namespace cv;

int main(){
    VideoCapture cap(0);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    if(!cap.isOpened()){
        cout << "cam not open\n";
        return -1;
    }
    Mat frame;
    cap >> frame;
    if(frame.empty()){
        std::cout << "Empty frame\n";
        return -1;;
    }
    vector<uchar> raw;
    if(!frame.isContinuous()){
        cv::Mat continuous = frame.clone();
        raw = std::vector<uchar>(continuous.data, continuous.data + continuous.total() * continuous.elemSize());
    }
    else 
        raw = std::vector<uchar>(frame.data, frame.data + frame.total() * frame.elemSize());

    uLong dest_len = compressBound(raw.size());
    vector<uchar> compressed(dest_len);

    auto start = high_resolution_clock::now();
    int result = compress(compressed.data(), &dest_len, raw.data(), raw.size());
    auto end = high_resolution_clock::now();

    if(result == Z_OK){
        compressed.resize(dest_len);
        cout << "raw size: " << raw.size() << " compressed size: " << compressed.size() << " ratio: " << double(raw.size())/double(compressed.size()) << " time: " << duration_cast<milliseconds>(end-start).count() << "\n"; 
    } 
    else{
        cout << "compression error\n";
        return -1;
    }

    raw.clear();
    vector<uchar> compressed2;
    start = high_resolution_clock::now();
    imencode(".jpg", frame, compressed2, {cv::IMWRITE_JPEG_QUALITY, 50});
    end = high_resolution_clock::now();
    cout << "raw size: " << raw.size() << " compressed size: " << compressed2.size() << " ratio: " << double(raw.size())/double(compressed2.size()) << " time: " << duration_cast<milliseconds>(end-start).count() << "\n"; 
    Mat final = imdecode(compressed2, IMREAD_COLOR);
    cv::cvtColor(final, final, cv::COLOR_BGR2RGB);
    imshow("??", final);
    return 0;
}