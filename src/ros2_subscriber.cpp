#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h> // Still useful for header and encoding type

#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <functional> // Required for std::bind, though we'll switch to lambda

// Define a maximum number of cameras to check to prevent infinite loops
// if a non-existent device index somehow opens without error.
const int MAX_CAMERAS_TO_CHECK = 10;

// Define the default frame rate (in Hz) for publishing images
const double DEFAULT_FRAME_RATE = 30.0;

class CompressedWebcamPublisher : public rclcpp::Node
{
public:
    CompressedWebcamPublisher()
    : rclcpp::Node("compressed_webcam_publisher")
    {
        RCLCPP_INFO(this->get_logger(), "Starting webcam scanner and publisher...");

        // Iterate through possible camera indices to find available webcams
        for (int i = 0; i < MAX_CAMERAS_TO_CHECK; ++i)
        {
            cv::VideoCapture cap(i); // Open the camera by index

            if (!cap.isOpened())
            {
                // If the camera cannot be opened, it likely doesn't exist at this index.
                // We'll continue checking higher indices, but this often means we've
                // exhausted available cameras.
                RCLCPP_WARN(this->get_logger(), "Could not open camera at index %d. Skipping.", i);
                continue;
            }

            // Successfully opened a camera! Store its capture object.
            RCLCPP_INFO(this->get_logger(), "Found camera at index %d. Setting up publisher.", i);
            // We store a copy of the VideoCapture object. When the vector is resized,
            // or elements are moved, the copy constructor will be called.
            // This is generally safe for cv::VideoCapture as it manages its internal pointer.
            webcam_captures_.push_back(std::make_pair(i, std::move(cap))); // Use std::move for efficiency

            // Create a publisher for this camera's compressed image feed
            // The topic name will be unique for each camera, e.g., /camera0/compressed, /camera1/compressed
            std::string topic_name = "camera" + std::to_string(i) + "/compressed";
            auto publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(topic_name, 10);
            webcam_publishers_.push_back(publisher);

            // Create a timer for this camera to publish frames at a specified rate
            // The timer period is calculated from the desired frame rate.
            auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / DEFAULT_FRAME_RATE));

            // Use a lambda function for the timer callback.
            // This is generally more robust and readable than std::bind for member functions
            // with arguments, especially when dealing with ROS2's timer API.
            auto timer = this->create_wall_timer(
                timer_period,
                [this, i]() { // Capture 'this' pointer and the current camera index 'i' by value
                    this->publish_webcam_frame(i);
                }
            );
            webcam_timers_.push_back(timer);

            // Set some default properties for the camera if needed (e.g., resolution)
            // Note: Not all cameras support all resolutions.
            // cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
            // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        }

        if (webcam_captures_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No webcams found! Exiting.");
            // Consider shutting down the node if no cameras are found
            // rclcpp::shutdown(); // Uncomment if you want the node to exit immediately
        }
    }

    ~CompressedWebcamPublisher()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down webcam publisher.");
        // Release all capture devices when the node is destroyed
        for (auto& pair : webcam_captures_)
        {
            if (pair.second.isOpened())
            {
                pair.second.release();
            }
        }
    }

private:
    // Vector to store pairs of camera index and its cv::VideoCapture object
    std::vector<std::pair<int, cv::VideoCapture>> webcam_captures_;
    // Vector to store publishers for each webcam
    std::vector<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr> webcam_publishers_;
    // Vector to store timers for each webcam
    std::vector<rclcpp::TimerBase::SharedPtr> webcam_timers_;

    /**
     * @brief Publishes a compressed frame from the specified webcam.
     * This function is called by the timer associated with each webcam.
     * @param camera_index The index of the camera to capture from.
     */
    void publish_webcam_frame(int camera_index)
    {
        // Find the capture object for the given camera_index
        cv::VideoCapture* cap = nullptr;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher = nullptr;

        // Use a range-based for loop for cleaner iteration
        for (auto& pair : webcam_captures_)
        {
            if (pair.first == camera_index)
            {
                cap = &pair.second;
                // Find the corresponding publisher by index.
                // This assumes webcam_captures_ and webcam_publishers_ maintain the same order of cameras.
                // A more robust approach would be to store the publisher directly with the capture object,
                // or use a map from camera index to publisher.
                size_t pub_idx = 0;
                for (size_t k = 0; k < webcam_captures_.size(); ++k) {
                    if (webcam_captures_[k].first == camera_index) {
                        publisher = webcam_publishers_[k];
                        break;
                    }
                }
                break;
            }
        }


        if (cap == nullptr || publisher == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "Error: Could not find capture object or publisher for camera index %d. This should not happen.", camera_index);
            return;
        }

        cv::Mat frame;
        *cap >> frame; // Capture a new frame from the camera

        if (frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Failed to capture frame from camera %d. Is it still connected or busy?", camera_index);
            return;
        }

        // Convert OpenCV Mat to ROS2 CompressedImage message
        try
        {
            std_msgs::msg::Header header;
            header.stamp = this->now(); // Set the timestamp to the current ROS time
            header.frame_id = "camera" + std::to_string(camera_index); // Set the frame ID

            // Encode the OpenCV Mat to JPEG format
            std::vector<uchar> buffer;
            // The compression quality can be adjusted here (0-100, 95 is default for JPEG)
            std::vector<int> compression_params;
            compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
            compression_params.push_back(90); // Set JPEG quality to 90

            cv::imencode(".jpeg", frame, buffer, compression_params);

            // Create a new CompressedImage message
            auto compressed_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
            compressed_msg->header = header;
            compressed_msg->format = "jpeg"; // Specify the format
            compressed_msg->data = buffer; // Assign the compressed data

            publisher->publish(*compressed_msg);
        }
        catch (const cv::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV exception during image encoding for camera %d: %s", camera_index, e.what());
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Standard exception during image conversion for camera %d: %s", camera_index, e.what());
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CompressedWebcamPublisher>());
    rclcpp::shutdown();
    return 0;
}
