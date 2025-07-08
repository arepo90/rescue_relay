// works:
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/interactive_marker_update.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/msg/motion_plan_request.hpp"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/robot_state.hpp"
#include "moveit_msgs/msg/workspace_parameters.hpp"
#include <cmath>
#include <vector>
#include <mutex>

class Controller : public rclcpp::Node{
public:
    Controller() : Node("rescue_controller"){
        feedback_pub_ = this->create_publisher<visualization_msgs::msg::InteractiveMarkerFeedback>(
            "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback",
            10
        );
        update_sub_ = this->create_subscription<visualization_msgs::msg::InteractiveMarkerUpdate>(
            "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update",
            10,
            std::bind(&Controller::update_callback, this, std::placeholders::_1)
        );
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy2",
            10,
            std::bind(&Controller::joy_callback, this, std::placeholders::_1)
        );
        this->move_group_action_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
            this,
            "move_group"); // This is the default action server name

        // Wait for the action server to be available
        if (!this->move_group_action_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "MoveGroup action server not available after waiting.");
            // Handle error or exit
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Connected to MoveGroup action server.");
        }

        // Initialize TF2 components for getting current pose (explained later)
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        current_pose_.position.x = -0.10242821276187897;
        current_pose_.position.y = 0.0048788683488965034;
        current_pose_.position.z = 0.27170345187187195;
        current_pose_.orientation.x = 0.5630346536636353;
        current_pose_.orientation.y = -1.4204900367076334e-07;
        current_pose_.orientation.z = 0.8264333009719849;
        current_pose_.orientation.w = -1.9982182948297122e-06;
        
        move_increment_ = 0.01;  // 1cm per button press
        rotation_increment_ = 0.05;  // Small rotation increment
        
        // Initialize button states
        button_pressed_.resize(15, false);  // Xbox controller typically has ~15 buttons
        
        RCLCPP_INFO(this->get_logger(), "Xbox Robot Controller started!");
        RCLCPP_INFO(this->get_logger(), "Controls:");
        RCLCPP_INFO(this->get_logger(), "  Left stick: Move X/Y");
        RCLCPP_INFO(this->get_logger(), "  Right stick: Move Z/Rotate Z");
        RCLCPP_INFO(this->get_logger(), "  D-pad: Fine position control");
        RCLCPP_INFO(this->get_logger(), "  Shoulder buttons: Rotate X/Y");
        RCLCPP_INFO(this->get_logger(), "  B button: Reset to initial position");
        RCLCPP_INFO(this->get_logger(), "  Start button: Quit");
    }

private:
rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr move_group_action_client_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void goal_response_callback(
        rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result...");
        }
    }

    void result_callback(
        const rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::WrappedResult& result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
        }
        else if (result.code == rclcpp_action::ResultCode::ABORTED)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted by server.");
        }
        else if (result.code == rclcpp_action::ResultCode::CANCELED)
        {
            RCLCPP_WARN(this->get_logger(), "Goal was canceled by server.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unknown result code: %d", result.code);
        }
        // You can access result->result.error_code to get more detailed MoveIt error codes
    }
    void update_callback(const visualization_msgs::msg::InteractiveMarkerUpdate::SharedPtr msg){
        std::lock_guard<std::mutex> lock(pose_mutex_);
        for (const auto& pose : msg->poses) {
            if(pose.name == "EE:goal_Link6"){
                current_pose_ = pose.pose;
                auto now = this->get_clock()->now();
                RCLCPP_INFO(this->get_logger(), "[UPDATE] Updated pose from robot feedback at time: %ld.%09ld", 
                            now.seconds(), now.nanoseconds() % 1000000000);
                break;
            }
        }
    }
    
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
        // Ensure we have enough axes and buttons
        if (msg->axes.size() < 6 || msg->buttons.size() < 8) {
            RCLCPP_WARN(this->get_logger(), "Insufficient axes or buttons in joy message");
            return;
        }
        
        std::lock_guard<std::mutex> lock(pose_mutex_);
        
        bool moved = false;
        
        tf2::Quaternion current_quat(
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z,
            current_pose_.orientation.w
        );
        tf2::Matrix3x3 rotation_matrix(current_quat);
        
        // Left stick - X/Y movement relative to end effector orientation
        double left_x = 0;  // up - down
        if(msg->buttons[0] && !msg->buttons[1]){
            left_x = 1;
        }
        else if(!msg->buttons[0] && msg->buttons[1]){
            left_x = -1;
        }
        
        double left_y = msg->axes[0];  // side - side 
        
        if (std::abs(left_x) > 0.1 || std::abs(left_y) > 0.1) {  // Deadzone
            // Create movement vector in end effector's local frame
            tf2::Vector3 local_movement(left_x * move_increment_, left_y * move_increment_, 0.0);
            
            // Transform to world frame
            tf2::Vector3 world_movement = rotation_matrix * local_movement;
            
            current_pose_.position.x += world_movement.x();
            current_pose_.position.y += world_movement.y();
            current_pose_.position.z += world_movement.z();
            moved = true;
        }
        
        // Right stick - Z movement and Z rotation relative to end effector
        
        double right_x = 0;  // rot y
        /*
        std::cout << msg->axes[4] << " " << msg->axes[5] << std::endl;
        std::cout << (msg->axes[4] > 0.0) << " " << (msg->axes[5] > 0.0) << std::endl;
        std::cout << (msg->axes[4] > 0.0 && !msg->axes[5] > 0.0) << 
        */
        if(msg->axes[4] > 0.0 && !(msg->axes[5] > 0.0)){
            right_x = -msg->axes[4];
            //std::cout << "here: " << right_x << std::endl;
        }
        else if(!(msg->axes[4] > 0.0) && msg->axes[5] > 0.0){
            right_x = msg->axes[5];
            //std::cout << "not here: " << right_x << std::endl;
        }
        double right_y = msg->axes[1];  // forward backward
        
        if (std::abs(right_y) > 0.1) {  // Deadzone - Z movement
            // Z movement in end effector's local frame
            tf2::Vector3 local_z_movement(0.0, 0.0, right_y * move_increment_);
            tf2::Vector3 world_z_movement = rotation_matrix * local_z_movement;
            
            current_pose_.position.x += world_z_movement.x();
            current_pose_.position.y += world_z_movement.y();
            current_pose_.position.z += world_z_movement.z();
            moved = true;
        }
        
        if (std::abs(right_x) > 0.1) {  // Deadzone - Z rotation
            // Rotate around Z axis in end effector's local frame
            rotate_relative_z(right_x * rotation_increment_);
            moved = true;
        }
        
        // D-pad for fine control (axes 6 and 7 on most controllers)
        if (msg->axes.size() > 7) {
            double hat_x = msg->axes[6];  // D-pad X
            double hat_y = msg->axes[7];  // D-pad Y
            
            if (std::abs(hat_x) > 0.1 || std::abs(hat_y) > 0.1) {
                // Fine movement in end effector's local frame
                tf2::Vector3 local_fine_movement(hat_x * move_increment_ * 0.5, hat_y * move_increment_ * 0.5, 0.0);
                tf2::Vector3 world_fine_movement = rotation_matrix * local_fine_movement;
                
                current_pose_.position.x += world_fine_movement.x();
                current_pose_.position.y += world_fine_movement.y();
                current_pose_.position.z += world_fine_movement.z();
                moved = true;
            }
        }
        
        // Shoulder buttons for X rotation relative to end effector
        double idfk = msg->axes[2];
        if(std::abs(idfk) > 0.1){
            rotate_relative_x(-idfk * rotation_increment_);
            moved = true;
        }
        /*
        bool left_shoulder = msg->buttons[4];   // LB
        bool right_shoulder = msg->buttons[5];  // RB
        
        if (left_shoulder && !button_pressed_[4]) {
            rotate_relative_x(-rotation_increment_);
            moved = true;
            button_pressed_[4] = true;
        } else if (!left_shoulder) {
            button_pressed_[4] = false;
        }
        
        if (right_shoulder && !button_pressed_[5]) {
            rotate_relative_x(rotation_increment_);
            moved = true;
            button_pressed_[5] = true;
        } else if (!right_shoulder) {
            button_pressed_[5] = false;
        }
        */
        
        // Trigger buttons for Y rotation relative to end effector
        if (msg->axes.size() > 5) {

            double idk = msg->axes[3];
            if (std::abs(idk) > 0.1) {
                rotate_relative_y(-idk * rotation_increment_);
                moved = true;
            }
            /*
            double left_trigger = -1   // rot down
            double right_trigger = -1;  // rot up
            if (left_trigger > 0.1) {
                rotate_relative_y(-left_trigger * rotation_increment_);
                moved = true;
            }
            
            if (right_trigger > 0.1) {
                rotate_relative_y(right_trigger * rotation_increment_);
                moved = true;
            }
            */
        }
        
        // B button - Reset to initial position (button 1)
        if (msg->buttons[11] /*&& !button_pressed_[1]*/) {
            reset_position();
            moved = true;
            button_pressed_[1] = true;
        } else if (!msg->buttons[11]) {
            button_pressed_[1] = false;
        }
        
        // Start button - Quit (button 7)
        if (msg->buttons[7]) {
            RCLCPP_INFO(this->get_logger(), "Start button pressed - shutting down");
            rclcpp::shutdown();
            return;
        }
        
        // Publish if moved
        if (moved) {
            publish_feedback();
            auto now = this->get_clock()->now();
            RCLCPP_INFO(this->get_logger(), "[FEEDBACK] Published at local time: %ld.%09ld", 
                        now.seconds(), now.nanoseconds() % 1000000000);
        }
    }
    
    void rotate_relative_x(double angle)
    {
        // Create rotation quaternion around X axis in end effector's local frame
        tf2::Quaternion rotation_quat;
        rotation_quat.setRPY(angle, 0, 0);
        
        // Get current orientation
        tf2::Quaternion current_quat(
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z,
            current_pose_.orientation.w
        );
        
        // Apply rotation in local frame (multiply on the right)
        tf2::Quaternion result = current_quat * rotation_quat;
        result.normalize();
        
        current_pose_.orientation.x = result.x();
        current_pose_.orientation.y = result.y();
        current_pose_.orientation.z = result.z();
        current_pose_.orientation.w = result.w();
    }
    
    void rotate_relative_y(double angle)
    {
        // Create rotation quaternion around Y axis in end effector's local frame
        tf2::Quaternion rotation_quat;
        rotation_quat.setRPY(0, angle, 0);
        
        // Get current orientation
        tf2::Quaternion current_quat(
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z,
            current_pose_.orientation.w
        );
        
        // Apply rotation in local frame (multiply on the right)
        tf2::Quaternion result = current_quat * rotation_quat;
        result.normalize();
        
        current_pose_.orientation.x = result.x();
        current_pose_.orientation.y = result.y();
        current_pose_.orientation.z = result.z();
        current_pose_.orientation.w = result.w();
    }
    
    void rotate_relative_z(double angle)
    {
        // Create rotation quaternion around Z axis in end effector's local frame
        tf2::Quaternion rotation_quat;
        rotation_quat.setRPY(0, 0, angle);
        
        // Get current orientation
        tf2::Quaternion current_quat(
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z,
            current_pose_.orientation.w
        );
        
        // Apply rotation in local frame (multiply on the right)
        tf2::Quaternion result = current_quat * rotation_quat;
        result.normalize();
        
        current_pose_.orientation.x = result.x();
        current_pose_.orientation.y = result.y();
        current_pose_.orientation.z = result.z();
        current_pose_.orientation.w = result.w();
    }
    
    void reset_position()
    {
        current_pose_.position.x = -0.10242821276187897;
        current_pose_.position.y = 0.0048788683488965034;
        current_pose_.position.z = 0.27170345187187195;
        current_pose_.orientation.x = 0.5630346536636353;
        current_pose_.orientation.y = -1.4204900367076334e-07;
        current_pose_.orientation.z = 0.8264333009719849;
        current_pose_.orientation.w = -1.9982182948297122e-06;
        RCLCPP_INFO(this->get_logger(), "Reset to initial position");
    }
    
    void publish_feedback()
    {
        auto feedback_msg = visualization_msgs::msg::InteractiveMarkerFeedback();
        feedback_msg.header.frame_id = "base_link";
        feedback_msg.header.stamp.sec = 0;
        feedback_msg.header.stamp.nanosec = 0;
        feedback_msg.client_id = "/interactive_marker_display";
        feedback_msg.marker_name = "EE:goal_Link6";
        feedback_msg.control_name = "move";
        feedback_msg.event_type = 1;
        feedback_msg.pose = current_pose_;
        feedback_msg.menu_entry_id = 0;
        feedback_msg.mouse_point.x = current_pose_.position.x;
        feedback_msg.mouse_point.y = current_pose_.position.y;
        feedback_msg.mouse_point.z = current_pose_.position.z;
        feedback_msg.mouse_point_valid = true;
        
        feedback_pub_->publish(feedback_msg);
    }
    
    // Member variables
    rclcpp::Publisher<visualization_msgs::msg::InteractiveMarkerFeedback>::SharedPtr feedback_pub_;
    rclcpp::Subscription<visualization_msgs::msg::InteractiveMarkerUpdate>::SharedPtr update_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    
    geometry_msgs::msg::Pose current_pose_;
    double move_increment_;
    double rotation_increment_;
    std::vector<bool> button_pressed_;
    std::mutex pose_mutex_;  // Mutex to protect pose data
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::cout << "starting" << std::endl;
    
    try {
        auto node = std::make_shared<Controller>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception caught: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}