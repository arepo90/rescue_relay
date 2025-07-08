#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <SDL2/SDL.h>
#include <vector>
#include <cmath>

class Joystick : public rclcpp::Node {
public:
    Joystick()
        : Node("joystick"), dead_zone(8000) {
        
        joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("joy", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Joystick::publishJoy, this)
        );

        if (SDL_Init(SDL_INIT_GAMECONTROLLER) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to init SDL: %s", SDL_GetError());
            rclcpp::shutdown();
        }

        if (SDL_NumJoysticks() < 1) {
            RCLCPP_ERROR(this->get_logger(), "No controllers found.");
            rclcpp::shutdown();
        }

        controller = SDL_GameControllerOpen(0);
        if (!controller) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open controller: %s", SDL_GetError());
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "Controller connected.");
    }

    ~Joystick() {
        if (controller) SDL_GameControllerClose(controller);
        SDL_Quit();
    }

private:
    SDL_GameController* controller = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    const int dead_zone;

    void publishJoy() {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) return;
        }

        sensor_msgs::msg::Joy msg;
        msg.header.stamp = this->now();

        std::vector<float> axes(6, 0.0f);  // LEFTX, LEFTY, RIGHTX, RIGHTY, LT, RT
        std::vector<int> raw_states(20, 0);

        auto readAxis = [&](SDL_GameControllerAxis axis) {
            Sint16 val = SDL_GameControllerGetAxis(controller, axis);
            return (std::abs(val) < dead_zone) ? 0 : val;
        };

        raw_states[0] = readAxis(SDL_CONTROLLER_AXIS_LEFTX);
        raw_states[1] = -readAxis(SDL_CONTROLLER_AXIS_LEFTY);
        raw_states[2] = readAxis(SDL_CONTROLLER_AXIS_RIGHTX);
        raw_states[3] = -readAxis(SDL_CONTROLLER_AXIS_RIGHTY);
        raw_states[4] = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_TRIGGERLEFT) / 128;
        raw_states[5] = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_TRIGGERRIGHT) / 128;

        /*
        for (int i = 0; i < 14; ++i) {
            raw_states[6 + i] = SDL_GameControllerGetButton(controller, static_cast<SDL_GameControllerButton>(i)) ? 1 : 0;
        }
        */
        raw_states[6] = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_DPAD_UP) ? 1 : 0;
        raw_states[7] = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_DPAD_DOWN) ? 1 : 0;
        raw_states[8] = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_DPAD_LEFT) ? 1 : 0;
        raw_states[9] = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_DPAD_RIGHT) ? 1 : 0;
        raw_states[10] = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_START) ? 1 : 0;
        raw_states[11] = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_BACK) ? 1 : 0;
        raw_states[12] = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_LEFTSTICK) ? 1 : 0;
        raw_states[13] = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_RIGHTSTICK) ? 1 : 0;
        raw_states[14] = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_LEFTSHOULDER) ? 1 : 0;
        raw_states[15] = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER) ? 1 : 0;
        raw_states[16] = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_A) ? 1 : 0;
        raw_states[17] = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_B) ? 1 : 0;
        raw_states[18] = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_X) ? 1 : 0;
        raw_states[19] = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_Y) ? 1 : 0;

        // Fill Joy message
        axes[0] = raw_states[0] / 32768.0f;
        axes[1] = raw_states[1] / 32768.0f;
        axes[2] = raw_states[2] / 32768.0f;
        axes[3] = raw_states[3] / 32768.0f;
        axes[4] = ((raw_states[4]/255.0) * 2.0 - 1.0);
        axes[5] = ((raw_states[5]/255.0) * 2.0 - 1.0);

        std::vector<int32_t> buttons(raw_states.begin() + 6, raw_states.end());

        msg.axes = axes;
        msg.buttons = buttons;

        joy_pub_->publish(msg);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::cout << "starting" << std::endl;
    
    try {
        auto node = std::make_shared<Joystick>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception caught: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}