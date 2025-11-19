#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>

class RCCarController : public rclcpp::Node
{
public:
    RCCarController() : Node("rc_car_controller")
    {
        // Create publisher for cmd_vel
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // Initialize speeds
        linear_speed_ = 0.5;   // m/s
        angular_speed_ = 1.0;  // rad/s
        speed_increment_ = 0.1;
        
        current_linear_ = 0.0;
        current_angular_ = 0.0;
        
        // Store terminal settings
        tcgetattr(STDIN_FILENO, &old_terminal_);
        
        RCLCPP_INFO(this->get_logger(), "RC Car Controller Started!");
        RCLCPP_INFO(this->get_logger(), "Controls:");
        RCLCPP_INFO(this->get_logger(), "  Z/S: Forward/Backward");
        RCLCPP_INFO(this->get_logger(), "  Q/D: Turn Left/Right");
        RCLCPP_INFO(this->get_logger(), "  A/E: Increase/Decrease speed");
        RCLCPP_INFO(this->get_logger(), "  Space: Stop");
        RCLCPP_INFO(this->get_logger(), "  X: Quit");
    }
    
    ~RCCarController()
    {
        // Restore terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal_);
        stop();
    }
    
    char getKey()
    {
        // Set terminal to raw mode
        struct termios raw;
        tcgetattr(STDIN_FILENO, &raw);
        raw.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        
        char c;
        if (read(STDIN_FILENO, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }
        
        // Restore terminal
        tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal_);
        
        return c;
    }
    
    void publishVelocity()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = current_linear_;
        msg.angular.z = current_angular_;
        publisher_->publish(msg);
        
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), 
            *this->get_clock(), 
            500,  // milliseconds
            "Linear: %.2f m/s, Angular: %.2f rad/s", 
            current_linear_, 
            current_angular_
        );
    }
    
    void stop()
    {
        current_linear_ = 0.0;
        current_angular_ = 0.0;
        publishVelocity();
        RCLCPP_INFO(this->get_logger(), "Stopped!");
    }
    
    void run()
    {
        while (rclcpp::ok())
        {
            char key = getKey();
            
            // Convert to lowercase
            if (key >= 'A' && key <= 'Z')
            {
                key = key + 32;
            }
            
            switch (key)
            {
                case 'z':
                    // Forward
                    current_linear_ = linear_speed_;
                    current_angular_ = 0.0;
                    break;
                    
                case 's':
                    // Backward
                    current_linear_ = -linear_speed_;
                    current_angular_ = 0.0;
                    break;
                    
                case 'q':
                    // Turn left
                    if (current_linear_ != 0.0)
                    {
                        current_angular_ = angular_speed_;
                    }
                    else
                    {
                        // Rotate in place
                        current_linear_ = 0.0;
                        current_angular_ = angular_speed_;
                    }
                    break;
                    
                case 'd':
                    // Turn right
                    if (current_linear_ != 0.0)
                    {
                        current_angular_ = -angular_speed_;
                    }
                    else
                    {
                        // Rotate in place
                        current_linear_ = 0.0;
                        current_angular_ = -angular_speed_;
                    }
                    break;
                    
                case ' ':
                    // Stop
                    stop();
                    break;
                    
                case 'a':
                    // Increase speed
                    linear_speed_ += speed_increment_;
                    angular_speed_ += speed_increment_;
                    RCLCPP_INFO(this->get_logger(), "Speed increased: %.2f m/s", linear_speed_);
                    break;
                    
                case 'e':
                    // Decrease speed
                    linear_speed_ = std::max(0.1, linear_speed_ - speed_increment_);
                    angular_speed_ = std::max(0.1, angular_speed_ - speed_increment_);
                    RCLCPP_INFO(this->get_logger(), "Speed decreased: %.2f m/s", linear_speed_);
                    break;
                    
                case 'x':
                    // Quit
                    RCLCPP_INFO(this->get_logger(), "Exiting...");
                    stop();
                    return;
                    
                default:
                    // Invalid key - stop
                    stop();
                    continue;
            }
            
            publishVelocity();
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    
    double linear_speed_;
    double angular_speed_;
    double speed_increment_;
    
    double current_linear_;
    double current_angular_;
    
    struct termios old_terminal_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto controller = std::make_shared<RCCarController>();
    
    try
    {
        controller->run();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(controller->get_logger(), "Error: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}