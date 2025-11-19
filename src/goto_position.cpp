#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class GotoPositionController : public rclcpp::Node
{
public:
    GotoPositionController(double target_x, double target_y)
        : Node("goto_position_controller"),
          target_x_(target_x),
          target_y_(target_y),
          current_x_(0.0),
          current_y_(0.0),
          current_yaw_(0.0),
          reached_goal_(false)
    {
        // PID gains for linear velocity
        kp_linear_ = 0.5;
        ki_linear_ = 0.0;
        kd_linear_ = 0.1;
        
        // PID gains for angular velocity
        kp_angular_ = 2.0;
        ki_angular_ = 0.0;
        kd_angular_ = 0.5;
        
        // Tolerances
        position_tolerance_ = 0.1;  // meters
        angle_tolerance_ = 0.1;      // radians
        
        // Limits
        max_linear_speed_ = 0.5;
        max_angular_speed_ = 1.0;
        
        // PID error tracking
        prev_linear_error_ = 0.0;
        prev_angular_error_ = 0.0;
        integral_linear_ = 0.0;
        integral_angular_ = 0.0;
        
        // Create publisher and subscriber
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&GotoPositionController::odomCallback, this, std::placeholders::_1)
        );
        
        // Create timer for control loop (50Hz)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&GotoPositionController::controlLoop, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "=== Goto Position Controller Started ===");
        RCLCPP_INFO(this->get_logger(), "Target position: (%.2f, %.2f)", target_x_, target_y_);
        RCLCPP_INFO(this->get_logger(), "Waiting for odometry...");
    }
    
private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract current position
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        
        // Extract current orientation (convert quaternion to yaw)
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        
        double roll, pitch;
        tf2::Matrix3x3(q).getRPY(roll, pitch, current_yaw_);
    }
    
    void controlLoop()
    {
        if (reached_goal_)
        {
            return;
        }
        
        // Calculate distance and angle to target
        double dx = target_x_ - current_x_;
        double dy = target_y_ - current_y_;
        double distance = std::sqrt(dx * dx + dy * dy);
        double target_angle = std::atan2(dy, dx);
        
        // Calculate angular error (normalize to [-pi, pi])
        double angular_error = target_angle - current_yaw_;
        angular_error = std::atan2(std::sin(angular_error), std::cos(angular_error));
        
        // Check if goal is reached
        if (distance < position_tolerance_)
        {
            stopRobot();
            RCLCPP_INFO(this->get_logger(), "=== Goal Reached! ===");
            RCLCPP_INFO(this->get_logger(), "Final position: (%.3f, %.3f)", current_x_, current_y_);
            RCLCPP_INFO(this->get_logger(), "Distance error: %.3f m", distance);
            reached_goal_ = true;
            return;
        }
        
        // PID control
        geometry_msgs::msg::Twist cmd_vel;
        
        // First, rotate towards target if angle error is large
        if (std::abs(angular_error) > angle_tolerance_)
        {
            // Only rotate, don't move forward
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = computePID(
                angular_error,
                prev_angular_error_,
                integral_angular_,
                kp_angular_,
                ki_angular_,
                kd_angular_,
                max_angular_speed_
            );
            
            prev_angular_error_ = angular_error;
        }
        else
        {
            // Move forward and adjust angle
            cmd_vel.linear.x = computePID(
                distance,
                prev_linear_error_,
                integral_linear_,
                kp_linear_,
                ki_linear_,
                kd_linear_,
                max_linear_speed_
            );
            
            cmd_vel.angular.z = computePID(
                angular_error,
                prev_angular_error_,
                integral_angular_,
                kp_angular_,
                ki_angular_,
                kd_angular_,
                max_angular_speed_
            );
            
            prev_linear_error_ = distance;
            prev_angular_error_ = angular_error;
        }
        
        // Publish velocity command
        cmd_vel_pub_->publish(cmd_vel);
        
        // Log status periodically
        static int counter = 0;
        if (++counter % 25 == 0)  // Every 0.5 seconds
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Position: (%.2f, %.2f) | Distance: %.2fm | Angle error: %.2f°",
                current_x_, current_y_, distance, angular_error * 180.0 / M_PI
            );
        }
    }
    
    double computePID(double error, double prev_error, double& integral,
                     double kp, double ki, double kd, double max_output)
    {
        // Proportional term
        double p_term = kp * error;
        
        // Integral term (with anti-windup)
        integral += error * 0.02;  // dt = 20ms
        integral = std::clamp(integral, -1.0, 1.0);
        double i_term = ki * integral;
        
        // Derivative term
        double derivative = (error - prev_error) / 0.02;
        double d_term = kd * derivative;
        
        // Compute output
        double output = p_term + i_term + d_term;
        
        // Clamp to maximum speed
        return std::clamp(output, -max_output, max_output);
    }
    
    void stopRobot()
    {
        auto cmd_vel = geometry_msgs::msg::Twist();
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
        
        // Reset PID
        integral_linear_ = 0.0;
        integral_angular_ = 0.0;
        prev_linear_error_ = 0.0;
        prev_angular_error_ = 0.0;
    }
    
    // Target position
    double target_x_, target_y_;
    
    // Current state
    double current_x_, current_y_, current_yaw_;
    bool reached_goal_;
    
    // PID parameters
    double kp_linear_, ki_linear_, kd_linear_;
    double kp_angular_, ki_angular_, kd_angular_;
    
    // PID state
    double prev_linear_error_, prev_angular_error_;
    double integral_linear_, integral_angular_;
    
    // Control parameters
    double position_tolerance_;
    double angle_tolerance_;
    double max_linear_speed_;
    double max_angular_speed_;
    
    // ROS 2 components
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // Check if target coordinates are provided
    if (argc != 3)
    {
        std::cerr << "Usage: ros2 run simple_rc_car goto_position <x> <y>" << std::endl;
        std::cerr << "Example: ros2 run simple_rc_car goto_position 2.0 1.5" << std::endl;
        return 1;
    }
    
    // Parse target coordinates
    double target_x, target_y;
    try
    {
        target_x = std::stod(argv[1]);
        target_y = std::stod(argv[2]);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: Invalid coordinates. Please provide numeric values." << std::endl;
        return 1;
    }
    
    // Create and run controller
    auto controller = std::make_shared<GotoPositionController>(target_x, target_y);
    rclcpp::spin(controller);
    
    rclcpp::shutdown();
    return 0;
}