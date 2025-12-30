#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>

class LidarVisualizer : public rclcpp::Node
{
public:
    LidarVisualizer()
        : Node("lidar_visualizer"),
          window_size_(800),
          scale_(40.0),  // pixels per meter
          robot_x_(0.0),
          robot_y_(0.0),
          robot_yaw_(0.0)
    {
        // Create subscribers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&LidarVisualizer::scanCallback, this, std::placeholders::_1)
        );

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&LidarVisualizer::odomCallback, this, std::placeholders::_1)
        );

        // Create OpenCV window
        cv::namedWindow("Lidar Visualization", cv::WINDOW_AUTOSIZE);

        // Create timer for updating display (30 Hz)
        display_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&LidarVisualizer::updateDisplay, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Lidar Visualizer Started!");
        RCLCPP_INFO(this->get_logger(), "Window size: %dx%d pixels", window_size_, window_size_);
        RCLCPP_INFO(this->get_logger(), "Scale: %.1f pixels/meter", scale_);
        RCLCPP_INFO(this->get_logger(), "Controls:");
        RCLCPP_INFO(this->get_logger(), "  '+' / '-' : Zoom in/out");
        RCLCPP_INFO(this->get_logger(), "  'r' : Reset view");
        RCLCPP_INFO(this->get_logger(), "  ESC : Quit");
    }
    
    ~LidarVisualizer()
    {
        cv::destroyAllWindows();
    }
    
private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        
        // Store scan data
        lidar_points_.clear();
        
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float range = msg->ranges[i];
            
            // Filter invalid ranges
            if (std::isfinite(range) && 
                range >= msg->range_min && 
                range <= msg->range_max)
            {
                // Calculate angle for this point
                float angle = msg->angle_min + i * msg->angle_increment;
                
                // Convert polar to Cartesian (relative to robot)
                float x = range * std::cos(angle);
                float y = range * std::sin(angle);
                
                lidar_points_.push_back({x, y});
            }
        }
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        
        // Extract robot position
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        
        // Extract yaw from quaternion
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        
        // Convert quaternion to yaw
        robot_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy),
                                1.0 - 2.0 * (qy * qy + qz * qz));
    }
    
    void updateDisplay()
    {
        // Create blank image (black background)
        cv::Mat image(window_size_, window_size_, CV_8UC3, cv::Scalar(0, 0, 0));
        
        // Draw grid
        drawGrid(image);
        
        // Draw lidar points
        {
            std::lock_guard<std::mutex> lock(scan_mutex_);
            drawLidarPoints(image);
        }
        
        // Draw robot
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            drawRobot(image);
        }
        
        // Draw info text
        drawInfo(image);
        
        // Display image
        cv::imshow("Lidar Visualization", image);
        
        // Handle keyboard input
        int key = cv::waitKey(1);
        handleKeyboard(key);
    }
    
    void drawGrid(cv::Mat& image)
    {
        cv::Point center(window_size_ / 2, window_size_ / 2);
        
        // Draw grid lines every meter
        double grid_spacing = scale_;  // 1 meter
        
        for (int i = -10; i <= 10; ++i)
        {
            int offset = static_cast<int>(i * grid_spacing);
            
            // Vertical lines
            cv::line(image,
                    cv::Point(center.x + offset, 0),
                    cv::Point(center.x + offset, window_size_),
                    cv::Scalar(40, 40, 40), 1);
            
            // Horizontal lines
            cv::line(image,
                    cv::Point(0, center.y + offset),
                    cv::Point(window_size_, center.y + offset),
                    cv::Scalar(40, 40, 40), 1);
        }
        
        // Draw center cross (brighter)
        cv::line(image,
                cv::Point(center.x, 0),
                cv::Point(center.x, window_size_),
                cv::Scalar(80, 80, 80), 1);
        cv::line(image,
                cv::Point(0, center.y),
                cv::Point(window_size_, center.y),
                cv::Scalar(80, 80, 80), 1);
    }
    
    void drawLidarPoints(cv::Mat& image)
    {
        cv::Point center(window_size_ / 2, window_size_ / 2);
        
        for (const auto& point : lidar_points_)
        {
            // Convert from meters to pixels
            int px = center.x + static_cast<int>(point.x * scale_);
            int py = center.y - static_cast<int>(point.y * scale_);  // Flip Y axis
            
            // Check if point is within image bounds
            if (px >= 0 && px < window_size_ && py >= 0 && py < window_size_)
            {
                // Draw point in green
                cv::circle(image, cv::Point(px, py), 2, cv::Scalar(0, 255, 0), -1);
            }
        }
    }
    
    void drawRobot(cv::Mat& image)
    {
        cv::Point center(window_size_ / 2, window_size_ / 2);
        
        // Robot position is always at center in robot-centric view
        int robot_px = center.x;
        int robot_py = center.y;
        
        // Draw robot as red circle
        cv::circle(image, cv::Point(robot_px, robot_py), 8, cv::Scalar(0, 0, 255), -1);
        cv::circle(image, cv::Point(robot_px, robot_py), 10, cv::Scalar(0, 0, 255), 2);
        
        // Draw orientation arrow
        int arrow_length = 20;
        int arrow_x = robot_px + static_cast<int>(arrow_length * std::cos(robot_yaw_));
        int arrow_y = robot_py - static_cast<int>(arrow_length * std::sin(robot_yaw_));
        
        cv::arrowedLine(image,
                       cv::Point(robot_px, robot_py),
                       cv::Point(arrow_x, arrow_y),
                       cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0, 0.3);
    }
    
    void drawInfo(cv::Mat& image)
    {
        // Draw semi-transparent background for text
        cv::rectangle(image,
                     cv::Point(5, 5),
                     cv::Point(250, 100),
                     cv::Scalar(0, 0, 0),
                     -1);
        cv::rectangle(image,
                     cv::Point(5, 5),
                     cv::Point(250, 100),
                     cv::Scalar(100, 100, 100),
                     1);
        
        // Draw text info
        std::string pos_text, yaw_text, points_text, scale_text;
        
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            pos_text = "Position: (" + 
                      std::to_string(robot_x_).substr(0, 4) + ", " + 
                      std::to_string(robot_y_).substr(0, 4) + ")";
            yaw_text = "Yaw: " + 
                      std::to_string(robot_yaw_ * 180.0 / M_PI).substr(0, 5) + " deg";
        }
        
        {
            std::lock_guard<std::mutex> lock(scan_mutex_);
            points_text = "Points: " + std::to_string(lidar_points_.size());
        }
        
        scale_text = "Scale: " + std::to_string(static_cast<int>(scale_)) + " px/m";
        
        cv::putText(image, pos_text, cv::Point(10, 25),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
        cv::putText(image, yaw_text, cv::Point(10, 45),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
        cv::putText(image, points_text, cv::Point(10, 65),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);
        cv::putText(image, scale_text, cv::Point(10, 85),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);
    }
    
    void handleKeyboard(int key)
    {
        if (key == 27)  // ESC
        {
            RCLCPP_INFO(this->get_logger(), "Shutting down...");
            rclcpp::shutdown();
        }
        else if (key == '+' || key == '=')
        {
            scale_ *= 1.2;
            RCLCPP_INFO(this->get_logger(), "Zoom in: %.1f px/m", scale_);
        }
        else if (key == '-' || key == '_')
        {
            scale_ /= 1.2;
            scale_ = std::max(scale_, 5.0);  // Minimum zoom
            RCLCPP_INFO(this->get_logger(), "Zoom out: %.1f px/m", scale_);
        }
        else if (key == 'r')
        {
            scale_ = 40.0;
            RCLCPP_INFO(this->get_logger(), "Reset view");
        }
    }
    
    // Parameters
    int window_size_;
    double scale_;
    
    // Robot state
    double robot_x_, robot_y_, robot_yaw_;
    std::mutex odom_mutex_;
    
    // Lidar data
    struct Point2D
    {
        float x, y;
    };
    std::vector<Point2D> lidar_points_;
    std::mutex scan_mutex_;
    
    // ROS 2 components
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr display_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto visualizer = std::make_shared<LidarVisualizer>();
    rclcpp::spin(visualizer);
    
    rclcpp::shutdown();
    return 0;
}
