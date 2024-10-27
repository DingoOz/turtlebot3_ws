#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraViewer : public rclcpp::Node
{
public:
    CameraViewer() : Node("camera_viewer", 
                         rclcpp::NodeOptions()
                         .allow_undeclared_parameters(true)
                         .automatically_declare_parameters_from_overrides(true))
    {
        RCLCPP_INFO(this->get_logger(), "Starting Camera Viewer Node");
        
        // Print network-related environment variables
        print_network_info();
        
        // Create explicit QoS profile for reliable transport
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
            .reliable()
            .durability_volatile()
            .deadline(std::chrono::milliseconds(200))
            .liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC)
            .liveliness_lease_duration(std::chrono::milliseconds(1000));
        
        // Create window early
        try {
            cv::namedWindow("TurtleBot3 Webcam", cv::WINDOW_AUTOSIZE);
            RCLCPP_INFO(this->get_logger(), "Created display window");
            
            // Test pattern
            cv::Mat test_image(240, 320, CV_8UC3, cv::Scalar(0, 255, 0));
            cv::putText(test_image, "Waiting for camera feed...", 
                       cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 
                       0.5, cv::Scalar(255, 255, 255), 1);
            cv::imshow("TurtleBot3 Webcam", test_image);
            cv::waitKey(1);
            RCLCPP_INFO(this->get_logger(), "Displayed test pattern");
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV error: %s", e.what());
        }

        // Subscribe with absolute topic name
        subscription_ = create_subscription<sensor_msgs::msg::Image>(
            "/webcam/image_raw", 
            qos,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                this->image_callback(msg);
            });
            
        RCLCPP_INFO(this->get_logger(), "Created subscription to /webcam/image_raw");
        
        // Add timers for monitoring
        discovery_timer_ = create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&CameraViewer::discovery_callback, this));
            
        topic_timer_ = create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&CameraViewer::topic_callback, this));
    }

private:
    void print_network_info()
    {
        RCLCPP_INFO(this->get_logger(), "Network Configuration:");
        RCLCPP_INFO(this->get_logger(), "  ROS_DOMAIN_ID: %s", 
                    std::getenv("ROS_DOMAIN_ID") ? std::getenv("ROS_DOMAIN_ID") : "not set");
        RCLCPP_INFO(this->get_logger(), "  ROS_LOCALHOST_ONLY: %s", 
                    std::getenv("ROS_LOCALHOST_ONLY") ? std::getenv("ROS_LOCALHOST_ONLY") : "not set");
        RCLCPP_INFO(this->get_logger(), "  RMW_IMPLEMENTATION: %s", 
                    std::getenv("RMW_IMPLEMENTATION") ? std::getenv("RMW_IMPLEMENTATION") : "not set");
    }
    
    void discovery_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Discovery Update:");
        RCLCPP_INFO(this->get_logger(), "  Subscription active: %s", 
                    subscription_->get_publisher_count() > 0 ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  Publisher count: %zu", 
                    subscription_->get_publisher_count());
        RCLCPP_INFO(this->get_logger(), "  Frames received: %d", frame_count_);
    }
    
    void topic_callback()
    {
        auto topic_names_and_types = this->get_topic_names_and_types();
        RCLCPP_INFO(this->get_logger(), "Available topics:");
        for (const auto& topic : topic_names_and_types) {
            RCLCPP_INFO(this->get_logger(), "  %s", topic.first.c_str());
            for (const auto& type : topic.second) {
                RCLCPP_INFO(this->get_logger(), "    Type: %s", type.c_str());
            }
        }
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        frame_count_++;
        RCLCPP_INFO(this->get_logger(), "Received frame %d, encoding: %s, size: %dx%d", 
                    frame_count_, msg->encoding.c_str(), msg->width, msg->height);
        
        try {
            auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            cv::imshow("TurtleBot3 Webcam", cv_ptr->image);
            cv::waitKey(1);
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV error: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr discovery_timer_;
    rclcpp::TimerBase::SharedPtr topic_timer_;
    int frame_count_{0};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // Force FastDDS/FastRTPS discovery
    setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp", 1);
    
    auto node = std::make_shared<CameraViewer>();
    rclcpp::spin(node);
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}

