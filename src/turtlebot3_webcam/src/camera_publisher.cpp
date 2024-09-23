#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher() : Node("camera_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("webcam/image_raw", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&CameraPublisher::timer_callback, this));
        
        // Try to open the camera with V4L2 backend
        cap_.open(4, cv::CAP_V4L2);
        
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera!");
            rclcpp::shutdown();
        } else {
            RCLCPP_INFO(this->get_logger(), "Camera opened successfully");
			RCLCPP_INFO(this->get_logger(), "Camera backend: %s", cap_.getBackendName().c_str());
			RCLCPP_INFO(this->get_logger(), "Camera FPS: %f", cap_.get(cv::CAP_PROP_FPS));
			RCLCPP_INFO(this->get_logger(), "Camera resolution: %fx%f", cap_.get(cv::CAP_PROP_FRAME_WIDTH), cap_.get(cv::CAP_PROP_FRAME_HEIGHT));

         
            
            // Set camera properties
            cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
            cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
            cap_.set(cv::CAP_PROP_FPS, 30);
        }
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        if (cap_.read(frame)) {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            publisher_->publish(*msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to capture frame");
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}
