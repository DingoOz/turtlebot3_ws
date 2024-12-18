#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include <opencv2/opencv.hpp>
#include <libudev.h>
#include <memory>
#include <string>

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher()
        : Node("camera_publisher")
        , it_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}))  // Correct initialization
    {
        // Create publisher using image_transport
        publisher_ = it_.advertise("webcam/image_raw", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(33),
                                       std::bind(&CameraPublisher::timer_callback, this));

        // Find LifeCam HD-3000
        std::string device_path = find_lifecam();

        if (device_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "LifeCam HD-3000 not found");
            device_path = "/dev/video0";  // Fallback to default camera
            RCLCPP_WARN(this->get_logger(), "Using default camera at %s", device_path.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Found LifeCam HD-3000 at: %s", device_path.c_str());
        }

        // Open the camera
        cap_.open(device_path, cv::CAP_V4L2);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera at %s", device_path.c_str());
            return;
        }

        // Set camera properties
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
        cap_.set(cv::CAP_PROP_FPS, 30);

        // Get actual camera properties
        double actual_width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
        double actual_height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
        double actual_fps = cap_.get(cv::CAP_PROP_FPS);

        RCLCPP_INFO(this->get_logger(), "Camera initialized with resolution: %.0f x %.0f at %.0f FPS",
                    actual_width, actual_height, actual_fps);
    }

private:
    std::string find_lifecam()
    {
        struct udev* udev = udev_new();
        if (!udev) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create udev context");
            return "";
        }

        std::string result;
        struct udev_enumerate* enumerate = udev_enumerate_new(udev);
        udev_enumerate_add_match_subsystem(enumerate, "video4linux");
        udev_enumerate_scan_devices(enumerate);
        struct udev_list_entry* devices = udev_enumerate_get_list_entry(enumerate);
        struct udev_list_entry* dev_list_entry;

        udev_list_entry_foreach(dev_list_entry, devices) {
            const char* path = udev_list_entry_get_name(dev_list_entry);
            struct udev_device* dev = udev_device_new_from_syspath(udev, path);
            if (!dev) continue;

            const char* dev_path = udev_device_get_devnode(dev);
            if (!dev_path) {
                udev_device_unref(dev);
                continue;
            }

            struct udev_device* parent =
                udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");

            if (parent) {
                const char* vendor = udev_device_get_sysattr_value(parent, "idVendor");
                const char* product = udev_device_get_sysattr_value(parent, "idProduct");

                // Check for Microsoft LifeCam HD-3000 (Vendor ID: 045e, Product ID: 0779)
                if (vendor && product &&
                    strcmp(vendor, "045e") == 0 &&
                    strcmp(product, "0779") == 0) {
                    result = dev_path;
                }
            }
            udev_device_unref(dev);
            if (!result.empty()) break;
        }

        udev_enumerate_unref(enumerate);
        udev_unref(udev);
        return result;
    }

    void timer_callback()
    {
        cv::Mat frame;
        if (!cap_.read(frame)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read frame from camera");
            return;
        }

        // Convert OpenCV image to ROS message
        sensor_msgs::msg::Image::SharedPtr msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_frame";

        publisher_.publish(msg);
    }

    image_transport::ImageTransport it_;
    image_transport::Publisher publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}
