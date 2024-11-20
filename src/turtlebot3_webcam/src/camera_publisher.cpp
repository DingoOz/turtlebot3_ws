#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <libudev.h>
#include <cstring>

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher() : Node("camera_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/webcam/image_raw", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&CameraPublisher::timer_callback, this));
        
        std::string device_path = find_lifecam_device();
        if (device_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to find LifeCam HD-3000!");
            rclcpp::shutdown();
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Found LifeCam HD-3000 at: %s", device_path.c_str());
        
        // Try to open the camera with V4L2 backend
        cap_.open(device_path, cv::CAP_V4L2);
        
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

    std::string find_lifecam_device()
    {
        struct udev* udev = udev_new();
        if (!udev) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create udev context");
            return "";
        }

        struct udev_enumerate* enumerate = udev_enumerate_new(udev);
        udev_enumerate_add_match_subsystem(enumerate, "video4linux");
        udev_enumerate_scan_devices(enumerate);

        struct udev_list_entry* devices = udev_enumerate_get_list_entry(enumerate);
        struct udev_list_entry* entry;

        std::string result;

        udev_list_entry_foreach(entry, devices) {
            const char* path = udev_list_entry_get_name(entry);
            struct udev_device* dev = udev_device_new_from_syspath(udev, path);

            struct udev_device* parent = udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");
            if (parent) {
                const char* vendor = udev_device_get_sysattr_value(parent, "idVendor");
                const char* product = udev_device_get_sysattr_value(parent, "idProduct");

                if (vendor && product && strcmp(vendor, "045e") == 0 && strcmp(product, "0810") == 0) {
                    result = udev_device_get_devnode(dev);
                    udev_device_unref(dev);
                    break;
                }
            }

            udev_device_unref(dev);
        }

        udev_enumerate_unref(enumerate);
        udev_unref(udev);

        return result;
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
