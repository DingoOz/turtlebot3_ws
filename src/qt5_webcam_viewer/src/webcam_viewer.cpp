#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <QApplication>
#include <QMainWindow>
#include <QLabel>
#include <QImage>
#include <QPixmap>
#include <QTimer>

class WebcamViewer : public rclcpp::Node, public QMainWindow
{
public:
    WebcamViewer() : Node("webcam_viewer"), QMainWindow()
    {
        RCLCPP_INFO(this->get_logger(), "WebcamViewer constructor called");
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/webcam/image_raw", 10,
            std::bind(&WebcamViewer::imageCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscription created");

        image_label_ = new QLabel(this);
        setCentralWidget(image_label_);
        setGeometry(100, 100, 800, 600);

        timer_ = new QTimer(this);
        connect(timer_, &QTimer::timeout, this, &WebcamViewer::updateGUI);
        timer_->start(33); // ~30 fps
        RCLCPP_INFO(this->get_logger(), "Timer started");

        show();
        RCLCPP_INFO(this->get_logger(), "Window shown");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received image: %dx%d, encoding: %s", 
                    msg->width, msg->height, msg->encoding.c_str());
        try {
            cv_bridge::CvImageConstPtr cv_ptr;
            if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
                cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
                RCLCPP_INFO(this->get_logger(), "Converted to BGR8");
            } else {
                cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
                RCLCPP_INFO(this->get_logger(), "Converted to RGB8");
            }
            
            QImage qimg(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, 
                        cv_ptr->image.step, QImage::Format_RGB888);
            if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
                qimg = qimg.rgbSwapped();
            }
            latest_image_ = qimg;
            RCLCPP_INFO(this->get_logger(), "Image converted to QImage");

            // Save the image to disk for debugging
            cv::imwrite("/tmp/received_image.jpg", cv_ptr->image);
            RCLCPP_INFO(this->get_logger(), "Saved image to /tmp/received_image.jpg");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void updateGUI()
    {
        if (!latest_image_.isNull()) {
            RCLCPP_INFO(this->get_logger(), "Updating GUI with new image");
            image_label_->setPixmap(QPixmap::fromImage(latest_image_).scaled(
                image_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
        } else {
            RCLCPP_INFO(this->get_logger(), "No new image to display");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    QLabel* image_label_;
    QTimer* timer_;
    QImage latest_image_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    
    auto node = std::make_shared<WebcamViewer>();
    
    while (rclcpp::ok()) {
        app.processEvents();
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
