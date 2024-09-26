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
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSlider>
#include <QDateTime>
#include <QDir>
#include <QToolTip>

class WebcamViewer : public rclcpp::Node, public QMainWindow
{
public:
    WebcamViewer() : Node("webcam_viewer"), QMainWindow(), frame_count_(0), last_fps_print_time_(this->now())
    {
        // Create central widget and layout
        QWidget* central_widget = new QWidget(this);
        QVBoxLayout* main_layout = new QVBoxLayout(central_widget);
        setCentralWidget(central_widget);

        // Create image label for video display
        image_label_ = new QLabel(this);
        image_label_->setAlignment(Qt::AlignCenter);
        main_layout->addWidget(image_label_);

        // Create overlay widget and layout
        QWidget *overlay_widget = new QWidget(this);
        QHBoxLayout *overlay_layout = new QHBoxLayout(overlay_widget);
        overlay_widget->setStyleSheet("background-color: rgba(0, 0, 0, 50);");

        // Add buttons to overlay
        // Add buttons to overlay
        QPushButton *snapshot_button = new QPushButton("Take Snapshot", this);
        snapshot_button->setStyleSheet(
            "QPushButton {"
            "   background-color: rgba(255, 255, 255, 180);"
            "   color: black;"
            "   border: 1px solid black;"
            "   border-radius: 5px;"
            "   padding: 5px;"
            "}"
            "QPushButton:hover {"
            "   background-color: rgba(200, 200, 200, 180);"
            "}"
            "QToolTip {"
            "   background-color: white;"
            "   color: black;"
            "   border: 1px solid black;"
            "   padding: 5px;"
            "}");
        QString tooltip = QString("Snapshots will be saved in:\n%1").arg(getSnapshotDirectory());
        snapshot_button->setToolTip(tooltip);
        connect(snapshot_button, &QPushButton::clicked, this, &WebcamViewer::takeSnapshot);
        overlay_layout->addWidget(snapshot_button);

        // Add slider for brightness adjustment
        QSlider *brightness_slider = new QSlider(Qt::Horizontal, this);
        brightness_slider->setRange(-100, 100);
        brightness_slider->setValue(0);
        brightness_slider->setFixedHeight(40);
        connect(brightness_slider, &QSlider::valueChanged, this, &WebcamViewer::adjustBrightness);
        overlay_layout->addWidget(brightness_slider);

        // Add overlay to main layout
        main_layout->addWidget(overlay_widget);

        // Set up ROS subscription
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/webcam/image_raw", 10,
            std::bind(&WebcamViewer::imageCallback, this, std::placeholders::_1));

        // Set up timer for GUI updates
        timer_ = new QTimer(this);
        connect(timer_, &QTimer::timeout, this, &WebcamViewer::updateGUI);
        timer_->start(33); // ~30 fps

        setGeometry(100, 100, 800, 550);
        show();
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        frame_count_++;
        auto current_time = this->now();
        if ((current_time - last_fps_print_time_).seconds() >= 5.0) {
            double fps = frame_count_ / 5.0;
            RCLCPP_INFO(this->get_logger(), "FPS: %.2f", fps);
            frame_count_ = 0;
            last_fps_print_time_ = current_time;
        }

        try {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat adjusted_image;
            cv::convertScaleAbs(cv_ptr->image, adjusted_image, 1.0, brightness_);
            QImage qimg(adjusted_image.data, adjusted_image.cols, adjusted_image.rows, 
                        adjusted_image.step, QImage::Format_RGB888);
            latest_image_ = qimg.rgbSwapped();
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void updateGUI()
    {
        if (!latest_image_.isNull()) {
            image_label_->setPixmap(QPixmap::fromImage(latest_image_).scaled(image_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
        }
    }

    // Tell us where snapshots are being stored
    QString getSnapshotDirectory() const
    {
        return QDir::currentPath(); // This returns the current working directory
    }

    void takeSnapshot()
    {
        if (!latest_image_.isNull())
        {
            QString filename = QString("snapshot_%1.png").arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"));
            latest_image_.save(filename);
            RCLCPP_INFO(this->get_logger(), "Snapshot saved: %s", filename.toStdString().c_str());
        }
    }

    void adjustBrightness(int value)
    {
        brightness_ = value;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    QLabel* image_label_;
    QTimer* timer_;
    QImage latest_image_;
    int frame_count_;
    rclcpp::Time last_fps_print_time_;
    int brightness_ = 0;
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