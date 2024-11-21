#include "dashboard_turtlebot3_cpp/dashboard_node.hpp"
#include <QApplication>
#include <QDebug>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QLabel>
#include <QDockWidget>
#include <QTimer>
#include <QProgressBar>
#include <QMetaObject>

VideoWidget::VideoWidget(QWidget* parent) : QLabel(parent) {
  setScaledContents(true);
}

void VideoWidget::updateFrame(const cv::Mat& frame) {
  if (frame.empty()) {
    qDebug() << "Received empty frame";
    return;
  }

  QImage qimg = mat_to_qimage(frame);
  if (qimg.isNull()) {
    qDebug() << "Failed to convert cv::Mat to QImage";
  } else {
    qDebug() << "Updating frame: " << qimg.width() << "x" << qimg.height();
    QPixmap pixmap = QPixmap::fromImage(qimg);
    if (pixmap.isNull()) {
      qDebug() << "Failed to convert QImage to QPixmap";
    } else {
      setPixmap(pixmap);
    }
  }
}

QImage VideoWidget::mat_to_qimage(const cv::Mat& mat) {
  if(mat.type() == CV_8UC3) {
    cv::Mat rgb;
    cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
    return QImage((uchar*)rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888);
  }
  return QImage();
}

TwistWidget::TwistWidget(QWidget* parent) : QWidget(parent) {
  layout = new QVBoxLayout(this);
  linear_label = new QLabel("Linear: x=0.00, y=0.00, z=0.00", this);
  angular_label = new QLabel("Angular: x=0.00, y=0.00, z=0.00", this);

  linear_progress = new QProgressBar(this);
  linear_progress->setRange(-100, 100);
  linear_progress->setFormat("Linear X: %v");

  angular_progress = new QProgressBar(this);
  angular_progress->setRange(-100, 100);
  angular_progress->setFormat("Angular Z: %v");

  layout->addWidget(linear_label);
  layout->addWidget(angular_label);
  layout->addWidget(linear_progress);
  layout->addWidget(angular_progress);
  setLayout(layout);
}

void TwistWidget::updateTwist(const geometry_msgs::msg::Twist::SharedPtr msg) {
  linear_label->setText(QString("Linear: x=%1, y=%2, z=%3")
    .arg(msg->linear.x, 0, 'f', 2)
    .arg(msg->linear.y, 0, 'f', 2)
    .arg(msg->linear.z, 0, 'f', 2));
  angular_label->setText(QString("Angular: x=%1, y=%2, z=%3")
    .arg(msg->angular.x, 0, 'f', 2)
    .arg(msg->angular.y, 0, 'f', 2)
    .arg(msg->angular.z, 0, 'f', 2));

  int linear_progress_value = static_cast<int>(msg->linear.x * 100);
  linear_progress->setValue(linear_progress_value);

  int angular_progress_value = static_cast<int>(msg->angular.z / 1.5 * 100);
  angular_progress->setValue(angular_progress_value);
}

DashboardNode::DashboardNode() : Node("dashboard_turtlebot3") {
  twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&DashboardNode::twistCallback, this, std::placeholders::_1));
  image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "webcam/image_raw", 10, std::bind(&DashboardNode::imageCallback, this, std::placeholders::_1));
}

geometry_msgs::msg::Twist::SharedPtr DashboardNode::getLatestTwist() {
  return latest_twist_;
}

sensor_msgs::msg::Image::SharedPtr DashboardNode::getLatestImage() {
  return latest_image_;
}

void DashboardNode::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  latest_twist_ = msg;
}

void DashboardNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received image frame");
  latest_image_ = msg;
}

DashboardWindow::DashboardWindow(std::shared_ptr<DashboardNode> node, QWidget* parent)
  : QMainWindow(parent), dashboard_node_(node) {
  setWindowTitle("Turtlebot3 Dashboard");
  setGeometry(100, 100, 800, 600);

  video_widget = new VideoWidget(this);
  video_widget->setMinimumSize(640, 480);
  setCentralWidget(video_widget);

  twist_widget = new TwistWidget(this);
  QDockWidget* dock = new QDockWidget("Twist", this);
  dock->setWidget(twist_widget);
  dock->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable);
  addDockWidget(Qt::BottomDockWidgetArea, dock);

  QTimer* timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &DashboardWindow::updateWidgets);
  timer->start(33);  // Update at ~30 FPS
}

void DashboardWindow::updateWidgets() {
  auto twist_msg = dashboard_node_->getLatestTwist();
  if (twist_msg) {
    twist_widget->updateTwist(twist_msg);
  }

  auto image_msg = dashboard_node_->getLatestImage();
  if (image_msg) {
    try {
      cv::Mat frame = cv_bridge::toCvShare(image_msg, "bgr8")->image;
      if (frame.empty()) {
        RCLCPP_WARN(dashboard_node_->get_logger(), "Received empty frame");
      } else {
        RCLCPP_INFO(dashboard_node_->get_logger(), "Processing frame: %dx%d", frame.cols, frame.rows);

        QMetaObject::invokeMethod(video_widget, [this, frame]() {
          try {
            video_widget->updateFrame(frame.clone());  // Use clone() to ensure deep copy
          } catch (const std::exception& e) {
            qDebug() << "Exception in updateFrame: " << e.what();
          }
        }, Qt::QueuedConnection);
      }
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(dashboard_node_->get_logger(), "CV bridge exception: %s", e.what());
    } catch (std::exception& e) {
      RCLCPP_ERROR(dashboard_node_->get_logger(), "Exception in updateWidgets: %s", e.what());
    }
  } else {
    RCLCPP_WARN_THROTTLE(dashboard_node_->get_logger(), *dashboard_node_->get_clock(), 1000, "No image received");
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DashboardNode>();
  QApplication app(argc, argv);
  DashboardWindow window(node);
  window.show();
  QTimer timer;
  QObject::connect(&timer, &QTimer::timeout, [&]() {
    rclcpp::spin_some(node);
  });
  timer.start(10);  // 10ms
  return app.exec();
}
