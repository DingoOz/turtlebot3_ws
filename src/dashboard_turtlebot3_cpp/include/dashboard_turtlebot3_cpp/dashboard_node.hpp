#ifndef DASHBOARD_NODE_HPP
#define DASHBOARD_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QLabel>
#include <QDockWidget>
#include <QTimer>
#include <QProgressBar>
#include <QImage>
#include <QPixmap>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class VideoWidget : public QLabel {
  Q_OBJECT

public:
  explicit VideoWidget(QWidget* parent = nullptr);
  virtual ~VideoWidget() = default;
  void updateFrame(const cv::Mat& frame);

private:
  QImage mat_to_qimage(const cv::Mat& mat);
};

class TwistWidget : public QWidget {
  Q_OBJECT

public:
  explicit TwistWidget(QWidget* parent = nullptr);
  virtual ~TwistWidget() = default;
  void updateTwist(const geometry_msgs::msg::Twist::SharedPtr msg);

private:
  QVBoxLayout* layout;
  QLabel* linear_label;
  QLabel* angular_label;
  QProgressBar* linear_progress;
  QProgressBar* angular_progress;
};

class DashboardNode : public rclcpp::Node {
public:
  DashboardNode();
  geometry_msgs::msg::Twist::SharedPtr getLatestTwist();
  sensor_msgs::msg::Image::SharedPtr getLatestImage();

private:
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  geometry_msgs::msg::Twist::SharedPtr latest_twist_;
  sensor_msgs::msg::Image::SharedPtr latest_image_;
};

class DashboardWindow : public QMainWindow {
  Q_OBJECT

public:
  DashboardWindow(std::shared_ptr<DashboardNode> node, QWidget* parent = nullptr);
  virtual ~DashboardWindow() = default;

private slots:
  void updateWidgets();

private:
  std::shared_ptr<DashboardNode> dashboard_node_;
  TwistWidget* twist_widget;
  VideoWidget* video_widget;
};

#endif // DASHBOARD_NODE_HPP
