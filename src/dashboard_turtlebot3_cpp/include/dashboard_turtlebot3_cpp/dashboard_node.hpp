#ifndef DASHBOARD_NODE_HPP
#define DASHBOARD_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QLabel>
#include <QDockWidget>
#include <QTimer>
#include <QProgressBar>

class TwistWidget : public QWidget {
  Q_OBJECT

public:
  TwistWidget(QWidget* parent = nullptr);
  void updateTwist(const geometry_msgs::msg::Twist::SharedPtr msg);

private:
  QVBoxLayout* layout;
  QLabel* linear_label;
  QLabel* angular_label;
  QProgressBar* linear_progress;
};

class DashboardNode : public rclcpp::Node {
public:
  DashboardNode();
  geometry_msgs::msg::Twist::SharedPtr getLatestTwist();

private:
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;
  geometry_msgs::msg::Twist::SharedPtr latest_twist_;
};

class DashboardWindow : public QMainWindow {
  Q_OBJECT

public:
  DashboardWindow(std::shared_ptr<DashboardNode> node, QWidget* parent = nullptr);

private slots:
  void updateWidgets();

private:
  std::shared_ptr<DashboardNode> dashboard_node_;
  TwistWidget* twist_widget;
};

#endif // DASHBOARD_NODE_HPP
