// dashboard_node.cpp
#include "dashboard_turtlebot3_cpp/dashboard_node.hpp"
#include <QApplication>

TwistWidget::TwistWidget(QWidget* parent) : QWidget(parent) {
  layout = new QVBoxLayout(this);
  linear_label = new QLabel("Linear: x=0.00, y=0.00, z=0.00", this);
  angular_label = new QLabel("Angular: x=0.00, y=0.00, z=0.00", this);
  linear_progress = new QProgressBar(this);
  linear_progress->setRange(-100, 100);  // Assuming max speed is 1 m/s
  linear_progress->setFormat("Linear X: %v");
  layout->addWidget(linear_label);
  layout->addWidget(angular_label);
  layout->addWidget(linear_progress);
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
  
  // Update progress bar
  int progress_value = static_cast<int>(msg->linear.x * 100);  // Scale to -100 to 100
  linear_progress->setValue(progress_value);
}

DashboardNode::DashboardNode() : Node("dashboard_turtlebot3") {
  twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&DashboardNode::twistCallback, this, std::placeholders::_1));
}

geometry_msgs::msg::Twist::SharedPtr DashboardNode::getLatestTwist() {
  return latest_twist_;
}

void DashboardNode::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  latest_twist_ = msg;
}

DashboardWindow::DashboardWindow(std::shared_ptr<DashboardNode> node, QWidget* parent)
  : QMainWindow(parent), dashboard_node_(node) {
  setWindowTitle("Turtlebot3 Dashboard");
  setGeometry(100, 100, 400, 300);

  twist_widget = new TwistWidget(this);

  QDockWidget* dock = new QDockWidget("Twist", this);
  dock->setWidget(twist_widget);
  dock->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable);
  addDockWidget(Qt::LeftDockWidgetArea, dock);

  QTimer* timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &DashboardWindow::updateWidgets);
  timer->start(100);  // Update every 100ms
}

void DashboardWindow::updateWidgets() {
  auto twist_msg = dashboard_node_->getLatestTwist();
  if (twist_msg) {
    twist_widget->updateTwist(twist_msg);
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
