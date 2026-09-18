#include "robot_ui/control_panel.hpp"

#include <QFileDialog>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>

namespace robot_ui
{

ControlPanel::ControlPanel(QWidget * parent)
: rviz_common::Panel(parent), is_recording_(false)
{
  // --- Compact global styling: this is what stops the panel from
  // rendering huge on first load (default Qt margins/paddings + normal
  // font size add up fast across 5 group boxes). ---
  setStyleSheet(
    "QWidget { font-size: 9pt; }"
    "QGroupBox { font-weight: 600; border: 1px solid #3a3a3a; border-radius: 4px;"
    "  margin-top: 7px; padding-top: 4px; }"
    "QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px; }"
    "QPushButton { padding: 2px 8px; min-height: 18px; }"
    "QLabel { padding: 0px; }"
    "QLineEdit, QDoubleSpinBox { padding: 1px 3px; min-height: 18px; }");

  auto * main_layout = new QVBoxLayout;
  main_layout->setContentsMargins(4, 4, 4, 4);
  main_layout->setSpacing(4);

  // --- Battery group ---
  auto * battery_group = new QGroupBox("Battery");
  auto * battery_layout = new QVBoxLayout;
  battery_layout->setContentsMargins(4, 4, 4, 4);
  battery_layout->setSpacing(2);
  battery_progress_bar_ = new QProgressBar;
  battery_progress_bar_->setRange(0, 100);
  battery_progress_bar_->setFormat("no data yet");
  battery_progress_bar_->setFixedHeight(14);
  battery_layout->addWidget(battery_progress_bar_);
  battery_group->setLayout(battery_layout);

  // --- Localization group ---
  auto * localization_group = new QGroupBox("Localization");
  auto * localization_layout = new QVBoxLayout;
  localization_layout->setContentsMargins(4, 4, 4, 4);
  localization_layout->setSpacing(2);
  localization_button_ = new QPushButton("Trigger Localization");
  localization_status_label_ = new QLabel("idle");
  localization_status_label_->setFixedHeight(16);
  localization_layout->addWidget(localization_button_);
  localization_layout->addWidget(localization_status_label_);
  localization_group->setLayout(localization_layout);

  // --- Path recording group ---
  auto * recording_group = new QGroupBox("Path Recording");
  auto * recording_layout = new QVBoxLayout;
  recording_layout->setContentsMargins(4, 4, 4, 4);
  recording_layout->setSpacing(2);
  record_button_ = new QPushButton("Start Recording");
  auto * dwell_layout = new QHBoxLayout;
  dwell_layout->setSpacing(4);
  dwell_spinbox_ = new QDoubleSpinBox;
  dwell_spinbox_->setRange(0.0, 300.0);
  dwell_spinbox_->setValue(5.0);
  dwell_spinbox_->setSuffix(" s");
  dwell_spinbox_->setFixedWidth(70);
  mark_stop_point_button_ = new QPushButton("Mark Stop Point");
  dwell_layout->addWidget(new QLabel("Dwell:"));
  dwell_layout->addWidget(dwell_spinbox_);
  dwell_layout->addWidget(mark_stop_point_button_);
  recording_status_label_ = new QLabel("not recording");
  recording_status_label_->setFixedHeight(16);
  recording_layout->addWidget(record_button_);
  recording_layout->addLayout(dwell_layout);
  recording_layout->addWidget(recording_status_label_);
  recording_group->setLayout(recording_layout);

  // --- Path loading group ---
  auto * load_group = new QGroupBox("Path Loading");
  auto * load_layout = new QVBoxLayout;
  load_layout->setContentsMargins(4, 4, 4, 4);
  load_layout->setSpacing(2);
  auto * file_layout = new QHBoxLayout;
  file_layout->setSpacing(4);
  waypoints_file_edit_ = new QLineEdit(
    "/home/robot/dev/Pashupati/map/example/example_waypoint.csv");
  browse_button_ = new QPushButton("Browse...");
  file_layout->addWidget(waypoints_file_edit_);
  file_layout->addWidget(browse_button_);
  load_button_ = new QPushButton("Load Path");
  load_status_label_ = new QLabel("no path loaded yet");
  load_status_label_->setFixedHeight(16);
  load_layout->addLayout(file_layout);
  load_layout->addWidget(load_button_);
  load_layout->addWidget(load_status_label_);
  load_group->setLayout(load_layout);

  // --- Navigation group ---
  auto * nav_group = new QGroupBox("Navigation");
  auto * nav_layout = new QVBoxLayout;
  nav_layout->setContentsMargins(4, 4, 4, 4);
  nav_layout->setSpacing(2);
  auto * nav_button_layout = new QHBoxLayout;
  nav_button_layout->setSpacing(4);
  start_nav_button_ = new QPushButton("Start Navigation");
  cancel_nav_button_ = new QPushButton("Cancel");
  cancel_nav_button_->setEnabled(false);
  nav_button_layout->addWidget(start_nav_button_);
  nav_button_layout->addWidget(cancel_nav_button_);
  nav_progress_bar_ = new QProgressBar;
  nav_progress_bar_->setRange(0, 100);
  nav_progress_bar_->setFixedHeight(14);
  nav_status_label_ = new QLabel("NAV_INACTIVE");
  nav_status_label_->setAutoFillBackground(true);
  nav_status_label_->setFixedHeight(16);
  nav_layout->addLayout(nav_button_layout);
  nav_layout->addWidget(nav_progress_bar_);
  nav_layout->addWidget(nav_status_label_);
  nav_group->setLayout(nav_layout);

  // --- Cmd Vel group ---
  auto * cmd_vel_group = new QGroupBox("Cmd Vel");
  auto * cmd_vel_layout = new QVBoxLayout;
  cmd_vel_layout->setContentsMargins(4, 4, 4, 4);
  cmd_vel_layout->setSpacing(2);
  auto * cmd_vel_row = new QHBoxLayout;
  cmd_vel_row->setSpacing(6);
  cmd_vel_widget_ = new CmdVelWidget;
  cmd_vel_label_ = new QLabel("vx: 0.00  vy: 0.00  wz: 0.00");
  cmd_vel_label_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  cmd_vel_label_->setWordWrap(true);
  cmd_vel_row->addWidget(cmd_vel_widget_);
  cmd_vel_row->addWidget(cmd_vel_label_, 1);
  cmd_vel_layout->addLayout(cmd_vel_row);
  cmd_vel_group->setLayout(cmd_vel_layout);

  main_layout->addWidget(battery_group);
  main_layout->addWidget(localization_group);
  main_layout->addWidget(recording_group);
  main_layout->addWidget(load_group);
  main_layout->addWidget(nav_group);
  main_layout->addWidget(cmd_vel_group);
  setLayout(main_layout);

  connect(localization_button_, &QPushButton::clicked, this, &ControlPanel::onTriggerLocalization);
  connect(record_button_, &QPushButton::clicked, this, &ControlPanel::onToggleRecording);
  connect(mark_stop_point_button_, &QPushButton::clicked, this, &ControlPanel::onMarkStopPoint);
  connect(browse_button_, &QPushButton::clicked, this, &ControlPanel::onBrowseWaypointsFile);
  connect(load_button_, &QPushButton::clicked, this, &ControlPanel::onLoadPath);
  connect(start_nav_button_, &QPushButton::clicked, this, &ControlPanel::onStartNavigation);
  connect(cancel_nav_button_, &QPushButton::clicked, this, &ControlPanel::onCancelNavigation);
}

ControlPanel::~ControlPanel() = default;

void ControlPanel::onInitialize()
{
  node_ = std::make_shared<rclcpp::Node>("rviz_control_panel");

  localization_client_ = node_->create_client<std_srvs::srv::Trigger>("localization/start");
  record_client_ = node_->create_client<std_srvs::srv::SetBool>("mapping/path_record");
  mark_stop_point_client_ =
    node_->create_client<robot_interfaces::srv::MarkStopPoint>("mapping/mark_stop_point");
  load_path_client_ = node_->create_client<robot_interfaces::srv::LoadPath>("navigation/load_path");
  navigate_client_ = rclcpp_action::create_client<NavigateRoute>(node_, "navigate_route");

  nav_status_sub_ = node_->create_subscription<robot_interfaces::msg::NavigationStatus>(
    "navigation/status", 10,
    std::bind(&ControlPanel::navStatusCallback, this, std::placeholders::_1));

  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    "drivers/battery", 10,
    std::bind(&ControlPanel::batteryCallback, this, std::placeholders::_1));

  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&ControlPanel::cmdVelCallback, this, std::placeholders::_1));

  // rviz's own event loop is Qt's, not rclcpp's spin() -- pump callbacks
  // (service responses, action feedback/result, the status subscription)
  // on a timer instead of blocking the GUI thread with a real spin().
  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, this, &ControlPanel::onSpinRos);
  spin_timer_->start(50);
}

void ControlPanel::onSpinRos()
{
  rclcpp::spin_some(node_);
}

void ControlPanel::setStatusLabel(QLabel * label, const std::string & text, const std::string & color)
{
  label->setText(QString::fromStdString(text));
  label->setStyleSheet(QString::fromStdString("background-color: " + color + "; padding: 2px;"));
}

void ControlPanel::onTriggerLocalization()
{
  if (!localization_client_->service_is_ready()) {
    setStatusLabel(localization_status_label_, "service not available", "#ffcc00");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  setStatusLabel(localization_status_label_, "calling...", "#ffffff");
  localization_client_->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      auto response = future.get();
      setStatusLabel(
        localization_status_label_, response->message,
        response->success ? "#88ff88" : "#ff8888");
    });
}

void ControlPanel::onToggleRecording()
{
  if (!record_client_->service_is_ready()) {
    setStatusLabel(recording_status_label_, "service not available", "#ffcc00");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = !is_recording_;
  record_client_->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      auto response = future.get();
      if (response->success) {
        is_recording_ = !is_recording_;
        record_button_->setText(is_recording_ ? "Stop Recording" : "Start Recording");
      }
      setStatusLabel(
        recording_status_label_, response->message,
        response->success ? "#88ff88" : "#ff8888");
    });
}

void ControlPanel::onMarkStopPoint()
{
  if (!mark_stop_point_client_->service_is_ready()) {
    setStatusLabel(recording_status_label_, "service not available", "#ffcc00");
    return;
  }
  auto request = std::make_shared<robot_interfaces::srv::MarkStopPoint::Request>();
  request->dwell_sec = static_cast<float>(dwell_spinbox_->value());
  mark_stop_point_client_->async_send_request(
    request,
    [this](rclcpp::Client<robot_interfaces::srv::MarkStopPoint>::SharedFuture future) {
      auto response = future.get();
      setStatusLabel(
        recording_status_label_, response->message,
        response->success ? "#88ff88" : "#ff8888");
    });
}

void ControlPanel::onBrowseWaypointsFile()
{
  QString filename = QFileDialog::getOpenFileName(
    this, "Select waypoints CSV", waypoints_file_edit_->text(), "CSV files (*.csv)");
  if (!filename.isEmpty()) {
    waypoints_file_edit_->setText(filename);
  }
}

void ControlPanel::onLoadPath()
{
  if (!load_path_client_->service_is_ready()) {
    setStatusLabel(load_status_label_, "service not available", "#ffcc00");
    return;
  }
  auto request = std::make_shared<robot_interfaces::srv::LoadPath::Request>();
  request->waypoints_file = waypoints_file_edit_->text().toStdString();
  setStatusLabel(load_status_label_, "loading...", "#ffffff");
  load_path_client_->async_send_request(
    request,
    [this](rclcpp::Client<robot_interfaces::srv::LoadPath>::SharedFuture future) {
      auto response = future.get();
      setStatusLabel(
        load_status_label_, response->message,
        response->success ? "#88ff88" : "#ff8888");
    });
}

void ControlPanel::onStartNavigation()
{
  if (!navigate_client_->wait_for_action_server(std::chrono::seconds(0))) {
    setStatusLabel(nav_status_label_, "action server not available", "#ffcc00");
    return;
  }

  auto goal_msg = NavigateRoute::Goal();
  auto send_goal_options = rclcpp_action::Client<NavigateRoute>::SendGoalOptions();
  send_goal_options.feedback_callback = std::bind(
    &ControlPanel::navFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&ControlPanel::navResultCallback, this, std::placeholders::_1);
  send_goal_options.goal_response_callback =
    [this](GoalHandleNavigateRoute::SharedPtr goal_handle) {
      if (!goal_handle) {
        setStatusLabel(nav_status_label_, "goal rejected", "#ff8888");
      } else {
        current_goal_handle_ = goal_handle;
        start_nav_button_->setEnabled(false);
        cancel_nav_button_->setEnabled(true);
      }
    };

  navigate_client_->async_send_goal(goal_msg, send_goal_options);
}

void ControlPanel::onCancelNavigation()
{
  if (current_goal_handle_) {
    navigate_client_->async_cancel_goal(current_goal_handle_);
  }
}

void ControlPanel::navFeedbackCallback(
  GoalHandleNavigateRoute::SharedPtr,
  const std::shared_ptr<const NavigateRoute::Feedback> feedback)
{
  nav_progress_bar_->setValue(static_cast<int>(feedback->progress * 100.0f));
}

void ControlPanel::navResultCallback(const GoalHandleNavigateRoute::WrappedResult & result)
{
  start_nav_button_->setEnabled(true);
  cancel_nav_button_->setEnabled(false);
  current_goal_handle_.reset();

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      setStatusLabel(nav_status_label_, "SUCCEEDED: " + result.result->message, "#88ff88");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      setStatusLabel(nav_status_label_, "ABORTED: " + result.result->message, "#ff8888");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      setStatusLabel(nav_status_label_, "CANCELED", "#ffcc00");
      break;
    default:
      setStatusLabel(nav_status_label_, "UNKNOWN RESULT", "#ffcc00");
      break;
  }
}

void ControlPanel::navStatusCallback(const robot_interfaces::msg::NavigationStatus::SharedPtr msg)
{
  std::string color = "#cccccc";
  if (msg->state == "FOLLOWING") {
    color = "#88ff88";
  } else if (msg->state == "GOAL_REACHED") {
    color = "#88bbff";
  } else if (msg->state == "EMERGENCY_STOP" || msg->state == "TF_UNAVAILABLE") {
    color = "#ff8888";
  } else if (msg->state == "NO_PATH") {
    color = "#ffaa55";
  } else if (msg->state == "ALIGNING" || msg->state == "DWELLING") {
    color = "#ffff88";
  }

  setStatusLabel(nav_status_label_, msg->state + ": " + msg->message, color);
}

void ControlPanel::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  int percent = static_cast<int>(msg->percentage * 100.0f);
  battery_progress_bar_->setValue(percent);
  battery_progress_bar_->setFormat(QString("Battery: %1%").arg(percent));

  // Matches robot_driver_node's own low-battery gate (0.20) for auto mode.
  std::string color = "#88ff88";
  if (msg->percentage < 0.20f) {
    color = "#ff4444";
  } else if (msg->percentage < 0.40f) {
    color = "#ffaa00";
  }
  battery_progress_bar_->setStyleSheet(
    QString::fromStdString("QProgressBar::chunk { background-color: " + color + "; }"));
}

void ControlPanel::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_vel_widget_->setValues(msg->linear.x, msg->linear.y, msg->angular.z);
  cmd_vel_label_->setText(
    QString("vx: %1 m/s\nvy: %2 m/s\nwz: %3 rad/s")
      .arg(msg->linear.x, 0, 'f', 2)
      .arg(msg->linear.y, 0, 'f', 2)
      .arg(msg->angular.z, 0, 'f', 2));
}

}  // namespace robot_ui

PLUGINLIB_EXPORT_CLASS(robot_ui::ControlPanel, rviz_common::Panel)