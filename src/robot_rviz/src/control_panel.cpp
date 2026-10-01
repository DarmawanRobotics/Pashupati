#include "robot_rviz/control_panel.hpp"

#include <cmath>

#include <QFileDialog>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>

namespace robot_rviz
{

ControlPanel::ControlPanel(QWidget * parent)
: rviz_common::Panel(parent), is_recording_(false), is_navigating_(false), is_paused_(false)
{
  // Hard cap on the panel's own width -- this is what actually stops it
  // ballooning when a status label gets a long string (e.g. "loaded 90
  // waypoints from /home/robot/dev/.../example_waypoint.csv"). Word wrap on
  // a label only wraps text *within whatever width it's given*; without a
  // ceiling on the panel itself, an unconstrained label just asks for a
  // wider layout instead of wrapping.
  setMaximumWidth(260);

  // Compact global styling: this is what stops the panel from
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
  auto * battery_layout = new QHBoxLayout;
  battery_layout->setContentsMargins(4, 4, 4, 4);
  battery_layout->setSpacing(6);
  battery_progress_bar_ = new QProgressBar;
  battery_progress_bar_->setRange(0, 100);
  battery_progress_bar_->setTextVisible(false);
  battery_progress_bar_->setFixedSize(60, 12);
  battery_label_ = new QLabel("no data yet");
  battery_layout->addWidget(battery_progress_bar_);
  battery_layout->addWidget(battery_label_, 1);
  battery_group->setLayout(battery_layout);

  // --- Localization group ---
  auto * localization_group = new QGroupBox("Localization");
  auto * localization_layout = new QVBoxLayout;
  localization_layout->setContentsMargins(4, 4, 4, 4);
  localization_layout->setSpacing(2);
  localization_button_ = new QPushButton("Trigger Localization");
  localization_status_label_ = new QLabel("idle");
  localization_status_label_->setWordWrap(true);
  localization_status_label_->setFixedHeight(28);
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
  recording_status_label_->setWordWrap(true);
  recording_status_label_->setFixedHeight(28);
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
  load_status_label_->setWordWrap(true);
  load_status_label_->setFixedHeight(28);
  load_layout->addLayout(file_layout);
  load_layout->addWidget(load_button_);
  load_layout->addWidget(load_status_label_);
  load_group->setLayout(load_layout);

  // --- Navigation group ---
  auto * nav_group = new QGroupBox("Navigation");
  auto * nav_layout = new QVBoxLayout;
  nav_layout->setContentsMargins(4, 4, 4, 4);
  nav_layout->setSpacing(2);
  nav_button_ = new QPushButton("Start Navigation");
  pause_button_ = new QPushButton("Pause");
  pause_button_->setEnabled(false);
  auto * nav_buttons = new QHBoxLayout;
  nav_buttons->setSpacing(4);
  nav_buttons->addWidget(nav_button_);
  nav_buttons->addWidget(pause_button_);
  mission_label_ = new QLabel("no mission status");
  mission_label_->setWordWrap(true);
  nav_status_label_ = new QLabel("NAV_INACTIVE");
  nav_status_label_->setAutoFillBackground(true);
  nav_status_label_->setWordWrap(true);
  nav_status_label_->setFixedHeight(28);
  nav_layout->addLayout(nav_buttons);
  nav_layout->addWidget(nav_status_label_);
  nav_layout->addWidget(mission_label_);
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
  connect(nav_button_, &QPushButton::clicked, this, &ControlPanel::onToggleNavigation);
  connect(pause_button_, &QPushButton::clicked, this, &ControlPanel::onTogglePause);
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
  nav_client_ = node_->create_client<std_srvs::srv::SetBool>("navigation/start_nav");
  pause_client_ = node_->create_client<std_srvs::srv::SetBool>("navigation/pause");

  // Latched topics: match the publishers' transient-local durability to get the last value.
  auto latched = rclcpp::QoS(1).transient_local();
  localization_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "localization/status", latched,
    std::bind(&ControlPanel::localizationStatusCallback, this, std::placeholders::_1));
  mission_status_sub_ = node_->create_subscription<robot_interfaces::msg::MissionStatus>(
    "navigation/mission_status", 10,
    std::bind(&ControlPanel::missionStatusCallback, this, std::placeholders::_1));

  nav_status_sub_ = node_->create_subscription<robot_interfaces::msg::NavigationStatus>(
    "navigation/status", 10,
    std::bind(&ControlPanel::navStatusCallback, this, std::placeholders::_1));

  // Remap from the rviz2 command line, e.g. -r battery:=/l1w/battery -r cmd_vel:=/l1w/cmd_vel
  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    "battery", 10,
    std::bind(&ControlPanel::batteryCallback, this, std::placeholders::_1));

  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&ControlPanel::cmdVelCallback, this, std::placeholders::_1));

  // rviz's own event loop is Qt's, not rclcpp's spin() -- pump callbacks
  // (service responses, the status subscription) on a timer instead of
  // blocking the GUI thread with a real spin().
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

void ControlPanel::onToggleNavigation()
{
  if (!nav_client_->service_is_ready()) {
    setStatusLabel(nav_status_label_, "service not available", "#ffcc00");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = !is_navigating_;
  nav_client_->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      auto response = future.get();
      if (response->success) {
        is_navigating_ = !is_navigating_;
        nav_button_->setText(is_navigating_ ? "Stop Navigation" : "Start Navigation");
      }
      setStatusLabel(
        nav_status_label_, response->message,
        response->success ? "#88ff88" : "#ff8888");
    });
}

void ControlPanel::onTogglePause()
{
  if (!pause_client_->service_is_ready()) {
    setStatusLabel(nav_status_label_, "pause service not available", "#ffcc00");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = !is_paused_;
  pause_client_->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      if (future.get()->success) {
        is_paused_ = !is_paused_;
        pause_button_->setText(is_paused_ ? "Resume" : "Pause");
      }
    });
}

void ControlPanel::navStatusCallback(const robot_interfaces::msg::NavigationStatus::SharedPtr msg)
{
  const std::string & s = msg->state;
  std::string color = "#cccccc";
  if (s == "FOLLOWING") {
    color = "#88ff88";
  } else if (s == "GOAL_REACHED") {
    color = "#88bbff";
  } else if (s == "APPROACHING" || s == "DWELLING" || s == "PAUSED") {
    color = "#ffff88";
  } else if (s == "NO_PATH" || s == "BLOCKED") {
    color = "#ffaa55";
  } else if (s == "EMERGENCY_STOP" || s == "TF_UNAVAILABLE" || s == "AVOIDANCE_STALE" ||
    s == "OFF_PATH")
  {
    color = "#ff8888";
  }

  // Keep the buttons in sync when navigation is started/stopped from elsewhere.
  is_navigating_ = s != "NAV_INACTIVE";
  is_paused_ = s == "PAUSED";
  nav_button_->setText(is_navigating_ ? "Stop Navigation" : "Start Navigation");
  pause_button_->setEnabled(is_navigating_);
  pause_button_->setText(is_paused_ ? "Resume" : "Pause");

  setStatusLabel(nav_status_label_, s + ": " + msg->message, color);
}

void ControlPanel::missionStatusCallback(
  const robot_interfaces::msg::MissionStatus::SharedPtr msg)
{
  mission_label_->setText(QString::fromStdString(msg->active_behavior));
}

void ControlPanel::localizationStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
  const bool localized = msg->data != "none";
  setStatusLabel(
    localization_status_label_, localized ? "localized: " + msg->data : "not localized",
    localized ? "#88ff88" : "#ffcc00");
}

void ControlPanel::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  if (!std::isfinite(msg->percentage)) {
    battery_label_->setText("Battery: unknown");
    return;
  }
  int percent = static_cast<int>(msg->percentage * 100.0f);
  battery_progress_bar_->setValue(percent);
  battery_label_->setText(QString("Battery: %1%").arg(percent));

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

}  // namespace robot_rviz

PLUGINLIB_EXPORT_CLASS(robot_rviz::ControlPanel, rviz_common::Panel)