#include "robot_ui/control_panel.hpp"

#include <QFileDialog>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>

namespace robot_ui
{

ControlPanel::ControlPanel(QWidget * parent)
: rviz_common::Panel(parent), is_recording_(false), mission_is_active_(true)
{
  auto * main_layout = new QVBoxLayout;

  // --- Battery group ---
  auto * battery_group = new QGroupBox("Battery");
  auto * battery_layout = new QVBoxLayout;
  battery_progress_bar_ = new QProgressBar;
  battery_progress_bar_->setRange(0, 100);
  battery_progress_bar_->setFormat("no data yet");
  battery_layout->addWidget(battery_progress_bar_);
  battery_group->setLayout(battery_layout);

  // --- Localization group ---
  auto * localization_group = new QGroupBox("Localization");
  auto * localization_layout = new QVBoxLayout;
  localization_button_ = new QPushButton("Trigger Localization");
  localization_status_label_ = new QLabel("idle");
  localization_layout->addWidget(localization_button_);
  localization_layout->addWidget(localization_status_label_);
  localization_group->setLayout(localization_layout);

  // --- Path recording group ---
  auto * recording_group = new QGroupBox("Path Recording");
  auto * recording_layout = new QVBoxLayout;
  record_button_ = new QPushButton("Start Recording");
  auto * dwell_layout = new QHBoxLayout;
  dwell_spinbox_ = new QDoubleSpinBox;
  dwell_spinbox_->setRange(0.0, 300.0);
  dwell_spinbox_->setValue(5.0);
  dwell_spinbox_->setSuffix(" s");
  mark_stop_point_button_ = new QPushButton("Mark Stop Point");
  dwell_layout->addWidget(new QLabel("Dwell:"));
  dwell_layout->addWidget(dwell_spinbox_);
  dwell_layout->addWidget(mark_stop_point_button_);
  recording_status_label_ = new QLabel("not recording");
  recording_layout->addWidget(record_button_);
  recording_layout->addLayout(dwell_layout);
  recording_layout->addWidget(recording_status_label_);
  recording_group->setLayout(recording_layout);

  // --- Path loading group ---
  auto * load_group = new QGroupBox("Path Loading");
  auto * load_layout = new QVBoxLayout;
  auto * file_layout = new QHBoxLayout;
  waypoints_file_edit_ = new QLineEdit(
    "/home/robot/dev/Pashupati/map/example/example_waypoint.csv");
  browse_button_ = new QPushButton("Browse...");
  file_layout->addWidget(waypoints_file_edit_);
  file_layout->addWidget(browse_button_);
  load_button_ = new QPushButton("Load Path");
  load_status_label_ = new QLabel("no path loaded yet");
  load_layout->addLayout(file_layout);
  load_layout->addWidget(load_button_);
  load_layout->addWidget(load_status_label_);
  load_group->setLayout(load_layout);

  // --- Mission group (behavior tree) ---
  auto * mission_group = new QGroupBox("Mission (Behavior Tree)");
  auto * mission_layout = new QVBoxLayout;
  mission_root_status_label_ = new QLabel("root: unknown");
  mission_active_behavior_label_ = new QLabel("active: none");
  mission_pause_button_ = new QPushButton("Pause Mission");
  mission_layout->addWidget(mission_root_status_label_);
  mission_layout->addWidget(mission_active_behavior_label_);
  mission_layout->addWidget(mission_pause_button_);
  mission_group->setLayout(mission_layout);

  // --- Navigation group ---
  auto * nav_group = new QGroupBox("Navigation");
  auto * nav_layout = new QVBoxLayout;
  auto * nav_button_layout = new QHBoxLayout;
  start_nav_button_ = new QPushButton("Start Navigation");
  start_nav_button_->setEnabled(false);  // mission_is_active_ starts true, matches mission_node's default
  cancel_nav_button_ = new QPushButton("Cancel");
  cancel_nav_button_->setEnabled(false);
  nav_button_layout->addWidget(start_nav_button_);
  nav_button_layout->addWidget(cancel_nav_button_);
  nav_progress_bar_ = new QProgressBar;
  nav_progress_bar_->setRange(0, 100);
  nav_status_label_ = new QLabel("NAV_INACTIVE");
  nav_status_label_->setAutoFillBackground(true);
  nav_layout->addLayout(nav_button_layout);
  nav_layout->addWidget(nav_progress_bar_);
  nav_layout->addWidget(nav_status_label_);
  nav_group->setLayout(nav_layout);

  main_layout->addWidget(battery_group);
  main_layout->addWidget(localization_group);
  main_layout->addWidget(recording_group);
  main_layout->addWidget(load_group);
  main_layout->addWidget(mission_group);
  main_layout->addWidget(nav_group);
  setLayout(main_layout);

  connect(localization_button_, &QPushButton::clicked, this, &ControlPanel::onTriggerLocalization);
  connect(record_button_, &QPushButton::clicked, this, &ControlPanel::onToggleRecording);
  connect(mark_stop_point_button_, &QPushButton::clicked, this, &ControlPanel::onMarkStopPoint);
  connect(browse_button_, &QPushButton::clicked, this, &ControlPanel::onBrowseWaypointsFile);
  connect(load_button_, &QPushButton::clicked, this, &ControlPanel::onLoadPath);
  connect(start_nav_button_, &QPushButton::clicked, this, &ControlPanel::onStartNavigation);
  connect(cancel_nav_button_, &QPushButton::clicked, this, &ControlPanel::onCancelNavigation);
  connect(mission_pause_button_, &QPushButton::clicked, this, &ControlPanel::onToggleMissionActive);
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

  mission_set_active_client_ = node_->create_client<std_srvs::srv::SetBool>("mission/set_active");
  mission_status_sub_ = node_->create_subscription<robot_interfaces::msg::MissionStatus>(
    "mission/status", 10,
    std::bind(&ControlPanel::missionStatusCallback, this, std::placeholders::_1));

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

void ControlPanel::onToggleMissionActive()
{
  if (!mission_set_active_client_->service_is_ready()) {
    setStatusLabel(mission_root_status_label_, "mission/set_active not available", "#ffcc00");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = !mission_is_active_;
  mission_set_active_client_->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      // mission_is_active_, the button text, and start_nav_button_'s enabled state are all
      // driven from missionStatusCallback (the actual mission/status topic), not from here --
      // that keeps one source of truth even if something else also toggled mission/set_active.
      auto response = future.get();
      if (!response->success) {
        setStatusLabel(mission_root_status_label_, "set_active failed: " + response->message, "#ff8888");
      }
    });
}

void ControlPanel::missionStatusCallback(const robot_interfaces::msg::MissionStatus::SharedPtr msg)
{
  mission_is_active_ = msg->active;
  mission_pause_button_->setText(mission_is_active_ ? "Pause Mission" : "Resume Mission");
  start_nav_button_->setEnabled(!mission_is_active_);

  std::string color = "#cccccc";
  if (msg->root_status == "RUNNING") {
    color = "#88ff88";
  } else if (msg->root_status == "SUCCESS") {
    color = "#88bbff";
  } else if (msg->root_status == "FAILURE") {
    color = "#ff8888";
  }
  std::string suffix = msg->active ? "" : " (paused)";
  setStatusLabel(mission_root_status_label_, "root: " + msg->root_status + suffix, color);
  mission_active_behavior_label_->setText(
    QString::fromStdString("active: " + msg->active_behavior));
}

}  // namespace robot_ui

PLUGINLIB_EXPORT_CLASS(robot_ui::ControlPanel, rviz_common::Panel)
