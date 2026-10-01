#pragma once

#include <memory>
#include <string>

#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QTimer>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <robot_interfaces/msg/mission_status.hpp>
#include <robot_interfaces/msg/navigation_status.hpp>
#include <robot_interfaces/srv/load_path.hpp>
#include <robot_interfaces/srv/mark_stop_point.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "robot_rviz/cmd_vel_widget.hpp"

namespace robot_rviz
{

class ControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ControlPanel(QWidget * parent = nullptr);
  ~ControlPanel() override;

  void onInitialize() override;

private Q_SLOTS:
  void onTriggerLocalization();
  void onToggleRecording();
  void onMarkStopPoint();
  void onBrowseWaypointsFile();
  void onLoadPath();
  void onToggleNavigation();
  void onTogglePause();
  void onSpinRos();

private:
  void setStatusLabel(QLabel * label, const std::string & text, const std::string & color);
  void navStatusCallback(const robot_interfaces::msg::NavigationStatus::SharedPtr msg);
  void missionStatusCallback(const robot_interfaces::msg::MissionStatus::SharedPtr msg);
  void localizationStatusCallback(const std_msgs::msg::String::SharedPtr msg);
  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  QTimer * spin_timer_;

  // Battery -- a small fixed-size bar plus a separate text label, instead of
  // one wide bar with the percentage crammed inside it.
  QProgressBar * battery_progress_bar_;
  QLabel * battery_label_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

  // Localization
  QPushButton * localization_button_;
  QLabel * localization_status_label_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr localization_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_status_sub_;

  // Path recording
  QPushButton * record_button_;
  QDoubleSpinBox * dwell_spinbox_;
  QPushButton * mark_stop_point_button_;
  QLabel * recording_status_label_;
  bool is_recording_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr record_client_;
  rclcpp::Client<robot_interfaces::srv::MarkStopPoint>::SharedPtr mark_stop_point_client_;

  // Path loading
  QLineEdit * waypoints_file_edit_;
  QPushButton * browse_button_;
  QPushButton * load_button_;
  QLabel * load_status_label_;
  rclcpp::Client<robot_interfaces::srv::LoadPath>::SharedPtr load_path_client_;

  // Navigation -- a plain on/off toggle (mirrors Path Recording's pattern), not an
  // action: the path is a dense, continuous trajectory, not a handful of discrete
  // goals, so there's no useful "progress %" or mid-route cancel semantics to expose.
  QPushButton * nav_button_;
  QPushButton * pause_button_;
  QLabel * nav_status_label_;
  QLabel * mission_label_;
  bool is_navigating_;
  bool is_paused_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr nav_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr pause_client_;
  rclcpp::Subscription<robot_interfaces::msg::NavigationStatus>::SharedPtr nav_status_sub_;
  rclcpp::Subscription<robot_interfaces::msg::MissionStatus>::SharedPtr mission_status_sub_;

  // Cmd vel visualization
  CmdVelWidget * cmd_vel_widget_;
  QLabel * cmd_vel_label_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

}  // namespace robot_rviz