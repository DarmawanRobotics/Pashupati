#ifndef ROBOT_UI__CONTROL_PANEL_HPP_
#define ROBOT_UI__CONTROL_PANEL_HPP_

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
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/panel.hpp>

#include <robot_interfaces/action/navigate_route.hpp>
#include <robot_interfaces/msg/mission_status.hpp>
#include <robot_interfaces/msg/navigation_status.hpp>
#include <robot_interfaces/srv/load_path.hpp>
#include <robot_interfaces/srv/mark_stop_point.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace robot_ui
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
  void onStartNavigation();
  void onCancelNavigation();
  void onToggleMissionActive();
  void onSpinRos();

private:
  using NavigateRoute = robot_interfaces::action::NavigateRoute;
  using GoalHandleNavigateRoute = rclcpp_action::ClientGoalHandle<NavigateRoute>;

  void setStatusLabel(QLabel * label, const std::string & text, const std::string & color);
  void navFeedbackCallback(
    GoalHandleNavigateRoute::SharedPtr,
    const std::shared_ptr<const NavigateRoute::Feedback> feedback);
  void navResultCallback(const GoalHandleNavigateRoute::WrappedResult & result);
  void navStatusCallback(const robot_interfaces::msg::NavigationStatus::SharedPtr msg);
  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void missionStatusCallback(const robot_interfaces::msg::MissionStatus::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  QTimer * spin_timer_;

  // Battery
  QProgressBar * battery_progress_bar_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

  // Localization
  QPushButton * localization_button_;
  QLabel * localization_status_label_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr localization_client_;

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

  // Navigation
  QPushButton * start_nav_button_;
  QPushButton * cancel_nav_button_;
  QProgressBar * nav_progress_bar_;
  QLabel * nav_status_label_;
  rclcpp_action::Client<NavigateRoute>::SharedPtr navigate_client_;
  GoalHandleNavigateRoute::SharedPtr current_goal_handle_;
  rclcpp::Subscription<robot_interfaces::msg::NavigationStatus>::SharedPtr nav_status_sub_;

  // Mission (behavior tree)
  QLabel * mission_root_status_label_;
  QLabel * mission_active_behavior_label_;
  QPushButton * mission_pause_button_;
  bool mission_is_active_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr mission_set_active_client_;
  rclcpp::Subscription<robot_interfaces::msg::MissionStatus>::SharedPtr mission_status_sub_;
};

}  // namespace robot_ui

#endif  // ROBOT_UI__CONTROL_PANEL_HPP_
