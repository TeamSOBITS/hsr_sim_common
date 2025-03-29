#include "hsr_sim_common/hsr_sim_wheel_action_server.hpp"

namespace hsr_sim{

WheelActionServer::WheelActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("wheel_action_server", options)
{
  // Configure the QoS profile
  rclcpp::QoS qos_profile(1); // depth = 1
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);


  this->action_server_move_wheel_linear_ = rclcpp_action::create_server<MoveWheelLinear>(
      this,
      "move_wheel_linear",
      std::bind(&WheelActionServer::handle_move_wheel_linear_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&WheelActionServer::handle_move_wheel_linear_cancel, this, std::placeholders::_1),
      std::bind(&WheelActionServer::handle_move_wheel_linear_accepted, this, std::placeholders::_1));
  this->action_server_move_wheel_rotate_ = rclcpp_action::create_server<MoveWheelRotate>(
      this,
      "move_wheel_rotate",
      std::bind(&WheelActionServer::handle_move_wheel_rotate_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&WheelActionServer::handle_move_wheel_rotate_cancel, this, std::placeholders::_1),
      std::bind(&WheelActionServer::handle_move_wheel_rotate_accepted, this, std::placeholders::_1));


  this->pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
      // "diff_controller/cmd_vel", qos_profile);
      "/hsrb/command_velocity", qos_profile);
  // this->sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
  //     // "odom", qos_profile, std::bind(&WheelActionServer::odom_callback, this, std::placeholders::_1));
  //     "odometry/odometry", qos_profile, std::bind(&WheelActionServer::odom_callback, this, std::placeholders::_1));


  RCLCPP_INFO(this->get_logger(), "WheelActionServer has been initialized.");
}
WheelActionServer::~WheelActionServer()
{
  this->action_server_move_wheel_linear_.reset();
  this->action_server_move_wheel_rotate_.reset();

  this->pub_cmd_vel_.reset();
  // this->sub_odom_.reset();

  RCLCPP_INFO(this->get_logger(), "WheelActionServer has been terminated.");
}


rclcpp_action::GoalResponse WheelActionServer::handle_move_wheel_linear_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MoveWheelLinear::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::GoalResponse WheelActionServer::handle_move_wheel_rotate_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MoveWheelRotate::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse WheelActionServer::handle_move_wheel_linear_cancel(
  const std::shared_ptr<GoalHandleMoveWheelLinear> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}
rclcpp_action::CancelResponse WheelActionServer::handle_move_wheel_rotate_cancel(
  const std::shared_ptr<GoalHandleMoveWheelRotate> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}


void WheelActionServer::handle_move_wheel_linear_accepted(
  const std::shared_ptr<GoalHandleMoveWheelLinear> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)goal_handle;
  std::thread{std::bind(&WheelActionServer::exe_move_wheel_linear, this, std::placeholders::_1), goal_handle}.detach();
}
void WheelActionServer::handle_move_wheel_rotate_accepted(
  const std::shared_ptr<GoalHandleMoveWheelRotate> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)goal_handle;
  std::thread{std::bind(&WheelActionServer::exe_move_wheel_rotate, this, std::placeholders::_1), goal_handle}.detach();
}


void WheelActionServer::exe_move_wheel_linear(
  const std::shared_ptr<GoalHandleMoveWheelLinear> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveWheelLinear::Result>();

  double time_allowance = goal->time_allowance.sec + goal->time_allowance.nanosec / 1e9;
  double goal_dist = std::sqrt(std::pow(goal->target_point.x, 2) + std::pow(goal->target_point.y, 2));
  double curt_dist_x = 0.0;
  double curt_dist_y = 0.0;
  double elapsed_time = 0.0;
  double elapsed_time_last = elapsed_time;
  double vel_max = 0.3; // TODO parameter file from...
  double accel, inflection_time;
  if ((4.*goal_dist/time_allowance) > vel_max) {
    accel = std::pow(vel_max, 2.) / (vel_max*time_allowance - goal_dist);
    inflection_time = time_allowance - (goal_dist/vel_max);
  } else {
    vel_max = (4*goal_dist) / time_allowance;
    accel = (8.*goal_dist) / std::pow(time_allowance, 2);
    inflection_time = time_allowance / 2.;
  }

  // // Set current time
  auto start_time = this->now();

  while (elapsed_time < time_allowance) {
    // Check if the goal has been canceled
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Goal has been canceled");
      this->pub_cmd_vel_->publish(zero_vel_);

      result->success = false;
      result->message = "[FAIL] Goal has been canceled";
      result->total_elapsed_time.sec = (this->now() - start_time).seconds();
      result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

      goal_handle->canceled(result);
      return;
    }

    // Calculate the elapsed time
    rclcpp::Duration dur_elapsed_time = this->now() - start_time;
    elapsed_time = dur_elapsed_time.nanoseconds() / 1e9; 

    double sum_xy_vel;

    if      (elapsed_time <= inflection_time)                   sum_xy_vel = accel * elapsed_time;
    else if ((time_allowance - inflection_time) < elapsed_time) sum_xy_vel = vel_max - accel * (elapsed_time - time_allowance + inflection_time);
    else                                                        sum_xy_vel = vel_max;

    geometry_msgs::msg::Twist out_vel;
    out_vel.linear.x = sum_xy_vel * std::cos(std::atan2(goal->target_point.y, goal->target_point.x));
    out_vel.linear.y = sum_xy_vel * std::sin(std::atan2(goal->target_point.y, goal->target_point.x));
    this->pub_cmd_vel_->publish(out_vel);

    // Update the previous error
    curt_dist_x += out_vel.linear.x * (elapsed_time - elapsed_time_last);
    curt_dist_y += out_vel.linear.x * (elapsed_time - elapsed_time_last);

    // Publish feedback
    auto feedback = std::make_shared<MoveWheelLinear::Feedback>();
    feedback->current_point.x = curt_dist_x;
    feedback->current_point.y = curt_dist_y;
    feedback->move_time.sec = (this->now() - start_time).seconds();
    feedback->move_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);
    goal_handle->publish_feedback(feedback);

    elapsed_time_last = elapsed_time;
  }

  this->pub_cmd_vel_->publish(zero_vel_);

  // Publish the result
  result->success = true;
  result->message = "Goal has been succeeded";
  result->total_elapsed_time.sec = (this->now() - start_time).seconds();
  result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

  goal_handle->succeed(result);
}


void WheelActionServer::exe_move_wheel_rotate(
  const std::shared_ptr<GoalHandleMoveWheelRotate> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveWheelRotate::Result>();

  double time_allowance = goal->time_allowance.sec + goal->time_allowance.nanosec / 1e9;
  double goal_rotate = std::abs(goal->target_yaw);
  double curt_rotate_yaw = 0.0;
  double elapsed_time = 0.0;
  double elapsed_time_last = elapsed_time;
  double vel_max = 1.0; // TODO parameter file from...
  double accel, inflection_time;
  if ((4.*goal_rotate/time_allowance) > vel_max) {
    accel = std::pow(vel_max, 2.) / (vel_max*time_allowance - goal_rotate);
    inflection_time = time_allowance - (goal_rotate/vel_max);
  } else {
    vel_max = (4*goal_rotate) / time_allowance;
    accel = (8.*goal_rotate) / std::pow(time_allowance, 2);
    inflection_time = time_allowance / 2.;
  }

  // // Set current time
  auto start_time = this->now();

  while (elapsed_time < time_allowance) {
    // Check if the goal has been canceled
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Goal has been canceled");
      this->pub_cmd_vel_->publish(zero_vel_);

      result->success = false;
      result->message = "[FAIL] Goal has been canceled";
      result->total_elapsed_time.sec = (this->now() - start_time).seconds();
      result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

      goal_handle->canceled(result);
      return;
    }

    // Calculate the elapsed time
    rclcpp::Duration dur_elapsed_time = this->now() - start_time;
    elapsed_time = dur_elapsed_time.nanoseconds() / 1e9; 

    double rotate_vel;

    if      (elapsed_time <= inflection_time)                   rotate_vel = accel * elapsed_time;
    else if ((time_allowance - inflection_time) < elapsed_time) rotate_vel = vel_max - accel * (elapsed_time - time_allowance + inflection_time);
    else                                                        rotate_vel = vel_max;

    geometry_msgs::msg::Twist out_vel;
    out_vel.angular.z = (0 < goal->target_yaw) ? rotate_vel : -rotate_vel;
    this->pub_cmd_vel_->publish(out_vel);

    // Update the previous error
    curt_rotate_yaw += out_vel.angular.z * (elapsed_time - elapsed_time_last);

    // Publish feedback
    auto feedback = std::make_shared<MoveWheelRotate::Feedback>();
    feedback->current_yaw = curt_rotate_yaw;
    feedback->move_time.sec = (this->now() - start_time).seconds();
    feedback->move_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);
    goal_handle->publish_feedback(feedback);

    elapsed_time_last = elapsed_time;
  }

  this->pub_cmd_vel_->publish(zero_vel_);

  // Publish the result
  result->success = true;
  result->message = "Goal has been succeeded";
  result->total_elapsed_time.sec = (this->now() - start_time).seconds();
  result->total_elapsed_time.nanosec = (this->now() - start_time).nanoseconds() % int(10E9);

  goal_handle->succeed(result);
}


// void WheelActionServer::odom_callback(
//   const nav_msgs::msg::Odometry::SharedPtr msg)
// {
//   RCLCPP_INFO(this->get_logger(), "Received odometry");

//   this->curt_odom_ = *msg;
// }

} // namespace hsr_sim