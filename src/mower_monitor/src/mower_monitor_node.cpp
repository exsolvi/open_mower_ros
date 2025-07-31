#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <mower_msgs/Status.h>
#include <ros/ros.h>

#include <cmath>  // For std::abs
#include <deque>
#include <numeric>  // For std::accumulate

// Define a constant for the default max RPM
const float DEFAULT_MAX_RPM = 4200.0f;

class MowerMonitorNode {
 public:
  MowerMonitorNode() : nh_("~") {
    status_sub_ = nh_.subscribe("/ll/mower_status", 10, &MowerMonitorNode::statusCallback, this);
    log_timer_ = nh_.createTimer(ros::Duration(1.0), &MowerMonitorNode::logTimerCallback,
                                 this);  // Log every 1 second as requested initially
    window_duration_ = ros::Duration(1.0);
    dr_pub_ = nh_.advertise<dynamic_reconfigure::Config>("/move_base_flex/FTCPlanner/parameter_updates", 1);
  }

 private:
  ros::NodeHandle nh_;
  ros::Subscriber status_sub_;
  ros::Timer log_timer_;
  ros::Duration window_duration_;
  ros::Publisher dr_pub_;  // Publisher for dynamic reconfigure

  // Deque to store RPM values with their timestamps for moving average calculation
  std::deque<std::pair<ros::Time, float>> rpm_buffer_;
  float max_rpm_ = DEFAULT_MAX_RPM;  // Stores the maximum observed RPM, initialized with constant
  // Tracks if the mower motor is enabled. Using `prev_mow_enabled_` to detect transitions.
  bool mow_enabled_ = false;
  bool prev_mow_enabled_ = false;

  void statusCallback(const mower_msgs::Status::ConstPtr& msg) {
    // Store previous state and update current state
    prev_mow_enabled_ = mow_enabled_;
    mow_enabled_ = msg->mow_enabled;

    // If mow_enabled_ transitions from true to false, reset max_rpm_ and clear buffer
    if (prev_mow_enabled_ && !mow_enabled_) {
      max_rpm_ = DEFAULT_MAX_RPM;
      rpm_buffer_.clear();  // Clear buffer when motor is disabled
      ROS_INFO("MowerMonitor: Mower motor disabled. Resetting max RPM and clearing buffer.");
    }

    // Get the mowing motor RPM and take its absolute value
    float current_rpm = std::abs(msg->mower_motor_rpm);
    ros::Time current_time = ros::Time::now();

    // Update maximum RPM observed only if motor is enabled
    if (mow_enabled_ && current_rpm > max_rpm_) {
      max_rpm_ = current_rpm;
    }

    // Add current RPM and timestamp to the buffer only if motor is enabled
    if (mow_enabled_) {
      rpm_buffer_.push_back(std::make_pair(current_time, current_rpm));
    }

    // Remove old values from the buffer that are outside the window
    while (!rpm_buffer_.empty() && (current_time - rpm_buffer_.front().first) > window_duration_) {
      rpm_buffer_.pop_front();
    }
  }

  void logTimerCallback(const ros::TimerEvent& event) {
    if (!mow_enabled_) {
      // Optionally send a max load factor of 1.0 when not mowing
      dynamic_reconfigure::Config config_msg;
      dynamic_reconfigure::DoubleParameter double_param;
      double_param.name = "load_factor_scale";
      double_param.value = 1.0;  // No load when mower is off
      config_msg.doubles.push_back(double_param);
      dr_pub_.publish(config_msg);
      return;
    }

    if (rpm_buffer_.empty()) {
      ROS_INFO("MowerMonitor: Mower motor enabled, but no RPM data received yet for current window.");
      return;
    }

    // Calculate the sum of RPMs in the current window
    float sum_rpm = 0.0;
    for (const auto& entry : rpm_buffer_) {
      sum_rpm += entry.second;
    }

    // Calculate the average RPM
    float average_rpm = 0.0;
    if (rpm_buffer_.size() > 0) {
      average_rpm = sum_rpm / rpm_buffer_.size();
    }

    // Calculate load_factor
    float load_factor = 1.0f;  // Default to 1.0 (full speed)
    if (max_rpm_ > 0) {        // Avoid division by zero
      load_factor = average_rpm / max_rpm_;
    }

    // Cap load_factor to prevent it exceeding 1.0 (if average somehow exceeds max)
    load_factor = std::min(1.0f, load_factor);
    load_factor = std::max(0.3f, load_factor);  // Ensure load_factor doesn't go below 0.3

    ROS_INFO_STREAM("MowerMonitor: Current average RPM: " << average_rpm << " | Max RPM observed: " << max_rpm_
                                                          << " | Load Factor: " << load_factor * 100.0 << "%");

    // Publish the load_factor to ftc_local_planner via dynamic reconfigure
    dynamic_reconfigure::Config config_msg;
    dynamic_reconfigure::DoubleParameter double_param;
    double_param.name = "load_factor_scale";
    double_param.value = load_factor;
    config_msg.doubles.push_back(double_param);
    dr_pub_.publish(config_msg);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mower_monitor_node");
  MowerMonitorNode mower_monitor;
  ros::spin();
  return 0;
}
