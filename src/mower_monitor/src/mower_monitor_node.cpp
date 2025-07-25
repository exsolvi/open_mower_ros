#include <mower_msgs/Status.h>
#include <ros/ros.h>

#include <deque>
#include <numeric>  // For std::accumulate

class MowerMonitorNode {
 public:
  MowerMonitorNode() : nh_("~") {
    status_sub_ = nh_.subscribe("/ll/mower_status", 10, &MowerMonitorNode::statusCallback, this);
    log_timer_ = nh_.createTimer(ros::Duration(1.0), &MowerMonitorNode::logTimerCallback, this);
    window_duration_ = ros::Duration(0.5);  // 500 milliseconds
  }

 private:
  ros::NodeHandle nh_;
  ros::Subscriber status_sub_;
  ros::Timer log_timer_;
  ros::Duration window_duration_;

  // Deque to store RPM values with their timestamps for moving average calculation
  std::deque<std::pair<ros::Time, float>> rpm_buffer_;
  float max_rpm_ = 0.0f;      // Stores the maximum observed RPM
  bool mow_enabled_ = false;  // Tracks if the mower motor is enabled

  void statusCallback(const mower_msgs::Status::ConstPtr& msg) {
    // Update mow_enabled_ state
    mow_enabled_ = msg->mow_enabled;

    // Get the mowing motor RPM
    float current_rpm = msg->mower_motor_rpm;
    ros::Time current_time = ros::Time::now();

    // Update maximum RPM observed
    if (current_rpm > max_rpm_) {
      max_rpm_ = current_rpm;
    }

    // Add current RPM and timestamp to the buffer
    rpm_buffer_.push_back(std::make_pair(current_time, current_rpm));

    // Remove old values from the buffer that are outside the window
    while (!rpm_buffer_.empty() && (current_time - rpm_buffer_.front().first) > window_duration_) {
      rpm_buffer_.pop_front();
    }
  }

  void logTimerCallback(const ros::TimerEvent& event) {
    if (!mow_enabled_) {
      ROS_INFO("MowerMonitor: Mower motor is not enabled. Skipping RPM calculation and logging.");
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
    float average_rpm = sum_rpm / rpm_buffer_.size();

    ROS_INFO_STREAM("MowerMonitor: Current average RPM over " << window_duration_.toSec() * 1000.0 << "ms window: "
                                                              << average_rpm << " | Max RPM observed: " << max_rpm_);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mower_monitor_node");
  MowerMonitorNode mower_monitor;
  ros::spin();
  return 0;
}
