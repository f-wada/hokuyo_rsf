/*
 * hokuyo_spel_master
 * 
 * Copyright (c) 2025 LOCT Co., Ltd.
 * All rights reserved.
 *
 * This software is the property of LOCT Co., Ltd.
 * It is provided solely for evaluation and joint development purposes
 * under prior agreement with LOCT Co., Ltd.
 *
 * Redistribution or use outside the agreed scope is prohibited
 * without written permission from LOCT Co., Ltd.
 */

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

#include <string>

class SpelDiagnosticsPublisher {
 public:
  SpelDiagnosticsPublisher()
  : nh_(), pnh_("~")
  {
    std::string topic;
    pnh_.param<std::string>("diagnostics_topic", topic, "/diagnostics");
    pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(topic, 10);
    timer_ = nh_.createTimer(ros::Duration(1.0), &SpelDiagnosticsPublisher::onTimer, this);

    ROS_INFO("SPEL diagnostics publisher started (1 Hz).");
  }

 private:
  void onTimer(const ros::TimerEvent&) {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();

    diagnostic_msgs::DiagnosticStatus status;
    status.name = "spel_device";
    status.hardware_id = "H0000001";
    status.level = diagnostic_msgs::DiagnosticStatus::OK;
    status.message = "OK";

    // ---- Key-Value entries ----
    add(status, "ip_address", "192.168.0.1");
    add(status, "ip_port", "10940");
    add(status, "product_name", "RSF-X001");
    add(status, "firmware_version", "1.0.0");
    add(status, "device_id", "H0000001");
    add(status, "device_status", "0");
    add(status, "device_temperature", "35000");
    add(status, "cpu_usage", "35");
    add(status, "elapsed_time", "10000");
    add(status, "odometry_state", "0");
    add(status, "odometry_type", "0");
    add(status, "gnss_state", "0");
    add(status, "gnss_type", "0");

    array.status.push_back(status);
    pub_.publish(array);
  }

  static void add(
    diagnostic_msgs::DiagnosticStatus& status,
    const std::string& key,
    const std::string& value)
  {
    diagnostic_msgs::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(kv);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_;
  ros::Timer timer_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "spel_diagnostics_publisher");
  SpelDiagnosticsPublisher publisher;
  ros::spin();
  return 0;
}
