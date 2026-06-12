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
#include <std_msgs/UInt8.h>

#include <cstdlib>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "send_uint8");

  // argv[1] = uint8 value
  if (argc < 2) {
    std::cerr << "Usage: rosrun hokuyo_rsf send_uint8_command <0-255>\n"
              << "Example: rosrun hokuyo_rsf send_uint8_command 1\n";
    return 1;
  }

  int value = std::atoi(argv[1]);
  if (value < 0 || value > 255) {
    std::cerr << "Value must be in range [0, 255]\n";
    return 1;
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Topic parameter (合わせやすいように param 化)
  std::string topic;
  pnh.param<std::string>("topic", topic, "/spel/cmd_to_spel");

  ros::Publisher pub = nh.advertise<std_msgs::UInt8>(topic, 10, true);

  // Wait for publisher setup and subscriber connection.
  ros::Duration(0.2).sleep();

  std_msgs::UInt8 msg;
  msg.data = static_cast<uint8_t>(value);
  pub.publish(msg);

  // Flush
  ros::spinOnce();
  ros::Duration(0.2).sleep();

  ROS_INFO("Published UInt8 value %u on topic '%s'", msg.data, topic.c_str());

  return 0;
}
