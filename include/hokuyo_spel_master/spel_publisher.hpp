/*
 * hokuyo_spel_ros_node
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

#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nmea_msgs/Gpgga.h>
#include <nmea_msgs/Gprmc.h>
#include <hokuyo_spel_master/gpzda_msg.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

#include <iostream>
#include <string>
#include <cstring>
#include <vector>
#include <memory>
 
#include <hokuyo_spel_master/spnet_utils.hpp>
 
namespace hsp {
 
class HokuyoSpelPublisher {
     
 public:
  HokuyoSpelPublisher();
     
  ~HokuyoSpelPublisher();

  void publishTf(
    const std::shared_ptr<tf2_ros::TransformBroadcaster>& broadcaster,
    uint64_t stamp,
    const std::string& frame_id,
    const std::string& child_frame_id,
    const spnet::OdomPacket& pkt);
  
  void publishOdom(
    const ros::Publisher& pub,
    uint64_t stamp,
    const std::string& frame_id,
    const std::string& child_frame_id,
    const spnet::OdomPacket& pkt);

  void publishHokuyoCloud2(
    const ros::Publisher& pub,
    uint64_t stamp,
    const std::string& frame_id,
    const spnet::PointCloudPacketHeader& hdr,
    const spnet::PointXYZIT* points);

  void publishImu(
    const ros::Publisher& pub,
    uint64_t stamp,
    const std::string& frame_id,
    const spnet::ImuPacket& pkt);

  void publishNavSatFix(
    const ros::Publisher& pub,
    uint64_t stamp,
    const std::string& frame_id,
    const spnet::NavSatFixPacket& pkt);

  void publishGpgga(
    const ros::Publisher& pub,
    uint64_t stamp,
    const std::string& frame_id,
    const spnet::GpggaPacket& pkt);

  void publishGprmc(
    const ros::Publisher& pub,
    uint64_t stamp,
    const std::string& frame_id,
    const spnet::GprmcPacket& pkt);

  void publishGpzda(
    const ros::Publisher& pub,
    uint64_t stamp,
    const std::string& frame_id,
    const spnet::GpzdaPacket& pkt);

  void publishString(
    const ros::Publisher& pub,
    const std::string& str);

  void publishDiagnostics(
    const ros::Publisher& pub,
    uint64_t stamp,
    const spnet::DiagnosticPacket& pkt);
}; // class HokuyoSpelPublisher
     
} // namespace hsp