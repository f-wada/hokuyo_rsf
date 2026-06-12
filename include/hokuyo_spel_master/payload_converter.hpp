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

#pragma once

#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nmea_msgs/Gpgga.h>
#include <nmea_msgs/Gprmc.h>
#include <hokuyo_spel_master/gpzda_msg.hpp>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

#include <arpa/inet.h>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

#include <hokuyo_spel_master/spnet_utils.hpp>

namespace hsp {

class PayloadConverter {

 public:
  PayloadConverter();

  ~PayloadConverter();

  void odomToPayload(
    const nav_msgs::Odometry& odom,
    std::vector<uint8_t>& pl);

  void hokuyoCloud2ToPayload(
    const sensor_msgs::PointCloud2& cloud,
    std::vector<uint8_t>& pl);

  void imuToPayload(
    const sensor_msgs::Imu& imu,
    std::vector<uint8_t>& pl);

  void navSatFixToPayload(
    const sensor_msgs::NavSatFix& msg,
    std::vector<uint8_t>& pl);

  void gpggaToPayload(
    const nmea_msgs::Gpgga& msg,
    std::vector<uint8_t>& pl);

  void gprmcToPayload(
    const nmea_msgs::Gprmc& msg,
    std::vector<uint8_t>& pl);

  void gpzdaToPayload(
    const GpzdaMsg& gpzda,
    std::vector<uint8_t>& pl);

  void stringToPayload(
    const std_msgs::String& msg,
    std::vector<uint8_t>& pl);

  void diagnosticsToPayload(
    const diagnostic_msgs::DiagnosticArray& diag,
    std::vector<uint8_t>& pl,
    const std::string& target_status_name);

 private:
  static inline std::string Trim(std::string s) {
    auto not_space = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
    return s;
  }

  static inline bool ParseU8(const std::string& s, uint8_t& out) {
    char* end = nullptr;
    errno = 0;
    long v = std::strtol(s.c_str(), &end, 10);
    if (errno != 0 || end == s.c_str() || *end != '\0') return false;
    if (v < 0 || v > 255) return false;
    out = static_cast<uint8_t>(v);
    return true;
  }

  static inline bool ParseU16(const std::string& s, uint16_t& out) {
    char* end = nullptr;
    errno = 0;
    long v = std::strtol(s.c_str(), &end, 10);
    if (errno != 0 || end == s.c_str() || *end != '\0') return false;
    if (v < 0 || v > 65535) return false;
    out = static_cast<uint16_t>(v);
    return true;
  }

  static inline bool ParseU32(const std::string& s, uint32_t& out) {
    char* end = nullptr;
    errno = 0;
    unsigned long v = std::strtoul(s.c_str(), &end, 10);
    if (errno != 0 || end == s.c_str() || *end != '\0') return false;
    if (v > 0xFFFFFFFFul) return false;
    out = static_cast<uint32_t>(v);
    return true;
  }

  static inline bool ParseU64(const std::string& s, uint64_t& out) {
    char* end = nullptr;
    errno = 0;
    unsigned long long v = std::strtoull(s.c_str(), &end, 10);
    if (errno != 0 || end == s.c_str() || *end != '\0') return false;
    out = static_cast<uint64_t>(v);
    return true;
  }

  // "1.0.0" -> 1,0,0
  static inline bool ParseSemver3(const std::string& s,
                                uint8_t& maj, uint8_t& min, uint8_t& pat) {
    std::string t = Trim(s);
    size_t p1 = t.find('.');
    if (p1 == std::string::npos) return false;
    size_t p2 = t.find('.', p1 + 1);
    if (p2 == std::string::npos) return false;
    uint8_t a,b,c;
    if (!ParseU8(t.substr(0, p1), a)) return false;
    if (!ParseU8(t.substr(p1 + 1, p2 - (p1 + 1)), b)) return false;
    if (!ParseU8(t.substr(p2 + 1), c)) return false;
    maj=a; min=b; pat=c;
    return true;
  }

  static inline std::unordered_map<std::string, std::string>
  ToMap(const diagnostic_msgs::DiagnosticStatus& st) {
    std::unordered_map<std::string, std::string> m;
    m.reserve(st.values.size());
    for (const auto& kv : st.values) {
      m[kv.key] = kv.value;
    }
    return m;
  }
}; // class PayloadConverter

} //  namespace hsp