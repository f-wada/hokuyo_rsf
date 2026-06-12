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

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nmea_msgs/Gpgga.h>
#include <nmea_msgs/Gprmc.h>
#include <nmea_msgs/Gpzda.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <unistd.h>
#include <signal.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>

#include <mutex>
#include <thread>
#include <atomic>
#include <cstring>
#include <iostream>
#include <vector>
#include <chrono>
#include <cstdint>
#include <memory>

#include <hokuyo_spel_master/spnet_utils.hpp>
#include <hokuyo_spel_master/payload_converter.hpp>
#include <hokuyo_spel_master/spel_parser.hpp>
#include <hokuyo_spel_master/spel_publisher.hpp>

// this should be a common file for master, lio, and spel_ros_node
// #include <hokuyo_spel_master/lio_status.hpp>

class HokuyoSpelRosNode {
 public:
  HokuyoSpelRosNode()
  : nh_(), pnh_("~")
  {
    is_set_last_imu_rate_odom_ = false;

    // Subscriber
    std::string cmdToSpelTopic;
    pnh_.param<std::string>("cmd_to_spel_topic", cmdToSpelTopic, "/spel/cmd_to_spel");
    cmdToSpelSub_ = nh_.subscribe(cmdToSpelTopic, 10, &HokuyoSpelRosNode::cmdToSpelCallback, this);

    std::string ipAddressTopic;
    pnh_.param<std::string>("ip_address_topic", ipAddressTopic, "/spel/ip_address");
    ipAddressSub_ = nh_.subscribe(ipAddressTopic, 10, &HokuyoSpelRosNode::ipAddressCallback, this);

    // Publisher
    std::string navSatFixTopic;
    pnh_.param<std::string>("nav_sat_fix_topic", navSatFixTopic, "/spel/nav_sat_fix");
    navSatFixPub_ = nh_.advertise<sensor_msgs::NavSatFix>(navSatFixTopic, 100);

    std::string gpggaTopic;
    pnh_.param<std::string>("gpgga_topic", gpggaTopic, "/spel/gpgga");
    gpggaPub_ = nh_.advertise<nmea_msgs::Gpgga>(gpggaTopic, 100);

    std::string gprmcTopic;
    pnh_.param<std::string>("gprmc_topic", gprmcTopic, "/spel/gprmc");
    gprmcPub_ = nh_.advertise<nmea_msgs::Gprmc>(gprmcTopic, 100);

    std::string gpzdaTopic;
    pnh_.param<std::string>("gpzda_topic", gpzdaTopic, "/spel/gpzda");
    gpzdaPub_ = nh_.advertise<nmea_msgs::Gpzda>(gpzdaTopic, 100);

    std::string hokuyoCloud2Topic;
    pnh_.param<std::string>("hokuyo_cloud2_topic", hokuyoCloud2Topic, "/spel/hokuyo_cloud2");
    hokuyoCloud2Pub_ = nh_.advertise<sensor_msgs::PointCloud2>(hokuyoCloud2Topic, 100);

    std::string imuTopic;
    pnh_.param<std::string>("imu_topic", imuTopic, "/spel/imu");
    imuPub_ = nh_.advertise<sensor_msgs::Imu>(imuTopic, 100);

    std::string imuRateOdomTopic;
    pnh_.param<std::string>("imu_rate_odom_topic", imuRateOdomTopic, "/spel/imu_rate_odom");
    imuRateOdomPub_ = nh_.advertise<nav_msgs::Odometry>(imuRateOdomTopic, 100);

    std::string lidarRateOdomTopic;
    pnh_.param<std::string>("lidar_rate_odom_topic", lidarRateOdomTopic, "/spel/lidar_rate_odom");
    lidarRateOdomPub_ = nh_.advertise<nav_msgs::Odometry>(lidarRateOdomTopic, 100);

    std::string navSatFixSwitchTopic;
    pnh_.param<std::string>("nav_sat_fix_switch_topic", navSatFixSwitchTopic, "/spel/nav_sat_fix_switch");
    navSatFixSwitchPub_ = nh_.advertise<sensor_msgs::NavSatFix>(navSatFixSwitchTopic, 100);

    std::string utmOdomTopic;
    pnh_.param<std::string>("utm_odom_topic", utmOdomTopic, "/spel/utm_odom");
    utmOdomPub_ = nh_.advertise<nav_msgs::Odometry>(utmOdomTopic, 100);

    std::string switchOdomTopic;
    pnh_.param<std::string>("switch_odom_topic", switchOdomTopic, "/spel/switch_odom");
    switchOdomPub_ = nh_.advertise<nav_msgs::Odometry>(switchOdomTopic, 100);

    std::string switchOdomStateTopic;
    pnh_.param<std::string>("switch_odom_state_topic", switchOdomStateTopic, "/spel/switch_odom_state");
    switchOdomStatePub_ = nh_.advertise<std_msgs::String>(switchOdomStateTopic, 100);

    std::string switchOdomTypeTopic;
    pnh_.param<std::string>("switch_odom_type_topic", switchOdomTypeTopic, "/spel/switch_odom_type");
    switchOdomTypePub_ = nh_.advertise<std_msgs::String>(switchOdomTypeTopic, 100);

    std::string switchFixStateTopic;
    pnh_.param<std::string>("switch_fix_state_topic", switchFixStateTopic, "/spel/switch_fix_state");
    switchFixStatePub_ = nh_.advertise<std_msgs::String>(switchFixStateTopic, 100);

    std::string switchFixTypeTopic;
    pnh_.param<std::string>("switch_fix_type_topic", switchFixTypeTopic, "/spel/switch_fix_type");
    switchFixTypePub_ = nh_.advertise<std_msgs::String>(switchFixTypeTopic, 100);

    std::string diagnosticsTopic;
    pnh_.param<std::string>("diagnostics_topic", diagnosticsTopic, "/spel/diagnostics");
    diagnosticsPub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(diagnosticsTopic, 100);

    // Load SPEL parameters
    pnh_.param<std::string>("spel_ip_address", spelIpAdress_, "127.0.0.1");
    pnh_.param<int>("spel_port", spelPort_, 10940);
    std::cout << "IP Address: " << spelIpAdress_ << " Port: " << spelPort_ << std::endl;

    pnh_.param<bool>("broadcast_tf", broadcastTf_, true);
    pnh_.param<std::string>("odom_frame", odomFrame_, "odom");
    pnh_.param<std::string>("lidr_frame", lidarFrame_, "hokuyo3d");
    pnh_.param<std::string>("imu_frame", imuFrame_, "hokuyo3d_imu");
    pnh_.param<std::string>("gnss_frame", gnssFrame_, "gnss");
    pnh_.param<std::string>("utm_frame", utmFrame_, "utm/utm_53Z");
    if (broadcastTf_) {
      tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
    }

    // Initialization for socket communication
    setupSocket();
    clientRunning_.store(true);
    clientThread_ = std::thread(&HokuyoSpelRosNode::spelClientLoop, this);
  }

  ~HokuyoSpelRosNode() {
    HokuyoSpelRosNode::spelClientClose(sock_);

    clientRunning_.store(false);
    ::shutdown(sock_, SHUT_RDWR);
    ::close(sock_);
    if (clientThread_.joinable()) {
      clientThread_.join();
    }
  }
 private:
  void cmdToSpelCallback(const std_msgs::UInt8::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    cmdToSpel_ = *msg;
    cmdToSpelStamp_ = ros::Time::now().toNSec();
  }

  void ipAddressCallback(const std_msgs::String::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    ipAddress_ = *msg;
    ipAddressStamp_ = ros::Time::now().toNSec();
  }

  void spelClientLoop() {
    std::atomic<bool> run(true);

    while (ros::ok()) {
      std::thread rx(&HokuyoSpelRosNode::spelClientRxLoop, this, sock_, std::ref(run));
      std::thread tx(&HokuyoSpelRosNode::spelClientTxLoop, this, sock_, std::ref(run));
      rx.join();
      run.store(false);
      tx.join();

      if (ros::ok()) {
        cleanupSocket();
        setupSocket();
        run.store(true);
        ROS_INFO( "Restart client loop.");
      }
    }

    ros::shutdown();
  }

  void setupSocket() {
    // Ignore broken pipe error
    signal(SIGPIPE, SIG_IGN);

    sock_ = ::socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(spelPort_);
    ::inet_pton(AF_INET, spelIpAdress_.c_str(), &addr.sin_addr);
    if (connect(sock_, (sockaddr*)&addr, sizeof(addr)) < 0) {
      perror("connect");
      exit(1);
    }

    const int yes = 1;
    ::setsockopt(sock_, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
  }

  void cleanupSocket() {
    ::shutdown(sock_, SHUT_RDWR);
    ::close(sock_);
  }

  void spelClientRxLoop(
    int sock,
    std::atomic<bool>& run)
  {
    while (run.load() && ros::ok()) {
      spnet::Header h{};
      std::vector<uint8_t> pl;
      if (!spnet::recvFrame(sock, h, pl)) {
        ROS_ERROR( "Failed to receive data from SPEL.");
        break;
      }

      const spnet::MsgType type = spnet::toMsgType(h.type);

      if (type == spnet::MsgType::ERROR) {
        std::string str;
        hspParser_.parseStringPayload(pl, str);
        ROS_ERROR( "Got error from SPEL. %s", str.c_str());
        break;
      
      } else if (type == spnet::MsgType::WARN) {
        std::string str;
        hspParser_.parseStringPayload(pl, str);
        ROS_WARN( "Got warning from SPEL. %s", str.c_str());

      } else if (type == spnet::MsgType::ACK) {
        ROS_INFO( "Got ACK.");

      } else if (type == spnet::MsgType::DATA) {
        const spnet::DataType datatype = spnet::toDataType(h.subtype);
        const uint32_t sec = ntohl(h.sec);
        const uint32_t nsec = ntohl(h.nsec);
        const uint64_t stamp = static_cast<uint64_t>(sec) * 1000000000ULL + static_cast<uint64_t>(nsec);;

        if (datatype == spnet::DataType::NAV_SAT_FIX) {
          spnet::NavSatFixPacket pkt;
          if (!hspParser_.parseNavSatFixPayload(pl, pkt)) {
            ROS_ERROR( "Failed to parse NavSatFix payload.");
            continue;
          }
          hspPublisher_.publishNavSatFix(navSatFixPub_, stamp, gnssFrame_, pkt);

        } else if (datatype == spnet::DataType::GPGGA) {
          spnet::GpggaPacket pkt;
          if (!hspParser_.parseGpggaPayload(pl, pkt)) {
            ROS_ERROR( "Failed to parse GPGGA payload.");
            continue;
          }
          hspPublisher_.publishGpgga(gpggaPub_, stamp, gnssFrame_, pkt);

        } else if (datatype == spnet::DataType::GPRMC) {
          spnet::GprmcPacket pkt;
          if (!hspParser_.parseGprmcPayload(pl, pkt)) {
            ROS_ERROR( "Failed to parse GPRMC payload.");
            continue;
          }
          hspPublisher_.publishGprmc(gprmcPub_, stamp, gnssFrame_, pkt);

        } else if (datatype == spnet::DataType::GPZDA) {
          spnet::GpzdaPacket pkt;
          if (!hspParser_.parseGpzdaPayload(pl, pkt)) {
            ROS_ERROR( "Failed to parse GPZDA payload.");
            continue;
          }
          hspPublisher_.publishGpzda(gpzdaPub_, stamp, gnssFrame_, pkt);

        } else if (datatype == spnet::DataType::HOKUYO_CLOUD2) {
          spnet::PointCloudPacketHeader pcHdr{};
          const spnet::PointXYZIT* points = nullptr;
          if (!hspParser_.parseHokuyoCloud2Payload(pl, pcHdr, points)) {
            ROS_ERROR( "Failed to parse hokuyo cloud2 payload.");
            continue;
          }
          hspPublisher_.publishHokuyoCloud2(hokuyoCloud2Pub_, stamp, lidarFrame_, pcHdr, points);
          if(is_set_last_imu_rate_odom_){
            hspPublisher_.publishOdom(lidarRateOdomPub_, stamp, odomFrame_, lidarFrame_, last_imu_rate_odom_pkt_);
          }
        } else if (datatype == spnet::DataType::IMU) {
          spnet::ImuPacket pkt;
          if (!hspParser_.parseImuPayload(pl, pkt)) {
            ROS_ERROR( "Failed to parse IMU payload.");
            continue;
          }
          hspPublisher_.publishImu(imuPub_, stamp, imuFrame_, pkt);

        } else if (datatype == spnet::DataType::IMU_RATE_ODOMETRY) {
          spnet::OdomPacket pkt;
          if (!hspParser_.parseOdomPayload(pl, pkt)) {
            ROS_ERROR( "Failed to parse IMU rate odometry payload.");
            continue;
          }
          hspPublisher_.publishOdom(imuRateOdomPub_, stamp, odomFrame_, lidarFrame_, pkt);
          last_imu_rate_odom_pkt_ = pkt;
          is_set_last_imu_rate_odom_ = true;

          static int tfcnt = 0;
          tfcnt++;
          if (broadcastTf_ && tfcnt == 50) {
            hspPublisher_.publishTf(tfBroadcaster_, stamp, odomFrame_, lidarFrame_, pkt);
            tfcnt = 0;
          }

        } else if (datatype == spnet::DataType::NAV_SAT_FIX_SWITCH) {
          spnet::NavSatFixPacket pkt;
          if (!hspParser_.parseNavSatFixPayload(pl, pkt)) {
            ROS_ERROR( "Failed to parse NavSatFixSwitch payload.");
            continue;
          }
          hspPublisher_.publishNavSatFix(navSatFixSwitchPub_, stamp, gnssFrame_, pkt);

        } else if (datatype == spnet::DataType::UTM_ODOM) {
          spnet::OdomPacket pkt;
          if (!hspParser_.parseOdomPayload(pl, pkt)) {
            ROS_ERROR( "Failed to parse UTM odometry payload.");
            continue;
          }
          hspPublisher_.publishOdom(utmOdomPub_, stamp, utmFrame_, lidarFrame_, pkt);

        } else if (datatype == spnet::DataType::SWITCH_ODOM) {
          spnet::OdomPacket pkt;
          if (!hspParser_.parseOdomPayload(pl, pkt)) {
            ROS_ERROR( "Failed to parse switch odometry payload.");
            continue;
          }
          hspPublisher_.publishOdom(switchOdomPub_, stamp, odomFrame_, lidarFrame_, pkt);

        } else if (datatype == spnet::DataType::SWITCH_ODOM_STATE) {
          std::string str;
          if (!hspParser_.parseStringPayload(pl, str)) {
            ROS_ERROR( "Failed to parse switch odometry state payload.");
            continue;
          }
          hspPublisher_.publishString(switchOdomStatePub_, str);

        } else if (datatype == spnet::DataType::SWITCH_ODOM_TYPE) {
          std::string str;
          if (!hspParser_.parseStringPayload(pl, str)) {
            ROS_ERROR( "Failed to parse switch odometry type payload.");
            continue;
          }
          hspPublisher_.publishString(switchOdomTypePub_, str);

        } else if (datatype == spnet::DataType::SWITCH_FIX_STATE) {
          std::string str;
          if (!hspParser_.parseStringPayload(pl, str)) {
            ROS_ERROR( "Failed to parse switch fix state payload.");
            continue;
          }
          hspPublisher_.publishString(switchFixStatePub_, str);

        } else if (datatype == spnet::DataType::SWITCH_FIX_TYPE) {
          std::string str;
          if (!hspParser_.parseStringPayload(pl, str)) {
            ROS_ERROR( "Failed to parse switch fix type payload.");
            continue;
          }
          hspPublisher_.publishString(switchFixTypePub_, str);

        } else if (datatype == spnet::DataType::DIAGNOSTIC_ARRAY) {
          spnet::DiagnosticPacket pkt;
          if (!hspParser_.parseDiagnosticsPayload(pl, pkt)) {
            ROS_ERROR( "Failed to parse diagnostics payload.");
            continue;
          }
          hspPublisher_.publishDiagnostics(diagnosticsPub_, stamp, pkt);

        }
      }

    }

    run.store(false);
  }

  void spelClientTxLoop(
    int sock,
    std::atomic<bool>& run)
  {
    using namespace std::chrono_literals;

    uint32_t seq = 0;
    std::vector<uint8_t> payload;
    uint32_t payloadSize = payload.size();

    uint64_t prevCmdToSpelStamp = 0;
    uint64_t prevIpAddressStamp = 0;

    uint8_t type    = static_cast<uint8_t>(spnet::MsgType::CMD);
    uint8_t subtype = static_cast<uint8_t>(spnet::CmdType::START_STREAMING);
    uint64_t stamp  = ros::Time::now().toNSec();
    uint32_t sec    = static_cast<uint32_t>(stamp / 1000000000ULL);
    uint32_t nsec   = static_cast<uint32_t>(stamp % 1000000000ULL);
    payload.clear();
    payloadSize = payload.size();
    spnet::sendFrame(sock, type, subtype, sec, nsec, seq, payload.data(), payloadSize);
    subtype = static_cast<uint8_t>(spnet::CmdType::START_RSF);
    spnet::sendFrame(sock, type, subtype, sec, nsec, seq, payload.data(), payloadSize);

    while (run.load() && ros::ok()) {
      std::this_thread::sleep_for(1s);

      std_msgs::UInt8 cmdToSpel;
      uint64_t cmdToSpelStamp = 0;
      bool cmdToSpelUpdated = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        cmdToSpelStamp = cmdToSpelStamp_;
        if (cmdToSpelStamp != prevCmdToSpelStamp) {
          cmdToSpel = cmdToSpel_;
          prevCmdToSpelStamp = cmdToSpelStamp;
          cmdToSpelUpdated = true;
        }
      }        
      if (cmdToSpelUpdated) {
        const uint8_t type = static_cast<uint8_t>(spnet::MsgType::CMD);
        uint8_t subtype;
        if (cmdToSpel.data == 1) {
          subtype = static_cast<uint8_t>(spnet::CmdType::START_STREAMING);
          ROS_INFO(
            "Hokuyo SPEL ROS1 node sends start streaming command.");
        } else if (cmdToSpel.data == 2) {
          subtype = static_cast<uint8_t>(spnet::CmdType::STOP_STREAMING);
          ROS_INFO(
            "Hokuyo SPEL ROS1 node sends stop streaming command.");
        } else if (cmdToSpel.data == 3) {
          subtype = static_cast<uint8_t>(spnet::CmdType::START_RSF);
          ROS_INFO(
            "Hokuyo SPEL ROS1 node sends start software command.");
        } else if (cmdToSpel.data == 4) {
          subtype = static_cast<uint8_t>(spnet::CmdType::STOP_RSF);
          ROS_INFO(
            "Hokuyo SPEL ROS1 node sends stop software command.");
        } else if (cmdToSpel.data == 5) {
          subtype = static_cast<uint8_t>(spnet::CmdType::RESET_RSF);
          ROS_INFO(
            "Hokuyo SPEL ROS1 node sends reset software command.");
        } else {
          subtype = 0;
          ROS_WARN(
            "Hokuyo SPEL ROS1 node sends unknown command.");
        }
        const uint32_t sec = static_cast<uint32_t>(cmdToSpelStamp / 1000000000ULL);
        const uint32_t nsec = static_cast<uint32_t>(cmdToSpelStamp % 1000000000ULL);
        payload.clear();
        payloadSize = payload.size();
        if (!spnet::sendFrame(sock, type, subtype, sec, nsec, seq,
          payload.data(), payloadSize))
        {
          ROS_ERROR( "Failed to send a command to SPEL.");
          run.store(false);
          break;
        }
      }

      std_msgs::String ipAddress;
      uint64_t ipAddressStamp = 0;
      bool ipAddressUpdated = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        ipAddressStamp = ipAddressStamp_;
        if (ipAddressStamp != prevIpAddressStamp) {
          ipAddress = ipAddress_;
          prevIpAddressStamp = ipAddressStamp;
          ipAddressUpdated = true;
        }
      }
      if (ipAddressUpdated) {
        uint32_t ip;
        if (!spnet::Ipv4StringToUint32(ipAddress.data, ip)) {
          ROS_INFO(
            "Hokuyo SPEL ROS1 node receives incorrect IP address: %s", ipAddress.data.c_str());
        } else {
          const uint8_t type = static_cast<uint8_t>(spnet::MsgType::CMD);
          const uint8_t subtype = static_cast<uint8_t>(spnet::CmdType::SET_IP_ADDRESS);
          const uint32_t sec = static_cast<uint32_t>(ipAddressStamp / 1000000000ULL);
          const uint32_t nsec = static_cast<uint32_t>(ipAddressStamp % 1000000000ULL);
          payload.resize(ipAddress.data.size());
          std::memcpy(payload.data(), ipAddress.data.data(), ipAddress.data.size());
          payloadSize = payload.size();
          ROS_INFO(
            "Hokuyo SPEL ROS1 node sends IP address: %s", ipAddress.data.c_str());
          if (!spnet::sendFrame(sock, type, subtype, sec, nsec, seq,
            payload.data(), payloadSize))
          {
            ROS_ERROR( "Failed to send IP address.");
            run.store(false);
            break;
          }
        }
      }
    }

    run.store(false);
  }

  void spelClientClose(
    int sock)
  {
    uint32_t seq = 0;
    std::vector<uint8_t> payload;
    uint32_t payloadSize = payload.size();

    uint8_t type    = static_cast<uint8_t>(spnet::MsgType::CMD);
    uint8_t subtype = static_cast<uint8_t>(spnet::CmdType::STOP_RSF);
    uint64_t stamp  = ros::Time::now().toNSec();
    uint32_t sec    = static_cast<uint32_t>(stamp / 1000000000ULL);
    uint32_t nsec   = static_cast<uint32_t>(stamp % 1000000000ULL);
    payload.clear();
    payloadSize = payload.size();
    spnet::sendFrame(sock, type, subtype, sec, nsec, seq, payload.data(), payloadSize);
    subtype = static_cast<uint8_t>(spnet::CmdType::STOP_STREAMING);
    spnet::sendFrame(sock, type, subtype, sec, nsec, seq, payload.data(), payloadSize);
    ROS_INFO( "Close RSF connection.");
  }

  int sock_;
  std::thread clientThread_;
  std::atomic<bool> clientRunning_{false};
  uint64_t latestStreamingStamp_;
  spnet::OdomPacket last_imu_rate_odom_pkt_;
  bool is_set_last_imu_rate_odom_;

  std::string spelIpAdress_;
  int spelPort_;
  int spelOdomPort_;

  hsp::PayloadConverter plConv_;
  hsp::HokuyoSpelParser hspParser_;
  hsp::HokuyoSpelPublisher hspPublisher_;

  mutable std::mutex mutex_;

  ros::Subscriber cmdToSpelSub_;
  std_msgs::UInt8 cmdToSpel_{};
  uint64_t cmdToSpelStamp_{0};

  ros::Subscriber ipAddressSub_;
  std_msgs::String ipAddress_{};
  uint64_t ipAddressStamp_{0};

  ros::Publisher navSatFixPub_;
  ros::Publisher gpggaPub_;
  ros::Publisher gprmcPub_;
  ros::Publisher gpzdaPub_;
  ros::Publisher hokuyoCloud2Pub_;
  ros::Publisher imuPub_;
  ros::Publisher imuRateOdomPub_;
  ros::Publisher lidarRateOdomPub_;
  ros::Publisher navSatFixSwitchPub_;
  ros::Publisher utmOdomPub_;
  ros::Publisher switchOdomPub_;
  ros::Publisher switchOdomStatePub_;
  ros::Publisher switchOdomTypePub_;
  ros::Publisher switchFixStatePub_;
  ros::Publisher switchFixTypePub_;
  ros::Publisher diagnosticsPub_;

  bool broadcastTf_{true};
  std::string odomFrame_;
  std::string lidarFrame_;
  std::string imuFrame_;
  std::string gnssFrame_;
  std::string utmFrame_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
}; // class HokuyoSpelRosNode

int main(int argc, char** argv) {
  ros::init(argc, argv, "hokuyo_rsf");
  HokuyoSpelRosNode node;
  ros::spin();
  ros::shutdown();
  return 0;
}
