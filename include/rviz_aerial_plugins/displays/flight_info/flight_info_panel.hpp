// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef RVIZ_AERIAL_PLUGINS__PANELS__FLIGHTINFO__FLIGHTINFO_PANEL_HPP_
#define RVIZ_AERIAL_PLUGINS__PANELS__FLIGHTINFO__FLIGHTINFO_PANEL_HPP_

#include <memory>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QComboBox>

#include <rviz_aerial_plugins/utils/utils.hpp>

#ifndef Q_MOC_RUN

#include "rviz_common/panel.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_aerial_plugins/visibility_control.hpp"
#include "proposed_aerial_msgs/msg/attitude.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rviz_aerial_plugins/displays/flight_info/compass_widget.hpp"
#include "rviz_aerial_plugins/displays/flight_info/attitude_display_indicator_widget.hpp"
#include "rviz_aerial_plugins/displays/flight_info/vehicle_information_widget.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "tf2/utils.h"
#include "atl_msgs/msg/depth.hpp"
#include <atl_msgs/msg/ping_data.hpp>
#include <sensor_msgs/msg/imu.hpp> 
#include "std_msgs/msg/float32.hpp"
#include <QTimer>
#include <QProcess>
#include "atl_msgs/msg/servos_input.hpp"

#endif

namespace rviz_aerial_plugins
{

namespace displays
{

class RVIZ_AERIAL_PLUGINS_PUBLIC FlighInfoDisplay:
    public rviz_common::Panel
{
  Q_OBJECT
public:
  FlighInfoDisplay(QWidget* parent = 0);
  ~FlighInfoDisplay() override;

  void onInitialize() override;
  void checkHeartbeat();
  void updateHeartbeatStatus(const QString& pi_status, const QString& teensy_status);
  void shutdownPi();

private:
  rclcpp::Subscription<atl_msgs::msg::Depth>::SharedPtr vehicle_odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr vehicle_attitude_sub_;
  rclcpp::Subscription<atl_msgs::msg::PingData>::SharedPtr ping_distance_sub_;
  rclcpp::Subscription<atl_msgs::msg::ServosInput>::SharedPtr servos_input_sub_;
  


  void subcribe2topics();
  void add_namespaces_to_combobox();
  

  QTimer* heartbeat_timer_;  // Timer to periodically check heartbeat

  QString pi_ip_ = "192.168.1.100";  // Raspberry Pi IP
  QString teensy_ip_ = "192.168.1.101";  // Teensy IP

  QString pi_last_status_ = "Unknown";  // Store last status
  QString teensy_last_status_ = "Unknown";

private slots:
  void on_changed_namespace(const QString& text);
  void on_click_refresheButton();
  

protected:
  CompassWidget* compass_widget_;
  ADIWidget* adi_widget_;
  VehicleInformationWidget* vi_widget_;
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
  QComboBox* namespace_;
  std::string attitude_topic_name_;
  std::string odometry_topic_name_;
  std::string ping_topic_name_;
};

} // namespace displays

} // namespace rviz_hrim_plugin_

#endif // RVIZ_AERIAL_PLUGINS__DISPLAYS__FLIGHTINFO__FLIGHTINFO_DISPLAY_HPP_
