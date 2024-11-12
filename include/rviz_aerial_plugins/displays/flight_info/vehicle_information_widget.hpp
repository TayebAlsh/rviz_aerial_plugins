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

#ifndef VEHICLE_INFORMATION_WIDGET_H
#define VEHICLE_INFORMATION_WIDGET_H

#include <QWidget>
#include <QLabel>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QLineEdit>     // Include QLineEdit for editable PID values
#include <std_msgs/msg/float32_multi_array.hpp>


#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

class VehicleInformationWidget: public QWidget
{
public:
    VehicleInformationWidget(QWidget* parent = 0);

    void setAlt(float alt);
    void setGroundSpeed(float speed);
    void setPingInfo(float distance,float confidence);
    void setFlightTime();
    void setHeartbeat(const QString& pi_status, const QString& teensy_status);
    void setRollStatus(bool active);
    void publishPIDValues();  // Slot for publishing PID values

private:
    QLabel* alt_text;
    QLabel* ground_speed_text;
    QLabel* alt_label;
    QLabel* ground_speed_label;
    QLabel* ping_text;
    QLabel* ping_label;
    QLabel* heartbeat_text;
    QLabel* heartbeat_label;
    QLabel* roll_text;
    QLabel* roll_label;

       // PID input fields
    QLineEdit* p_input;
    QLineEdit* i_input;
    QLineEdit* d_input;

    // Publish button for PID values
    QPushButton* publish_pid_button;

      // ROS 2 Node and Publisher for PID values
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pid_publisher_;
    
};

#endif // VEHICLE_INFORMATION_WIDGET_H
