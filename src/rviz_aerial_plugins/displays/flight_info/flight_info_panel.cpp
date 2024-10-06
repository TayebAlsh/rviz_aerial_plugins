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

#include "rviz_aerial_plugins/displays/flight_info/flight_info_panel.hpp"
#include "rviz_common/load_resource.hpp"
#include "atl_msgs/msg/depth.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace rviz_aerial_plugins
{

namespace displays
{

FlighInfoDisplay::FlighInfoDisplay(QWidget* parent):
 rviz_common::Panel(parent), rviz_ros_node_()
{
  // setIcon(rviz_common::loadPixmap("package://rviz_aerial_plugins/icons/classes/Battery.png"));

  odometry_topic_name_ = "/depth";
  attitude_topic_name_ = "/imu";
}

FlighInfoDisplay::~FlighInfoDisplay()
{

}

void FlighInfoDisplay::onInitialize()
{
  rviz_ros_node_ = getDisplayContext()->getRosNodeAbstraction();

  compass_widget_ = new CompassWidget();
  adi_widget_ = new ADIWidget();
  vi_widget_ = new VehicleInformationWidget();
  namespace_ = new QComboBox();
  QPushButton* refresh_button = new QPushButton("Refresh");
  refresh_button->setIcon(rviz_common::loadPixmap("package://rviz_aerial_plugins/icons/classes/Refresh.png"));

  add_namespaces_to_combobox();

  QGridLayout *grid = new QGridLayout;

  grid->addWidget( compass_widget_ , 0, 0);
  grid->addWidget( adi_widget_, 0, 1);
  grid->addWidget( vi_widget_, 1, 0, 1, 2);
  grid->addWidget( namespace_, 2, 0);
  grid->addWidget( refresh_button, 2, 1);

  QObject::connect(namespace_, SIGNAL(currentIndexChanged(QString)),this, SLOT(on_changed_namespace(QString)));
  QObject::connect(refresh_button, SIGNAL(clicked()),this, SLOT(on_click_refresheButton()));

  setLayout(grid);

  subcribe2topics();

}

void FlighInfoDisplay::on_click_refresheButton()
{
  add_namespaces_to_combobox();
  auto names_and_namespaces = rviz_ros_node_.lock()->get_raw_node()->get_node_names();

  std::set<std::string> namespaces = get_namespaces(names_and_namespaces);

  if(namespaces.size() > 0)
    on_changed_namespace(QString((*namespaces.begin()).c_str()));
}

void FlighInfoDisplay::add_namespaces_to_combobox()
{
  auto names_and_namespaces = rviz_ros_node_.lock()->get_raw_node()->get_node_names();

  std::set<std::string> namespaces = get_namespaces(names_and_namespaces);

  namespace_->blockSignals(true);
  namespace_->clear();
  for(auto n: namespaces){
    namespace_->addItem(QString(n.c_str()));
  }
  namespace_->blockSignals(false);
}

void FlighInfoDisplay::on_changed_namespace(const QString& text)
{
  std::string namespace_str(text.toUtf8().constData());

  attitude_topic_name_ = "/imu";
  odometry_topic_name_ = "/depth";
  vehicle_attitude_sub_.reset();
  vehicle_odometry_sub_.reset();

  subcribe2topics();
}

void FlighInfoDisplay::subcribe2topics()
{
// Subscription to the IMU topic using the same variable name
    vehicle_attitude_sub_ = rviz_ros_node_.lock()->get_raw_node()->
    create_subscription<sensor_msgs::msg::Imu>(
        "/imu", // Replace with your actual IMU topic name
        10,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
            // Process the IMU data
            geometry_msgs::msg::Quaternion q = msg->orientation;

            double yaw, pitch, roll;
            tf2::getEulerYPR(q, yaw, pitch, roll);
            
            // Update widgets based on IMU data
            compass_widget_->setAngle(yaw * 180 / M_PI);
            compass_widget_->update();
            adi_widget_->setPitch(pitch * 180 / M_PI);
            adi_widget_->setRoll(-roll * 180 / M_PI);
            adi_widget_->update();
        }
    );

RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(),
             "FlighInfoDisplay: IMU topic subscribed to /imu");
                
                
    vehicle_odometry_sub_ = rviz_ros_node_.lock()->get_raw_node()->create_subscription<atl_msgs::msg::Depth>(
    "/depth",
    10,
    [this](const atl_msgs::msg::Depth::SharedPtr msg) { // Use SharedPtr to match the expected argument type
        // Use the actual data from the message
        vi_widget_->setGroundSpeed(-msg->temperature); // Set ground speed to negative depth (example)
        vi_widget_->setAlt(msg->depth);          // Set altitude to the depth value
        vi_widget_->update();
    });



}

} // namespace displays

} // namespace rviz_aerial_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_aerial_plugins::displays::FlighInfoDisplay, rviz_common::Panel)
