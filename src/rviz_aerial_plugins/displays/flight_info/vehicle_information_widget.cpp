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

#include "rviz_aerial_plugins/displays/flight_info/vehicle_information_widget.hpp"

VehicleInformationWidget::VehicleInformationWidget(QWidget* parent)
{

  node_ = std::make_shared<rclcpp::Node>("vehicle_information_widget_node");

  // Initialize the PID publisher
  pid_publisher_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("pid_values", 10);

  QFont font = QFont();
  font.setBold(true);

  alt_text = new QLabel("Depth (m)");
  alt_label = new QLabel("-0.1");
  alt_text->setAlignment(Qt::AlignCenter);
  alt_label->setAlignment(Qt::AlignCenter);
  alt_text->setFont(font);

  ground_speed_text = new QLabel("Tempreture (Celcius)");
  ground_speed_label = new QLabel("0.0");
  ground_speed_text->setAlignment(Qt::AlignCenter);
  ground_speed_label->setAlignment(Qt::AlignCenter);
  ground_speed_text->setFont(font);
  
  ping_text = new QLabel("Ping (mm)");
  ping_label = new QLabel("0.00");
  ping_text->setAlignment(Qt::AlignCenter);
  ping_label->setAlignment(Qt::AlignCenter);
  ping_text->setFont(font);

  heartbeat_text = new QLabel("Heartbeat (Pi, Teensy):");
  heartbeat_label = new QLabel("No Signal, No Signal");
  heartbeat_text->setAlignment(Qt::AlignCenter);
  heartbeat_label->setAlignment(Qt::AlignCenter);
  heartbeat_text->setFont(font);

    // New Roll Controller Indicator
  roll_text = new QLabel("Roll Controller Status:");
  roll_label = new QLabel("Inactive");  // Default state
  roll_text->setAlignment(Qt::AlignCenter);
  roll_label->setAlignment(Qt::AlignCenter);
  roll_text->setFont(font);

  // New PID indicators with QLineEdit inputs
  QLabel* pid_text = new QLabel("PID Values:");
  pid_text->setFont(font);
  pid_text->setAlignment(Qt::AlignCenter);

  QLabel* p_text = new QLabel("P:");
  p_input = new QLineEdit("0.1");
  p_text->setAlignment(Qt::AlignLeft);
  p_input->setAlignment(Qt::AlignRight);

  QLabel* i_text = new QLabel("I:");
  i_input = new QLineEdit("0.005");
  i_text->setAlignment(Qt::AlignLeft);
  i_input->setAlignment(Qt::AlignRight);

  QLabel* d_text = new QLabel("D:");
  d_input = new QLineEdit("0.01");
  d_text->setAlignment(Qt::AlignLeft);
  d_input->setAlignment(Qt::AlignRight);

  // Publish PID button
  publish_pid_button = new QPushButton("Publish PID");
  connect(publish_pid_button, &QPushButton::clicked, this, &VehicleInformationWidget::publishPIDValues);

  // Group box and layout
  QGroupBox* groupBox = new QGroupBox();
  QVBoxLayout* topic_layout = new QVBoxLayout;

  // Add existing widgets
  topic_layout->addWidget(alt_text);
  topic_layout->addWidget(alt_label);
  topic_layout->addWidget(ground_speed_text);
  topic_layout->addWidget(ground_speed_label);
  topic_layout->addWidget(ping_text);
  topic_layout->addWidget(ping_label);
  topic_layout->addWidget(roll_text);
  topic_layout->addWidget(roll_label);

  // Add heartbeat layout
  QHBoxLayout* heartbeat_layout = new QHBoxLayout;
  heartbeat_layout->addWidget(heartbeat_text);
  heartbeat_layout->addWidget(heartbeat_label);
  topic_layout->addLayout(heartbeat_layout);

  // Add PID labels to layout with input fields
  topic_layout->addWidget(pid_text);
  QHBoxLayout* pid_layout = new QHBoxLayout;
  pid_layout->addWidget(p_text);
  pid_layout->addWidget(p_input);
  pid_layout->addWidget(i_text);
  pid_layout->addWidget(i_input);
  pid_layout->addWidget(d_text);
  pid_layout->addWidget(d_input);
  topic_layout->addLayout(pid_layout);

  // Add publish PID button to layout
  topic_layout->addWidget(publish_pid_button);

  groupBox->setLayout(topic_layout);
  QGridLayout* grid = new QGridLayout;
  grid->addWidget(groupBox, 0, 0);

  setLayout(grid);
}

void VehicleInformationWidget::setFlightTime()
{
}

void VehicleInformationWidget::setGroundSpeed(float speed)
{
  ground_speed_label->setText(QString("%1").arg(speed));
}

void VehicleInformationWidget::setAlt(float alt)
{
  alt_label->setText(QString("%1").arg(alt));
}

void VehicleInformationWidget::setPingInfo(float distance, float confidence) {
    QString text = QString("Distance: %1 mm | Confidence: %2%")
                       .arg(distance, 0, 'f', 2)
                       .arg(confidence, 0, 'f', 1);  // Display as percentage
    ping_label->setText(text);
}


void VehicleInformationWidget::setHeartbeat(const QString& pi_status, const QString& teensy_status)
{
    QString pi_color = (pi_status == "Connected") ? "green" : "red";
    QString teensy_color = (teensy_status == "Connected") ? "green" : "red";

    // Apply the style to the label with HTML formatting
    QString styled_text = QString(
        "<span style='color:%1'>Pi: %2</span> | <span style='color:%3'>Teensy: %4</span>")
        .arg(pi_color, pi_status, teensy_color, teensy_status);

    heartbeat_label->setText(styled_text);
}

// Function to set roll controller status
void VehicleInformationWidget::setRollStatus(bool active)
{
    QString status = active ? "Active" : "Inactive";
    QString color = active ? "green" : "red";

    // Use HTML formatting to color the text
    QString styled_text = QString("<span style='color:%1'>%2</span>")
                              .arg(color, status);

    roll_label->setText(styled_text);
}
void VehicleInformationWidget::publishPIDValues()
{
  // Retrieve current PID values from input fields
  float p = p_input->text().toFloat();
  float i = i_input->text().toFloat();
  float d = d_input->text().toFloat();

  // Prepare the message to publish
  std_msgs::msg::Float32MultiArray pid_msg;
  pid_msg.data = {p, i, d};

  // Publish the message
  pid_publisher_->publish(pid_msg);

  // Debugging message (optional)
  RCLCPP_INFO(node_->get_logger(), "Published PID values - P: %f, I: %f, D: %f", p, i, d);
}