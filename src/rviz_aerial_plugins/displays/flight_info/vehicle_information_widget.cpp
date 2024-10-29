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


  QGroupBox* groupBox = new QGroupBox();

  QVBoxLayout* topic_layout = new QVBoxLayout;
  topic_layout->addWidget( alt_text );
  topic_layout->addWidget( alt_label );
  topic_layout->addWidget( ground_speed_text );
  topic_layout->addWidget( ground_speed_label );
  topic_layout->addWidget( ping_text );
  topic_layout->addWidget( ping_label );

  // Add roll controller indicator to the layout
  topic_layout->addWidget(roll_text);
  topic_layout->addWidget(roll_label);

  QHBoxLayout* heartbeat_layout = new QHBoxLayout;
  heartbeat_layout->addWidget(heartbeat_text);
  heartbeat_layout->addWidget(heartbeat_label);

  topic_layout->addLayout(heartbeat_layout);  // Add the heartbeat layout


  groupBox->setLayout(topic_layout);

  QGridLayout *grid = new QGridLayout;
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