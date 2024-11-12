#include "rviz_aerial_plugins/displays/flight_info/flight_info_panel.hpp"
#include "rviz_common/load_resource.hpp"
#include "atl_msgs/msg/depth.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <QDebug>
#include "atl_msgs/msg/ping_data.hpp"
#include "atl_msgs/msg/servos_input.hpp"


namespace rviz_aerial_plugins {
namespace displays {

FlighInfoDisplay::FlighInfoDisplay(QWidget* parent)
    : rviz_common::Panel(parent), rviz_ros_node_() {
    // Initialize the heartbeat timer
    heartbeat_timer_ = new QTimer(this);
    connect(heartbeat_timer_, &QTimer::timeout, [this]() { checkHeartbeat(); });
    heartbeat_timer_->start(5000);  // Ping every 5 seconds

    // Set topic names
    odometry_topic_name_ = "/depth";
    attitude_topic_name_ = "/imu";
    ping_topic_name_ = "/ping";
}

FlighInfoDisplay::~FlighInfoDisplay() = default;

void FlighInfoDisplay::checkHeartbeat() {
    // Ping Raspberry Pi
    QProcess* pi_ping = new QProcess(this);
    connect(pi_ping, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), 
            this, [this, pi_ping](int exitCode, QProcess::ExitStatus) {
        QString pi_status = (exitCode == 0) ? "Connected" : "Disconnected";
        //qDebug() << "Raspberry Pi Status: " << pi_status;
        
        // Update the widget with Pi status
        vi_widget_->setHeartbeat(pi_status, teensy_last_status_);
        vi_widget_->update();  // Ensure UI refresh
        
        pi_last_status_ = pi_status;  // Store the last known status
        pi_ping->deleteLater();  // Clean up
    });
    pi_ping->start("ping", {"-c", "1", "192.168.2.2"});  // Replace with actual Pi IP

    // Ping Teensy
    QProcess* teensy_ping = new QProcess(this);
    connect(teensy_ping, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), 
            this, [this, teensy_ping](int exitCode, QProcess::ExitStatus) {
        QString teensy_status = (exitCode == 0) ? "Connected" : "Disconnected";
        //qDebug() << "Teensy Status: " << teensy_status;

        // Update the widget with Teensy status
        vi_widget_->setHeartbeat(pi_last_status_, teensy_status);
        vi_widget_->update();  // Ensure UI refresh
        
        teensy_last_status_ = teensy_status;  // Store the last known status
        teensy_ping->deleteLater();  // Clean up
    });
    teensy_ping->start("ping", {"-c", "1", "192.168.2.3"});  // Replace with actual Teensy IP
}

void FlighInfoDisplay::onInitialize() {
    rviz_ros_node_ = getDisplayContext()->getRosNodeAbstraction();

    compass_widget_ = new CompassWidget();
    adi_widget_ = new ADIWidget();
    vi_widget_ = new VehicleInformationWidget();
    namespace_ = new QComboBox();
    QPushButton* refresh_button = new QPushButton("Refresh");
    QPushButton* shutdown_button = new QPushButton("Shutdown Pi");  // New Shutdown Button

    
    refresh_button->setIcon(rviz_common::loadPixmap("package://rviz_aerial_plugins/icons/classes/Refresh.png"));

    add_namespaces_to_combobox();

    QGridLayout* grid = new QGridLayout;
    grid->addWidget(compass_widget_, 0, 0);
    grid->addWidget(adi_widget_, 0, 1);
    grid->addWidget(vi_widget_, 1, 0, 1, 2);
    grid->addWidget(namespace_, 2, 0);
    grid->addWidget(refresh_button, 2, 1);
    grid->addWidget(shutdown_button, 3, 1);  // Add Shutdown Button

    QObject::connect(namespace_, SIGNAL(currentIndexChanged(QString)), 
                     this, SLOT(on_changed_namespace(QString)));
    QObject::connect(refresh_button, SIGNAL(clicked()), 
                     this, SLOT(on_click_refresheButton()));
    QObject::connect(shutdown_button, &QPushButton::clicked, 
                     this, &FlighInfoDisplay::shutdownPi);  // Connect Shutdown Button                 

    setLayout(grid);
    subcribe2topics();
    checkHeartbeat();
}

void FlighInfoDisplay::on_click_refresheButton() {
    add_namespaces_to_combobox();
    auto names_and_namespaces = rviz_ros_node_.lock()->get_raw_node()->get_node_names();
    std::set<std::string> namespaces = get_namespaces(names_and_namespaces);

    if (!namespaces.empty()) {
        on_changed_namespace(QString((*namespaces.begin()).c_str()));
    }
}

void FlighInfoDisplay::shutdownPi() {
    QProcess* ssh_process = new QProcess(this);

    // Attempt with the first set of credentials
    QString command1 = "/usr/bin/sshpass -p 'raspberry' ssh pi@192.168.2.2 'sudo /sbin/shutdown -h now'";
    ssh_process->start("bash", {"-c", command1});
    
    connect(ssh_process, 
        static_cast<void (QProcess::*)(int, QProcess::ExitStatus)>(&QProcess::finished),
        this, [this, ssh_process](int exitCode, QProcess::ExitStatus) {
            if (exitCode != 0) {
                // If the first attempt fails, try the second set of credentials
                QProcess* ssh_process2 = new QProcess(this);
                QString command2 = "/usr/bin/sshpass -p 'deepwater' ssh dwe@192.168.2.2 'sudo /sbin/shutdown -h now'";
                ssh_process2->start("bash", {"-c", command2});
                
                connect(ssh_process2, 
                    static_cast<void (QProcess::*)(int, QProcess::ExitStatus)>(&QProcess::finished),
                    this, [this, ssh_process2](int exitCode2, QProcess::ExitStatus) {
                        QString result = (exitCode2 == 0) ? "Shutdown command sent successfully." : "shutdown ongoing.";
                        qDebug() << result;
                        ssh_process2->deleteLater();  // Clean up
                    }
                );
            } else {
                qDebug() << "Shutdown command sent successfully.";
            }
            ssh_process->deleteLater();  // Clean up
        }
    );
}


void FlighInfoDisplay::add_namespaces_to_combobox() {
    auto names_and_namespaces = rviz_ros_node_.lock()->get_raw_node()->get_node_names();
    std::set<std::string> namespaces = get_namespaces(names_and_namespaces);

    namespace_->blockSignals(true);
    namespace_->clear();
    for (const auto& n : namespaces) {
        namespace_->addItem(QString::fromStdString(n));
    }
    namespace_->blockSignals(false);
}

void FlighInfoDisplay::on_changed_namespace(const QString& text) {
    std::string namespace_str = text.toUtf8().constData();

    attitude_topic_name_ = "/imu";
    odometry_topic_name_ = "/depth";
    ping_topic_name_ = "/ping";
    vehicle_attitude_sub_.reset();
    vehicle_odometry_sub_.reset();

    subcribe2topics();
}

void FlighInfoDisplay::subcribe2topics() {
    vehicle_attitude_sub_ = rviz_ros_node_.lock()->get_raw_node()->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
            geometry_msgs::msg::Quaternion q = msg->orientation;
            double yaw, pitch, roll;
            tf2::getEulerYPR(q, yaw, pitch, roll);

            compass_widget_->setAngle(yaw * 180 / M_PI);
            compass_widget_->update();
            adi_widget_->setPitch(pitch * 180 / M_PI);
            adi_widget_->setRoll(roll * 180 / M_PI);
            adi_widget_->update();
        });

    vehicle_odometry_sub_ = rviz_ros_node_.lock()->get_raw_node()->create_subscription<atl_msgs::msg::Depth>(
        "/depth", 10, [this](const atl_msgs::msg::Depth::SharedPtr msg) {
            vi_widget_->setGroundSpeed(msg->temperature);
            vi_widget_->setAlt(msg->depth);
            vi_widget_->update();
        });

    ping_distance_sub_ = rviz_ros_node_.lock()->get_raw_node()->create_subscription<atl_msgs::msg::PingData>(
    "/ping", 10, [this](const atl_msgs::msg::PingData::SharedPtr msg) {
        // Use both distance and confidence from the message
        float ping_distance = msg->distance;
        float ping_confidence = msg->confidence;

        // Update the widget with the new values
        vi_widget_->setPingInfo(ping_distance, ping_confidence);
        vi_widget_->update();

    
}); 

 servos_input_sub_ = rviz_ros_node_.lock()->get_raw_node()->create_subscription<atl_msgs::msg::ServosInput>(
    "/servos_input", 10, [this](const atl_msgs::msg::ServosInput::SharedPtr msg) {
        // Ensure there are at least 5 inputs to avoid out-of-bounds errors
        if (msg->inputs.size() < 5) {
            RCLCPP_WARN(rviz_ros_node_.lock()->get_raw_node()->get_logger(), 
                        "Received ServosInput message with insufficient inputs.");
            return;
        }

        // Latch state: static to retain the state across messages
        static bool roll_controller_active = false;  // Default: off

        // Logic: Activate if inputs[3].delta == 1, deactivate if inputs[4].delta == 1
        if (msg->inputs[3].delta == 1.0) {
            roll_controller_active = false;
        }
        if (msg->inputs[4].delta == 1.0) {
            roll_controller_active = true;
        }

        // Update the roll controller status in the widget
        vi_widget_->setRollStatus(roll_controller_active);
        vi_widget_->update();
    });

RCLCPP_INFO(rviz_ros_node_.lock()->get_raw_node()->get_logger(), 
                "FlighInfoDisplay: Subscribed to topics.");

}


}  // namespace displays
}  // namespace rviz_aerial_plugins
                
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_aerial_plugins::displays::FlighInfoDisplay, rviz_common::Panel)
