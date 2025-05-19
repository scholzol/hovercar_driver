#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <cmath>
#include "protocol.h"
#include <iostream>

// Global parameters
std::string port_;
double wheel_base_;
double max_steer_angle_;
int baudrate_;
int serial_fd_ = -1;
geometry_msgs::Twist cmdVel;
/*
// Serial protocol struct (ensure no padding)
struct __attribute__((packed)) SerialCommand {
    uint16_t start;
    int16_t steer;
    int16_t speed;
    uint16_t checksum;
};
*/
// Helper to map baudrate integer to termios constant
speed_t getBaudrate(int baudrate) {
    switch (baudrate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        default: return B115200; // Default fallback
    }
}

void openSerialPort() {
    serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ < 0) {
        ROS_ERROR_STREAM("Failed to open " << port_);
        // ros::shutdown();
        return;
    }

    struct termios tty;
    // memset(&tty, 0, sizeof tty);

    if (tcgetattr(serial_fd_, &tty) != 0) {
        ROS_ERROR("Error from tcgetattr");
        // ros::shutdown();
        return;
    }

    // speed_t baud = getBaudrate(baudrate_);
    // cfsetospeed(&tty, baud);
    // cfsetispeed(&tty, baud);

    tty.c_cflag = B115200 | CS8 | CLOCAL | CREAD;    // 8-bit chars
    tty.c_iflag = IGNPAR;                // disable break processing
    tty.c_lflag = 0;                       // no signaling chars, no echo
    tty.c_oflag = 0;                       // no remapping, no delays
    // tty.c_cc[VMIN]  = 0;                   // read doesn't block
    // tty.c_cc[VTIME] = 5;                   // 0.5 seconds read timeout

    tcflush(serial_fd_, TCIFLUSH);

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        ROS_ERROR("Error from tcsetattr");
        // ros::shutdown();
        return;
    }

    ROS_INFO_STREAM("Serial port " << port_ << " opened at baudrate " << baudrate_);
}

void twistCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    // ROS_WARN("Callback triggered");
    float speed = msg->linear.x * 100; // Scaling meters/sec to cm/sec
    float steer = 0.0f;

    cmdVel.linear.x = msg->linear.x;

    if (std::abs(speed) > 1e-4) {
        steer = std::atan(wheel_base_ * msg->angular.z / speed) * 180.0f / M_PI;
    }
    steer = std::max(std::min(steer, (float)max_steer_angle_), -(float)max_steer_angle_);

    ROS_INFO_STREAM("Sending: speed = " << speed << " steer = " << steer);

    SerialCommand command;
    command.start = (uint16_t)START_FRAME;
    command.steer = (int16_t)steer;
    command.speed = (int16_t)speed;
    command.checksum = (uint16_t)command.start ^ command.steer ^ command.speed;

    int rc = ::write(serial_fd_, (const void*)&command, sizeof(command));
    if (rc < 0) {
        ROS_ERROR("Error writing to hoverboard serial port");
    }
}
// Function to close the serial port
void closeSerialPort(int fd) { close(fd); }

int main(int argc, char** argv) {
    ros::init(argc, argv, "cmd_vel_to_serial");
    ros::NodeHandle nh;

    nh.param<std::string>("port", port_, "/dev/ttyACM0");
    nh.param<int>("baudrate", baudrate_, 115200);
    nh.param<double>("wheel_base", wheel_base_, 0.6);
    nh.param<double>("max_steer_angle", max_steer_angle_, 30.0);

    openSerialPort();

    ros::Subscriber sub = nh.subscribe("cmd_vel", 10, twistCallback);

    ros::Rate rate(10); // Loop at 10Hz
    ros::Time prev_time = ros::Time::now();

    ROS_INFO_STREAM("ROS Node started, listening on /cmd_vel");

    while (ros::ok()) {
        ros::spinOnce();
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;
        ROS_DEBUG_STREAM("Time: " << time << " cmdVel.linear.x: " << cmdVel.linear.x);
        prev_time = time;
        rate.sleep();
    }
    closeSerialPort(serial_fd_);
    return 0;
}
