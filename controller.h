// https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/ros2-devel/turtlebot3_gazebo/include/turtlebot3_gazebo/turtlebot3_drive.hpp

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2

#define LINEAR_VELOCITY  0.3
#define ANGULAR_VELOCITY 1.5

#define GET_TB3_DIRECTION 0
#define TB3_DRIVE_FORWARD 1
#define TB3_RIGHT_TURN    2
#define TB3_LEFT_TURN     3

class Controller : public rclcpp::Node
{
public:
    Controller();
    ~Controller();

private:
    // ROS topic publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // ROS topic subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Variables
    double robot_pose_;
    double prev_robot_pose_;
    double scan_data_[3];

    // ROS timer
    rclcpp::TimerBase::SharedPtr update_timer_;

    // Function prototypes
    void update_callback();
    void update_cmd_vel(double linear, double angular);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};
#endif // CONTROLLER_H
