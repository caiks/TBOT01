// https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/ros2-devel/turtlebot3_gazebo/include/turtlebot3_gazebo/turtlebot3_drive.hpp

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "dev.h"

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
#define TB3_RIGHT_TURN	2
#define TB3_LEFT_TURN	 3

typedef std::pair<std::string, std::string> StringPair;
typedef std::vector<std::string> StringList;

typedef std::pair<double, double> Coord;

class Controller : public rclcpp::Node
{
public:
	Controller(const std::string&, std::chrono::milliseconds, std::chrono::milliseconds, std::chrono::milliseconds, std::chrono::milliseconds, const std::string&, const std::string&);
	~Controller();

private:
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_pub;

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _scan_sub;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;

	rclcpp::TimerBase::SharedPtr _update_timer;
	rclcpp::TimerBase::SharedPtr _record_timer;

	double _robot_pose;
	double _prev_robot_pose;
	double _scan_data[3];

	TBOT01::Record _record;
	bool _pose_updated;
	bool _scan_updated;
	bool _action_updated;

	std::chrono::time_point<std::chrono::high_resolution_clock> _record_start;

	std::ofstream _record_out;

	int _left_turn_factor;
	int _right_turn_factor;

	int _demo_factor;
	std::string _demo_state0;
	std::string _demo_state1;
	std::map<StringPair, StringList> _demo_map;
	std::map<std::string, std::string> _demo_map2;
	std::map<std::string, Coord> _demo_coord_map;

	void update_callback();
	void record_callback();
	void update_cmd_vel(double linear, double angular);
	void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};
#endif // CONTROLLER_H
