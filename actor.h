#ifndef OBSERVER_H
#define OBSERVER_H

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "dev.h"

class Actor : public rclcpp::Node
{
public:
	Actor(const std::string&, const std::string&, std::chrono::milliseconds, const std::string&);
	~Actor();

private:
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _scan_sub;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;

	rclcpp::TimerBase::SharedPtr _observe_timer;

	TBOT01::Record _record;
	bool _pose_updated;
	bool _scan_updated;
	
	string _room;

	std::unique_ptr<Alignment::System> _uu;
	std::unique_ptr<Alignment::SystemRepa> _ur;
	std::unique_ptr<Alignment::ApplicationRepa> _dr;
	std::map<std::size_t, std::shared_ptr<Alignment::History>> _shr;

	void observe_callback();
	void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};
#endif // OBSERVER_H
