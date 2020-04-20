#ifndef OBSERVER_H
#define OBSERVER_H

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "dev.h"

class Observer : public rclcpp::Node
{
public:
	Observer(const std::string&, const std::string&, std::chrono::milliseconds);
	~Observer();

private:
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _scan_sub;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;

	rclcpp::TimerBase::SharedPtr _observe_timer;

	TBOT01::Record _record;
	bool _pose_updated;
	bool _scan_updated;

	std::unique_ptr<Alignment::ApplicationRepa> _dr;
	std::map<std::size_t, std::size_t> _sp;
	std::size_t _pl;
	std::string _label;
	int _matches;
	int _observations;

	void observe_callback();
	void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};
#endif // OBSERVER_H
