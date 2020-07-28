﻿#ifndef OBSERVER_H
#define OBSERVER_H

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "dev.h"

typedef std::tuple<std::string, std::string, std::string> String3;	
typedef std::vector<String3> String3List;	

class Actor : public rclcpp::Node
{
public:
	Actor(const std::string&, const std::string&, std::chrono::milliseconds, const std::string&);
	~Actor();

private:
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _scan_sub;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;

	rclcpp::TimerBase::SharedPtr _act_timer;

	TBOT01::Record _record;
	bool _pose_updated;
	bool _scan_updated;
	
	std::string _room;

	std::shared_ptr<Alignment::System> _uu;
	std::shared_ptr<Alignment::SystemRepa> _ur;
	std::shared_ptr<Alignment::ApplicationRepa> _dr;
	std::map<std::size_t, std::shared_ptr<Alignment::HistoryRepa>> _slice_history;
	String3List _room_locaction_goal;

	void act_callback();
	void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};
#endif // OBSERVER_H
