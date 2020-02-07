//#include "dev.h"

//using namespace Alignment;

// https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/ros2-devel/turtlebot3_gazebo/src

#include "controller.h"
#include <iostream>
#include <sstream>

using namespace std::chrono_literals;

typedef std::chrono::duration<double> sec;
typedef std::chrono::high_resolution_clock clk;

Controller::Controller(const std::string& filename, std::chrono::milliseconds record_interval)
: Node("TBOT01_controller_node")
{
  /************************************************************
  ** Initialise variables
  ************************************************************/
  scan_data_[0] = 0.0;
  scan_data_[1] = 0.0;
  scan_data_[2] = 0.0;

  robot_pose_ = 0.0;
  prev_robot_pose_ = 0.0;

  for (std::size_t i = 0; i < 7; i++)
      sensor_pose_[i] = 0.0;

  sensor_pose_updated = false;

  for (std::size_t i = 0; i < 360; i++)
      sensor_scan_[i] = 0.0;

  sensor_scan_updated = false;

  action_linear_ = 0.0;
  action_angular_ = 0.0;

  action_updated = false;

  record_id_ = 0;
  record_start_ = clk::now();
  record_out_ = std::ofstream(filename, std::ios::binary);

  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

  // Initialise subscribers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(), std::bind(&Controller::scan_callback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", qos, std::bind(&Controller::odom_callback, this, std::placeholders::_1));

  /************************************************************
  ** Initialise ROS timers
  ************************************************************/
  update_timer_ = this->create_wall_timer(10ms, std::bind(&Controller::update_callback, this));
  record_timer_ = this->create_wall_timer(record_interval, std::bind(&Controller::record_callback, this));

  RCLCPP_INFO(this->get_logger(), "TBOT01 controller node has been initialised");
}

Controller::~Controller()
{
  record_out_.close();
  RCLCPP_INFO(this->get_logger(), "TBOT01 controller node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
void Controller::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  robot_pose_ = yaw;

  sensor_pose_[0] = msg->pose.pose.position.x;
  sensor_pose_[1] = msg->pose.pose.position.y;
  sensor_pose_[2] = msg->pose.pose.position.z;
  sensor_pose_[3] = msg->pose.pose.orientation.x;
  sensor_pose_[4] = msg->pose.pose.orientation.y;
  sensor_pose_[5] = msg->pose.pose.orientation.z;
  sensor_pose_[6] = msg->pose.pose.orientation.w;
  sensor_pose_updated = true;
}

void Controller::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  uint16_t scan_angle[3] = {0, 30, 330};

  for (int num = 0; num < 3; num++)
  {
    if (std::isinf(msg->ranges.at(scan_angle[num])))
    {
      scan_data_[num] = msg->range_max;
    }
    else
    {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }

  for (std::size_t i = 0; i < 360; i++)
  {
      if (std::isinf(msg->ranges.at(i)))
	  sensor_scan_[i] = msg->range_max;
      else
	  sensor_scan_[i] = msg->ranges.at(i);
  }
  sensor_scan_updated = true;
}

void Controller::update_cmd_vel(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_->publish(cmd_vel);

  action_linear_ = cmd_vel.linear.x;
  action_angular_ = cmd_vel.angular.z;
  action_updated = true;
}

/********************************************************************************
** Update functions
********************************************************************************/
void Controller::update_callback()
{
  static uint8_t turtlebot3_state_num = 0;
  double escape_range = 30.0 * DEG2RAD;
  double check_forward_dist = 0.7;
  double check_side_dist = 0.6;

  switch (turtlebot3_state_num)
  {
    case GET_TB3_DIRECTION:
      if (scan_data_[CENTER] > check_forward_dist)
      {
        if (scan_data_[LEFT] < check_side_dist)
        {
          prev_robot_pose_ = robot_pose_;
          turtlebot3_state_num = TB3_RIGHT_TURN;
        }
        else if (scan_data_[RIGHT] < check_side_dist)
        {
          prev_robot_pose_ = robot_pose_;
          turtlebot3_state_num = TB3_LEFT_TURN;
        }
        else
        {
          turtlebot3_state_num = TB3_DRIVE_FORWARD;
        }
      }

      if (scan_data_[CENTER] < check_forward_dist)
      {
        prev_robot_pose_ = robot_pose_;
        turtlebot3_state_num = TB3_RIGHT_TURN;
      }
      break;

    case TB3_DRIVE_FORWARD:
      update_cmd_vel(LINEAR_VELOCITY, 0.0);
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;

    case TB3_RIGHT_TURN:
      if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range)
        turtlebot3_state_num = GET_TB3_DIRECTION;
      else
        update_cmd_vel(0.0, -1 * ANGULAR_VELOCITY);
      break;

    case TB3_LEFT_TURN:
      if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range)
        turtlebot3_state_num = GET_TB3_DIRECTION;
      else
        update_cmd_vel(0.0, ANGULAR_VELOCITY);
      break;

    default:
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;
  }
}

void Controller::record_callback()
{
    if (sensor_pose_updated && sensor_scan_updated && action_updated)
    {
	double ts = ((sec)(clk::now() - record_start_)).count();
	record_out_.write(reinterpret_cast<char*>(&record_id_), sizeof(std::size_t));
	record_out_.write(reinterpret_cast<char*>(&ts), sizeof(double));
	for (std::size_t i = 0; i < 7; i++)
	    record_out_.write(reinterpret_cast<char*>(&sensor_pose_[i]), sizeof(double));
	for (std::size_t i = 0; i < 360; i++)
	    record_out_.write(reinterpret_cast<char*>(&sensor_scan_[i]), sizeof(double));
	record_out_.write(reinterpret_cast<char*>(&action_linear_), sizeof(double));
	record_out_.write(reinterpret_cast<char*>(&action_angular_), sizeof(double));
	record_id_++;
    }
}


/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char** argv)
{
    std::string filename(argc >= 2 ? std::string(argv[1]) : "TBOT01.bin");
    std::chrono::milliseconds record_interval(argc >= 3 ? std::atol(argv[2]) : 250);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>(filename, record_interval));
    rclcpp::shutdown();

    return 0;
}

