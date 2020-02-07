// https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/ros2-devel/turtlebot3_gazebo/src

#include "controller.h"

using namespace Alignment;
using namespace TBOT01;
using namespace std;
using namespace std::chrono_literals;

typedef std::chrono::duration<double> sec;
typedef std::chrono::high_resolution_clock clk;

Controller::Controller(const std::string& filename, std::chrono::milliseconds record_interval)
: Node("TBOT01_controller_node")
{
  _scan_data[0] = 0.0;
  _scan_data[1] = 0.0;
  _scan_data[2] = 0.0;

  _robot_pose = 0.0;
  _prev_robot_pose = 0.0;

  _pose_updated = false;
  _scan_updated = false;
  _action_updated = false;

  _record_start = clk::now();
  _record_out = std::ofstream(filename, std::ios::binary);

  _cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(rclcpp::KeepLast(10)));

  _scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(), std::bind(&Controller::scan_callback, this, std::placeholders::_1));
  _odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::QoS(rclcpp::KeepLast(10)), std::bind(&Controller::odom_callback, this, std::placeholders::_1));

  _update_timer = this->create_wall_timer(10ms, std::bind(&Controller::update_callback, this));
  _record_timer = this->create_wall_timer(record_interval, std::bind(&Controller::record_callback, this));

  RCLCPP_INFO(this->get_logger(), "TBOT01 controller node has been initialised");
}

Controller::~Controller()
{
  _record_out.close();
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
  _robot_pose = yaw;

  _record.sensor_pose[0] = msg->pose.pose.position.x;
  _record.sensor_pose[1] = msg->pose.pose.position.y;
  _record.sensor_pose[2] = msg->pose.pose.position.z;
  _record.sensor_pose[3] = msg->pose.pose.orientation.x;
  _record.sensor_pose[4] = msg->pose.pose.orientation.y;
  _record.sensor_pose[5] = msg->pose.pose.orientation.z;
  _record.sensor_pose[6] = msg->pose.pose.orientation.w;
  _pose_updated = true;
}

void Controller::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  uint16_t scan_angle[3] = {0, 30, 330};

  for (int num = 0; num < 3; num++)
  {
    if (std::isinf(msg->ranges.at(scan_angle[num])))
    {
      _scan_data[num] = msg->range_max;
    }
    else
    {
      _scan_data[num] = msg->ranges.at(scan_angle[num]);
    }
  }

  for (std::size_t i = 0; i < 360; i++)
  {
      if (std::isinf(msg->ranges.at(i)))
	  _record.sensor_scan[i] = msg->range_max;
      else
	  _record.sensor_scan[i] = msg->ranges.at(i);
  }
  _scan_updated = true;
}

void Controller::update_cmd_vel(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  _cmd_vel_pub->publish(cmd_vel);

  _record.action_linear = cmd_vel.linear.x;
  _record.action_angular = cmd_vel.angular.z;
  _action_updated = true;
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
      if (_scan_data[CENTER] > check_forward_dist)
      {
        if (_scan_data[LEFT] < check_side_dist)
        {
          _prev_robot_pose = _robot_pose;
          turtlebot3_state_num = TB3_RIGHT_TURN;
        }
        else if (_scan_data[RIGHT] < check_side_dist)
        {
          _prev_robot_pose = _robot_pose;
          turtlebot3_state_num = TB3_LEFT_TURN;
        }
        else
        {
          turtlebot3_state_num = TB3_DRIVE_FORWARD;
        }
      }

      if (_scan_data[CENTER] < check_forward_dist)
      {
        _prev_robot_pose = _robot_pose;
        turtlebot3_state_num = TB3_RIGHT_TURN;
      }
      break;

    case TB3_DRIVE_FORWARD:
      update_cmd_vel(LINEAR_VELOCITY, 0.0);
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;

    case TB3_RIGHT_TURN:
      if (fabs(_prev_robot_pose - _robot_pose) >= escape_range)
        turtlebot3_state_num = GET_TB3_DIRECTION;
      else
        update_cmd_vel(0.0, -1 * ANGULAR_VELOCITY);
      break;

    case TB3_LEFT_TURN:
      if (fabs(_prev_robot_pose - _robot_pose) >= escape_range)
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
    if (_pose_updated && _scan_updated && _action_updated)
    {
	_record.ts = ((sec)(clk::now() - _record_start)).count();
	recordsPersistent(_record, _record_out);
	_record.id++;
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

