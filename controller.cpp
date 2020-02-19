// https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/ros2-devel/turtlebot3_gazebo/src

#include "controller.h"

#include <stdlib.h>
#include <cmath>

using namespace Alignment;
using namespace TBOT01;
using namespace std;
using namespace std::chrono_literals;

typedef std::chrono::duration<double> sec;
typedef std::chrono::high_resolution_clock clk;

#define ECHO(x) cout << #x << endl; x
#define EVAL(x) cout << #x << ": " << (x) << endl
#define EVALL(x) cout << #x << ": " << endl << (x) << endl
#define TRUTH(x) cout << #x << ": " << ((x) ? "true" : "false") << endl

Controller::Controller(const std::string& filename, std::chrono::milliseconds record_interval, std::chrono::milliseconds left_turn_interval, std::chrono::milliseconds right_turn_interval, std::chrono::milliseconds demo_interval, const std::string& demo_state0, const std::string& demo_state1)
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

    auto tenms = 10ms;

    _left_turn_factor = left_turn_interval.count() / tenms.count();
    _right_turn_factor = right_turn_interval.count() / tenms.count();

    _demo_factor = demo_interval.count() / tenms.count();
    _demo_state0 = demo_state0;
    _demo_state1 = demo_state1;

    _demo_map[StringPair("room1", "door12")] = StringList{ "door13" , "door14" };
    _demo_map[StringPair("room1", "door13")] = StringList{ "door12" , "door14" };
    _demo_map[StringPair("room1", "door14")] = StringList{ "door13" , "door12" };
    _demo_map[StringPair("room2", "door12")] = StringList{ "door12" };
    _demo_map[StringPair("room3", "door13")] = StringList{ "door13" };
    _demo_map[StringPair("room4", "door14")] = StringList{ "door45" };
    _demo_map[StringPair("room4", "door45")] = StringList{ "door14" };
    _demo_map[StringPair("room5", "door45")] = StringList{ "door56" };
    _demo_map[StringPair("room5", "door56")] = StringList{ "door45" };
    _demo_map[StringPair("room6", "door56")] = StringList{ "door56" };
    _demo_map[StringPair("door12", "room1")] = StringList{ "room2" };
    _demo_map[StringPair("door12", "room2")] = StringList{ "room1" };
    _demo_map[StringPair("door13", "room1")] = StringList{ "room3" };
    _demo_map[StringPair("door13", "room3")] = StringList{ "room1" };
    _demo_map[StringPair("door14", "room1")] = StringList{ "room4" };
    _demo_map[StringPair("door14", "room4")] = StringList{ "room1" };
    _demo_map[StringPair("door45", "room5")] = StringList{ "room4" };
    _demo_map[StringPair("door45", "room4")] = StringList{ "room5" };
    _demo_map[StringPair("door56", "room5")] = StringList{ "room6" };
    _demo_map[StringPair("door56", "room6")] = StringList{ "room5" };

    _demo_map2["room1"] = "door14";
    _demo_map2["room2"] = "door12";
    _demo_map2["room3"] = "door13";
    _demo_map2["room4"] = "door45";
    _demo_map2["room5"] = "door56";
    _demo_map2["room6"] = "door56";
    _demo_map2["door12"] = "room1";
    _demo_map2["door13"] = "room3";
    _demo_map2["door14"] = "room4";
    _demo_map2["door45"] = "room5";
    _demo_map2["door56"] = "room6";

    _demo_coord_map["door12"] = Coord(6.2, -0.175);
    _demo_coord_map["door13"] = Coord(2.3, 4.5);
    _demo_coord_map["door14"] = Coord(2.3, 0.375);
    _demo_coord_map["door45"] = Coord(-5.15, 3.1);
    _demo_coord_map["door56"] = Coord(-6.325, 0.925);
    _demo_coord_map["room1"] = Coord((2.3 + 7.5) / 2.0, (-0.175 + 5.27) / 2.0);
    _demo_coord_map["room2"] = Coord((4.9 + 7.5) / 2.0, (-5.275 - 0.175) / 2.0);
    _demo_coord_map["room3"] = Coord((-0.05 + 2.3) / 2.0, (0.925 + 5.27) / 2.0);
    _demo_coord_map["room4"] = Coord((-5.15 + 2.3) / 2.0, (-0.175 + 5.27) / 2.0);
    _demo_coord_map["room5"] = Coord((-7.5 - 5.15) / 2.0, (0.925 + 5.27) / 2.0);
    _demo_coord_map["room6"] = Coord((-7.5 - 5.15) / 2.0, (-3.925 + 0.925) / 2.0);

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

    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double x1 = _demo_coord_map[_demo_state0].first;
    double y1 = _demo_coord_map[_demo_state0].second;
    if ((x1 - x)*(x1 - x) + (y1 - y)*(y1 - y) <= 0.75*0.75)
    {
	if (_demo_map.find(StringPair(_demo_state0, _demo_state1)) != _demo_map.end())
	{
	    StringList demo_strings = _demo_map[StringPair(_demo_state0, _demo_state1)];
	    int demo_index = 0;
	    if (demo_strings.size() > 1)
		demo_index = rand() % demo_strings.size();
	    std::string demo_state0 = demo_strings[demo_index];
	    _demo_state1 = _demo_state0;
	    _demo_state0 = demo_state0;
	    cout << "demo transition" << endl;
	    EVAL(_demo_state1);
	    EVAL(_demo_state0);
	}
	else
	{
	    std::string demo_state0 = _demo_map2[_demo_state0];
	    _demo_state1 = _demo_state0;
	    _demo_state0 = demo_state0;
	    cout << "demo transition 2" << endl;
	    EVAL(_demo_state1);
	    EVAL(_demo_state0);
	}
    }
}

void Controller::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    uint16_t scan_angle[3] = {0, 30, 330};

    for (int num = 0; num < 3; num++)
    {
	if (std::isinf(msg->ranges.at(scan_angle[num])))
	    _scan_data[num] = msg->range_max;
	else
	    _scan_data[num] = msg->ranges.at(scan_angle[num]);
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
	if (_left_turn_factor > 0 && (rand() % _left_turn_factor) == 0)
	{
            _prev_robot_pose = _robot_pose;
	    turtlebot3_state_num = TB3_LEFT_TURN;
	    break;
	}
	else if (_right_turn_factor > 0 && (rand() % _right_turn_factor) == 0)
	{
	    _prev_robot_pose = _robot_pose;
	    turtlebot3_state_num = TB3_RIGHT_TURN;
	    break;
	}
	else if (_pose_updated && _demo_factor > 0 && (_demo_factor == 1 || (rand() % _demo_factor) == 0))
	{
	    double x1 = _demo_coord_map[_demo_state0].first;
	    double y1 = _demo_coord_map[_demo_state0].second;
	    double x = _record.sensor_pose[0];
	    double y = _record.sensor_pose[1];
	    double l = std::sqrt((x1 - x)*(x1 - x) + (y1 - y)*(y1 - y));
	    double yaw1 = std::acos((x1 - x) / l) * (y1 >= y ? 1.0 : -1.0);
	    double d = yaw1 - _robot_pose;
	    if (d < -M_PI)
		d += 2.0 * M_PI;
	    else if (d > M_PI)
		d -= 2.0 * M_PI;
	    if (d > 20.0 * DEG2RAD)
	    {
		_prev_robot_pose = _robot_pose;
		turtlebot3_state_num = TB3_LEFT_TURN;
		cout << "demo left turn" << endl;
		break;
	    }
	    else if (d < -20.0 * DEG2RAD)
	    {
		_prev_robot_pose = _robot_pose;
		turtlebot3_state_num = TB3_RIGHT_TURN;
		cout << "demo right turn" << endl;
		break;
	    }
	}
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


int main(int argc, char** argv)
{
    std::string filename(argc >= 2 ? std::string(argv[1]) : "TBOT01.bin");
    std::chrono::milliseconds record_interval(argc >= 3 ? std::atol(argv[2]) : 250);
    std::chrono::milliseconds left_turn_interval(argc >= 4 ? std::atol(argv[3]) : 0);
    std::chrono::milliseconds right_turn_interval(argc >= 5 ? std::atol(argv[4]) : 0);
    std::chrono::milliseconds demo_interval(argc >= 6 ? std::atol(argv[5]) : 0);
    std::string demo_state0(argc >= 7 ? std::string(argv[6]) : "room4");
    std::string demo_state1(argc >= 8 ? std::string(argv[7]) : "door14");

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>(filename, record_interval, left_turn_interval, right_turn_interval, demo_interval, demo_state0, demo_state1));
    rclcpp::shutdown();

    return 0;
}

