#include "observer.h"

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

Observer::Observer(const std::string& model, std::chrono::milliseconds observe_interval)
: Node("TBOT01_observer_node")
{
    _pose_updated = false;
    _scan_updated = false;
    _matches = 0;
    _observations = 0;

    _label = "location";
    if (model == "model006_position" || model == "model007_position")
	_label = "position";

    std::unique_ptr<Alignment::SystemRepa> ur;
    std::vector<std::string> files{
	"202001271320_room1.TBOT01.hr",
	"202001271320_room2.TBOT01.hr",
	"202001271320_room2_2.TBOT01.hr",
	"202001271320_room3.TBOT01.hr",
	"202001271320_room4.TBOT01.hr",
	"202001271320_room5.TBOT01.hr",
	"202001271320_room5_2.TBOT01.hr"
    };
    HistoryRepaPtrList ll;
    for (auto& f : files)
    {
	std::ifstream in(f, std::ios::binary);
	auto qq = persistentsRecordList(in);
	in.close();
	auto xx = recordListsHistoryRepa_2(8, *qq);
	ur = std::move(std::get<1>(xx));
	ll.push_back(std::move(std::get<2>(xx)));
    }
    auto hr = vectorHistoryRepasConcat_u(ll);

    auto& vvi = ur->mapVarSize();

    StrVarPtrMap m;
    std::ifstream in(model + ".dr", std::ios::binary);
    auto ur1 = persistentsSystemRepa(in, m);
    _dr = persistentsApplicationRepa(in);
    in.close();

    auto frmul = historyRepasFudRepasMultiply_u;

    auto hr1 = frmul(*hr, *_dr->fud);

    if (hr1->evient)
	hr1->transpose();
    auto z = hr1->size;
    auto& mvv = hr1->mapVarInt();
    auto sh = hr1->shape;
    auto rr = hr1->arr;
    _pl = mvv[vvi[Variable(_label)]];
    auto sl = sh[_pl];
    auto nn = treesLeafNodes(*_dr->slices);
    SizeList al(sl);
    for (auto& s : *nn)
    {
	for (std::size_t k = 0; k < sl; k++)
	    al[k] = 0;
	auto pk = mvv[s.first];
	for (std::size_t j = 0; j < z; j++)
	{
	    std::size_t u = rr[pk*z + j];
	    if (u)
	    {
		std::size_t w = rr[_pl*z + j];
		al[w]++;
	    }
	}
	std::size_t c = 0;
	std::size_t cl = sl;
	for (std::size_t k = 0; k < sl; k++)
	{
	    auto u = al[k];
	    if (u > c)
	    {
		c = u;
		cl = k;
	    }
	}
	_sp[s.first] = cl;
    }

    _scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
	"scan", rclcpp::SensorDataQoS(), std::bind(&Observer::scan_callback, this, std::placeholders::_1));
    _odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
	"odom", rclcpp::QoS(rclcpp::KeepLast(10)), std::bind(&Observer::odom_callback, this, std::placeholders::_1));

    _observe_timer = this->create_wall_timer(observe_interval, std::bind(&Observer::observe_callback, this));

    RCLCPP_INFO(this->get_logger(), "TBOT01 observer node has been initialised");
}

Observer::~Observer()
{
    RCLCPP_INFO(this->get_logger(), "TBOT01 observer node has been terminated");
}

void Observer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    _record.sensor_pose[0] = msg->pose.pose.position.x;
    _record.sensor_pose[1] = msg->pose.pose.position.y;
    _record.sensor_pose[2] = msg->pose.pose.position.z;
    _record.sensor_pose[3] = msg->pose.pose.orientation.x;
    _record.sensor_pose[4] = msg->pose.pose.orientation.y;
    _record.sensor_pose[5] = msg->pose.pose.orientation.z;
    _record.sensor_pose[6] = msg->pose.pose.orientation.w;
    _pose_updated = true;
}

void Observer::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    for (std::size_t i = 0; i < 360; i++)
    {
	if (std::isinf(msg->ranges.at(i)))
	    _record.sensor_scan[i] = msg->range_max;
	else
	    _record.sensor_scan[i] = msg->ranges.at(i);
    }
    _scan_updated = true;
}

void Observer::observe_callback()
{
    auto frmul = historyRepasFudRepasMultiply_u;
    auto hrhrred = setVarsHistoryRepasHistoryRepaReduced_u;

    if (_pose_updated && _scan_updated)
    {
	auto xx = recordListsHistoryRepa_2(8, RecordList{ _record });
	auto uu = std::move(std::get<0>(xx));
	auto ur = std::move(std::get<1>(xx));
	auto hr = std::move(std::get<2>(xx));
	SizeList ww{ _pl };
	auto nn = treesLeafNodes(*_dr->slices);
	for (auto& s : *nn)
	    ww.push_back(s.first);
	auto hr1 = hrhrred(ww.size(), ww.data(), *frmul(*hr, *_dr->fud));
	auto n = hr1->dimension;
	auto vv = hr1->vectorVar;
	auto sh = hr1->shape;
	auto rr = hr1->arr;
	auto sl = sh[0];
	std::size_t cl = rr[0];
	std::size_t l = sl;
	for (std::size_t i = 1; i < n; i++)
	{
	    std::size_t u = rr[i];
	    if (u)
	    {
		l = _sp[vv[i]];
		break;
	    }
	}
	std::vector<std::string> locations{ "door12", "door13", "door14", "door45", "door56",
	    "room1", "room2", "room3", "room4", "room5", "room6", "unknown" };
	std::vector<std::string> positions{ "centre", "corner", "side", "unknown" };
	bool is_match = l == cl;
	if (is_match)
	    _matches++;
	_observations++;

	std::string report;
	report += _label == "location" ? locations[cl] : positions[cl];
	report += "\t";
	report += _label == "location" ? locations[l] : positions[l];
	report += "\t";
	report += l == cl ? "match" : "fail";
	report += "\t";
	report += std::to_string((double)_matches / (double)_observations * 100.0);

	RCLCPP_INFO(this->get_logger(), report);
    }
}

int main(int argc, char** argv)
{
    std::string model = string(argc >= 2 ? argv[1] : "model006_location");
    std::chrono::milliseconds observe_interval(argc >= 3 ? std::atol(argv[2]) : 5*60);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Observer>(model, observe_interval));
    rclcpp::shutdown();

    return 0;
}

