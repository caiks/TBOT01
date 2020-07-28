#include "actor.h"
#include <sstream>

using namespace Alignment;
using namespace TBOT01;
using namespace std;
using namespace std::chrono_literals;

#define EVAL(x) { std::ostringstream str; str << #x << ": " << (x); RCLCPP_INFO(this->get_logger(), str.str());}

typedef std::chrono::duration<double> sec;
typedef std::chrono::high_resolution_clock clk;

Actor::Actor(const std::string& model, const std::string& room_initial, std::chrono::milliseconds act_interval, const std::string& dataset)
: Node("TBOT01_actor_node")
{
	auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
	auto hrred = setVarsHistoryRepasReduce_u;
	auto hrhrred = setVarsHistoryRepasHistoryRepaReduced_u;
	auto frmul = historyRepasFudRepasMultiply_u;
		
	_pose_updated = false;
	_scan_updated = false;
	
	_room = room_initial;
	
	_room_locaction_goal = String3List{
		String3("room1","door12","room1"),
		String3("room1","door13","room1"),
		String3("room1","door14","room1"),
		String3("room1","door45","room4"),
		String3("room1","door56","room5"),
		String3("room1","room1","room1"),
		String3("room1","room2","room1"),
		String3("room1","room3","room1"),
		String3("room1","room4","room1"),
		String3("room1","room5","room4"),
		String3("room1","room6","room5"),
		String3("room2","door12","room2"),
		String3("room2","door13","room1"),
		String3("room2","door14","room1"),
		String3("room2","door45","room4"),
		String3("room2","door56","room5"),
		String3("room2","room1","room2"),
		String3("room2","room2","room2"),
		String3("room2","room3","room1"),
		String3("room2","room4","room1"),
		String3("room2","room5","room4"),
		String3("room2","room6","room5"),		
		String3("room3","door12","room1"),
		String3("room3","door13","room3"),
		String3("room3","door14","room1"),
		String3("room3","door45","room4"),
		String3("room3","door56","room5"),
		String3("room3","room1","room3"),
		String3("room3","room2","room1"),
		String3("room3","room3","room3"),
		String3("room3","room4","room1"),
		String3("room3","room5","room4"),
		String3("room3","room6","room5"),	
		String3("room4","door12","room1"),
		String3("room4","door13","room2"),
		String3("room4","door14","room4"),
		String3("room4","door45","room4"),
		String3("room4","door56","room5"),
		String3("room4","room1","room4"),
		String3("room4","room2","room1"),
		String3("room4","room3","room1"),
		String3("room4","room4","room4"),
		String3("room4","room5","room4"),
		String3("room4","room6","room5"),	
		String3("room5","door12","room1"),
		String3("room5","door13","room1"),
		String3("room5","door14","room4"),
		String3("room5","door45","room5"),
		String3("room5","door56","room5"),
		String3("room5","room1","room4"),
		String3("room5","room2","room1"),
		String3("room5","room3","room1"),
		String3("room5","room4","room5"),
		String3("room5","room5","room5"),
		String3("room5","room6","room5"),	
		String3("room6","door12","room1"),
		String3("room6","door13","room1"),
		String3("room6","door14","room4"),
		String3("room6","door45","room5"),
		String3("room6","door56","room6"),
		String3("room6","room1","room4"),
		String3("room6","room2","room1"),
		String3("room6","room3","room1"),
		String3("room6","room4","room5"),
		String3("room6","room5","room6"),
		String3("room6","room6","room6")
	};
		
	{	
		std::unique_ptr<Alignment::HistoryRepa> hr;
		{
			std::vector<std::string> files{
				"data002_room1.bin",
				"data002_room2.bin",
				"data002_room2_2.bin",
				"data002_room3.bin",
				"data002_room4.bin",
				"data002_room5.bin",
				"data002_room5_2.bin"
			};
			if (dataset == "data003")
			{
				files.clear();
				files.push_back("data003.bin");
			}
			else if (dataset == "data004")
			{
				files.clear();
				files.push_back("data003.bin");
				files.push_back("data004_01.bin");
				files.push_back("data004_02.bin");
				files.push_back("data004_03.bin");
				files.push_back("data004_04.bin");
				files.push_back("data004_05.bin");
			}
			else if (dataset != "data002")
			{
				files.clear();
				files.push_back(dataset+".bin");
			}	
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();			
				SystemHistoryRepaTuple xx;
				xx = recordListsHistoryRepa_4(8, *qq);			
				_uu = std::move(std::get<0>(xx));
				_ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}
		
		auto& llu = _ur->listVarSizePair;
		{
			std::unique_ptr<Alignment::SystemRepa> ur1;
			StrVarPtrMap m;
			std::ifstream in(model + ".dr", std::ios::binary);
			ur1 = persistentsSystemRepa(in, m);
			_dr = persistentsApplicationRepa(in);
			in.close();
			auto& llu1 = ur1->listVarSizePair;			
			SizeSizeUMap nn;
			for (auto& ll : _dr->fud->layers)
				for (auto& tr : ll)
				{
					auto x = tr->derived;
					auto& p = llu1[x];
					llu.push_back(VarSizePair(p.first, p.second));
					nn[x] = llu.size() - 1;
				}
			_dr->reframe_u(nn);
		}	
		VarSet vvl;
		vvl.insert(Variable("motor"));
		vvl.insert(Variable("location"));
		vvl.insert(Variable("room_next"));

		auto& vvi = _ur->mapVarSize();
		SizeList vvl1;
		for (auto& v : vvl)
			vvl1.push_back(vvi[v]);
			
		{
			auto hr1 = frmul(*hr, *_dr->fud);
			auto hr2 = hrhrred(vvl1.size(), vvl1.data(), *hr);
			if (hr1->evient)
				hr1->transpose();
			auto z = hr1->size;
			auto& mvv = hr1->mapVarInt();
			auto sh = hr1->shape;
			auto rr = hr1->arr;
			auto nn = treesLeafNodes(*_dr->slices);
			for (auto& s : *nn)
			{
				SizeList ev;
				auto pk = mvv[s.first];
				for (std::size_t j = 0; j < z; j++)
				{
					std::size_t u = rr[pk*z + j];
					if (u)
					{
						ev.push_back(j);
					}
				}
				if (ev.size() > 0)	
					_slice_history[s.first] = std::move(hrsel(ev.size(), ev.data(), *hr2));
			}
		}
	}

	_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
		"scan", rclcpp::SensorDataQoS(), std::bind(&Actor::scan_callback, this, std::placeholders::_1));
	_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom", rclcpp::QoS(rclcpp::KeepLast(10)), std::bind(&Actor::odom_callback, this, std::placeholders::_1));

	_act_timer = this->create_wall_timer(act_interval, std::bind(&Actor::act_callback, this));

	RCLCPP_INFO(this->get_logger(), "TBOT01 actor node has been initialised");
}

Actor::~Actor()
{
	RCLCPP_INFO(this->get_logger(), "TBOT01 actor node has been terminated");
}

void Actor::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
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

void Actor::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
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

void Actor::act_callback()
{
	auto state = [](const Variable& v, const Value& u)
	{
		return State(VarValPairList{VarValPair(v, u)});
	};
	auto single = histogramSingleton_u;		
	auto mul = pairHistogramsMultiply;
	auto size = [](const Histogram& aa)
	{
		return (double)histogramsSize(aa).getNumerator();
	};		
	auto trim = histogramsTrim;
	auto aall = histogramsList;
	auto smax = [](const Histogram& aa)
	{
		std::vector<std::pair<Rational,State>> ll;
		auto ll0 = *histogramsList(aa);
		for (auto p : ll0)
			ll.push_back(std::pair<Rational,State>(p.second,p.first));
		auto ll1 = sorted(ll);
		return ll1.back().second;
	};		
	auto ared = [](const Histogram& aa, const VarUSet& vv)
	{
		return setVarsHistogramsReduce(vv, aa);
	};		
	auto add = pairHistogramsAdd_u;
	auto hraa = [](const System& uu, const SystemRepa& ur, const HistoryRepa& hr)
	{
		return historiesHistogram(*systemsHistoryRepasHistory_u(uu,ur,hr));
	};
	auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
	auto hrhrred = setVarsHistoryRepasHistoryRepaReduced_u;
	auto hrred = setVarsHistoryRepasReduce_u;
	auto frmul = historyRepasFudRepasMultiply_u;
		
	if (!_pose_updated || !_scan_updated)
		return;

	Variable motor("motor");
	Variable location("location");
	Variable room_next("room_next");
	
	std::size_t s = 0;
	{
		auto xx = recordListsHistoryRepa_4(8, RecordList{ _record });
		auto hr = std::move(std::get<2>(xx));
		SizeList ww;
		auto nn = treesLeafNodes(*_dr->slices);
		for (auto& s : *nn)
			ww.push_back(s.first);
		auto hr1 = hrhrred(ww.size(), ww.data(), *frmul(*hr, *_dr->fud));
		auto n = hr1->dimension;
		auto vv = hr1->vectorVar;
		auto rr = hr1->arr;
		bool found = false;
		for (std::size_t i = 0; !found && i < n; i++)
		{
			std::size_t u = rr[i];
			if (u)
			{
				s = vv[i];
				found = true;
			}
		}	
		if (!found)
		{
			RCLCPP_INFO(this->get_logger(), "act_callback: error: no slice");
			return;
		}
	}
	
	if (_slice_history.find(s) == _slice_history.end())
	{
		RCLCPP_INFO(this->get_logger(), "act_callback: error: no slice history");
		return;
	}
		
	{
		VarSet vvl;
		vvl.insert(motor);
		vvl.insert(location);
		vvl.insert(room_next);

		auto& vvi = _ur->mapVarSize();
		auto& llu = _ur->listVarSizePair;
		SizeList vvl1;
		for (auto& v : vvl)
			vvl1.push_back(vvi[v]);
	
		EVAL(*llu[s].first);
		auto aa = *trim(*hraa(*_uu, *_ur, *_slice_history[s]));
		EVAL(size(aa))
		// EVAL(aa);
		auto ss = smax(*ared(aa,VarUSet{location}));		
		// EVAL(ss);
		Value location_value = ss.map_u().begin()->second;
		EVAL(_room);
		EVAL(location_value);
		Value next_room_value("");
		for (auto t : _room_locaction_goal)
			if (std::get<0>(t) == _room && Value(std::get<1>(t)) == location_value)
				next_room_value = Value(std::get<2>(t));
		if (next_room_value == Value(""))
		{
			RCLCPP_INFO(this->get_logger(), "act_callback: error: no next room");
			return;
		}
		EVAL(next_room_value);			
	}	

}

int main(int argc, char** argv)
{
	std::string model = string(argc >= 2 ? argv[1] : "model006_location");
	std::string room_initial = string(argc >= 3 ? argv[2] : "room1");
	std::chrono::milliseconds act_interval(argc >= 4 ? std::atol(argv[3]) : 5*60);
	string dataset = string(argc >= 5 ? argv[4] : "data002");

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Actor>(model, room_initial, act_interval, dataset));
	rclcpp::shutdown();

	return 0;
}

