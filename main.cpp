#include "dev.h"

using namespace Alignment;
using namespace TBOT01;
using namespace std;

typedef std::chrono::duration<double> sec; 
typedef std::chrono::high_resolution_clock clk;

#define ECHO(x) cout << #x << endl; x
#define EVAL(x) cout << #x << " = " << (x) << endl
#define TRUTH(x) cout << #x << " = " << ((x) ? "true" : "false") << endl

int main(int argc, char **argv)
{
    if (false)
    {
	RecordList rr(3);
	rr[0].id = 0;
	rr[1].id = 1;
	rr[2].id = 2;
	rr[0].action_angular = 0;
	rr[1].action_angular = 1;
	rr[2].action_angular = 2;

	cout << "rr" << endl
	    << rr << endl << endl;

	std::ofstream out("test.bin", std::ios::binary);
	recordListsPersistent(rr, out); cout << endl;
	out.close();

	std::ifstream in("test.bin", std::ios::binary);
	auto rr2 = persistentsRecordList(in);
	in.close();

	cout << "rr2" << endl
	    << *rr2 << endl << endl;

	std::ifstream in2("test.bin", std::ios::binary);
	cout << in2 << endl;
	in2.close();
    }

    if (false)
    {
	std::ifstream in("202001222010_2.TBOT01.bin", std::ios::binary);
	auto rr = persistentsRecordList(in);
	in.close();
	cout << *rr << endl;
    }

    if (false)
    {
	std::ifstream in("202001222010_2.TBOT01.bin", std::ios::binary);
	cout << in;
	in.close();
    }

    if (true)
    {
	auto rr = std::make_unique<RecordList>();
	try
	{
	    std::ifstream in("202001222010_2.TBOT01.bin", std::ios::binary);
	    if (in.is_open())
	    {
		rr = persistentsRecordList(in);
		in.close();
	    }
	    else
	    {
		cout << "cannot open file" << endl;
		return 1;
	    }
	}
	catch (const exception&)
	{
	    cout << "cannot read file" << endl;
	    return 1;
	}

	EVAL(rr->size());
	EVAL(rr->front().id);
	EVAL(rr->front().ts);
	EVAL(rr->front());
	EVAL(rr->back().id);
	EVAL(rr->back().ts);
	EVAL(rr->back());

	/*
	rr->size() = 279
	rr->front().id = 0
	rr->front().ts = 0.252699
	rr->back().id = 278
	rr->back().ts = 69.7564
	*/

	std::vector<double> sensor_pose[7];
	std::vector<double> sensor_scan;
	std::vector<double> action_linear;
	std::vector<double> action_angular;
	for (auto& r : *rr)
	{
	    for (std::size_t i = 0; i < 7; i++)
		sensor_pose[i].push_back(r.sensor_pose[i]);
	    for (std::size_t i = 0; i < 360; i++)
		sensor_scan.push_back(r.sensor_scan[i]);
	    action_linear.push_back(r.action_linear);
	    action_angular.push_back(r.action_angular);
	}
	for (std::size_t i = 0; i < 7; i++)
	    std::sort(sensor_pose[i].begin(), sensor_pose[i].end());
	std::sort(sensor_scan.begin(), sensor_scan.end());
	std::sort(action_linear.begin(), action_linear.end());
	std::sort(action_angular.begin(), action_angular.end());
	for (std::size_t i = 0; i < 7; i++)
	{
	    EVAL(i);
	    EVAL(sensor_pose[i].front());
	    EVAL(sensor_pose[i].back());
	}
	EVAL(sensor_scan.front());
	EVAL(sensor_scan.back());
	EVAL(action_linear.front());
	EVAL(action_linear.back());
	EVAL(action_angular.front());
	EVAL(action_angular.back());

	/*
	i = 0
	sensor_pose[i].front() = -1.97511
	sensor_pose[i].back() = 4.35613
	i = 1
	sensor_pose[i].front() = -11.2565
	sensor_pose[i].back() = 1.50003
	i = 2
	sensor_pose[i].front() = 0.0083258
	sensor_pose[i].back() = 0.0107955
	i = 3
	sensor_pose[i].front() = -0.0210602
	sensor_pose[i].back() = 0.00900391
	i = 4
	sensor_pose[i].front() = -0.0261257
	sensor_pose[i].back() = 0.0302976
	i = 5
	sensor_pose[i].front() = -0.972794
	sensor_pose[i].back() = 0.150416
	i = 6
	sensor_pose[i].front() = 0.231651
	sensor_pose[i].back() = 0.999958
	sensor_scan.front() = 0.12
	sensor_scan.back() = 3.5
	action_linear.front() = 0
	action_linear.back() = 0.3
	action_angular.front() = -1.5
	action_angular.back() = 1.5
	*/

	std::vector<double> sensor_scan_range{ 0.5,1.0,1.5,2.0,2.5,3.0,3.5,4.0 };
	std::map<double, std::size_t> sensor_scan_dist;
	for (auto x : sensor_scan_range)
	    sensor_scan_dist[x] = 0;
	for (auto y : sensor_scan)
	    for (auto x : sensor_scan_range)
		if (y <= x)
		{
		    sensor_scan_dist[x]++;
		    break;
		}
	EVAL(sensor_scan.size());
	EVAL(sensor_scan_dist);
	/*
	sensor_scan.size() = 100440
	sensor_scan_dist = {(0.5,4391),(1,17001),(1.5,8267),(2,5194),(2.5,2919),(3,2160),(3.5,60508),(4,0)}
	*/

	std::size_t sensor_scan_limit = 0;
	for (auto y : sensor_scan)
	    if (y == 3.5)
		sensor_scan_limit++;
	EVAL(sensor_scan_limit);
	/*
	sensor_scan_limit = 58317
	*/

	for (std::size_t i = 0; i < sensor_scan.size() - sensor_scan_limit; i += (sensor_scan.size() - sensor_scan_limit) / 7)
	{
	    EVAL(i);
	    EVAL(sensor_scan[i]);
	}
	/*
	i = 0
	sensor_scan[i] = 0.12
	i = 6017
	sensor_scan[i] = 0.524778
	i = 12034
	sensor_scan[i] = 0.644876
	i = 18051
	sensor_scan[i] = 0.848718
	i = 24068
	sensor_scan[i] = 1.1434
	i = 30085
	sensor_scan[i] = 1.53067
	i = 36102
	sensor_scan[i] = 2.18768
	i = 42119
	sensor_scan[i] = 3.49977
	*/

	std::set<double> action_linear_dist;
	for (auto y : action_linear)
	    action_linear_dist.insert(y);
	EVAL(action_linear_dist);

	/*
	action_linear_dist = {0,0.3}
	*/

	std::set<double> action_angular_dist;
	for (auto y : action_angular)
	    action_angular_dist.insert(y);
	EVAL(action_angular_dist);
	/*
	action_angular_dist = {-1.5,0,1.5}
	*/

    }

    return 0;
}
