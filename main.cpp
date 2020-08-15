#include "dev.h"

using namespace Alignment;
using namespace TBOT01;
using namespace std;

typedef std::chrono::duration<double> sec; 
typedef std::chrono::high_resolution_clock clk;

#define ECHO(x) cout << #x << endl; x
#define EVAL(x) cout << #x << ": " << (x) << endl
#define EVALL(x) cout << #x << ": " << endl << (x) << endl
#define TRUTH(x) cout << #x << ": " << ((x) ? "true" : "false") << endl

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
		std::ifstream in("data001.bin", std::ios::binary);
		auto rr = persistentsRecordList(in);
		in.close();
		cout << *rr << endl;
	}

	if (false)
	{
		std::ifstream in("data001.bin", std::ios::binary);
		cout << in;
		in.close();
	}

	if (false)
	{
		auto rr = std::make_unique<RecordList>();
		try
		{
			std::ifstream in("data001.bin", std::ios::binary);
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

	if (false)
	{
		auto hrsel = [](const HistoryRepa& hr, const SizeList& ll)
		{
			return eventsHistoryRepasHistoryRepaSelection_u(ll.size(), (std::size_t*)ll.data(), hr);
		};

		std::ifstream in("data001.bin", std::ios::binary);
		auto qq = persistentsRecordList(in);
		in.close();

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
		std::unique_ptr<Alignment::HistoryRepa> hr;
		{
			auto xx = recordListsHistoryRepa(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}
		EVALL(*uu);
		EVALL(*ur);
		EVALL(*hrsel(*hr, SizeList{0}));
		EVALL(*hrsel(*hr, SizeList{hr->size-1}));

		auto bm = historyRepasBitmap(3, 8, *hr);
		bmwrite("data001.bmp", bm);
	}

	if (argc >= 3 && string(argv[1]) == "stats")
	{
		auto rr = std::make_unique<RecordList>();
		try
		{
			std::ifstream in(string(argv[2])+".bin", std::ios::binary);
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

		std::size_t sensor_scan_limit = 0;
		for (auto y : sensor_scan)
			if (y == 3.5)
				sensor_scan_limit++;
		EVAL(sensor_scan_limit);

		for (std::size_t i = 0; i < sensor_scan.size() - sensor_scan_limit; i += (sensor_scan.size() - sensor_scan_limit) / 7)
		{
			EVAL(i);
			EVAL(sensor_scan[i]);
		}

		std::set<double> action_linear_dist;
		for (auto y : action_linear)
			action_linear_dist.insert(y);
		EVAL(action_linear_dist);

		std::set<double> action_angular_dist;
		for (auto y : action_angular)
			action_angular_dist.insert(y);
		EVAL(action_angular_dist);
	}

	if (argc >= 3 && string(argv[1]) == "dump")
	{
		string dataset = string(argc >= 3 ? argv[2] : "data002");
		size_t start = argc >= 4 ? atoi(argv[3]) : 0;
		size_t end = argc >= 5 ? atoi(argv[4]) : 0;
		
		EVAL(dataset);

		std::ifstream in(dataset+".bin", std::ios::binary);
		auto rr = persistentsRecordList(in);
		in.close();
		
		end = end && end < rr->size() ? end : rr->size();

		EVAL(rr->size());
		EVAL(start);
		EVAL(end);
		for (std::size_t i = start; i < end; i++)
		{
			std::cout << (*rr)[i].id;
			for (std::size_t j = 0; j < 7; j++)
				std::cout << "," << (*rr)[i].sensor_pose[j];
			std::cout << std::endl;
		}
	}
	
	if (argc >= 3 && string(argv[1]) == "bitmap")
	{
		auto hrsel = [](const HistoryRepa& hr, const SizeList& ll)
		{
			return eventsHistoryRepasHistoryRepaSelection_u(ll.size(), (std::size_t*)ll.data(), hr);
		};
		
		size_t scale_vertical = argc >= 4 ? atoi(argv[3]) : 1;
		size_t event_start = argc >= 5 ? atoi(argv[4]) : 0;
		size_t event_end = argc >= 6 ? atoi(argv[5]) : 0;

		std::ifstream in(string(argv[2]) + ".bin", std::ios::binary);
		auto qq = persistentsRecordList(in);
		in.close();

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
		std::unique_ptr<Alignment::HistoryRepa> hr;
		{
			auto xx = recordListsHistoryRepa(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}
		if (event_end > 0)
		{
			SizeList ll;
			for (size_t i = event_start; i <= event_end; i++)
				ll.push_back(i);
			hr = hrsel(*hr, ll);
		}

		auto bm = historyRepasBitmap(scale_vertical, 8, *hr);
		bmwrite(string(argv[2]) + ".bmp", bm);
	}

	if (argc >= 3 && string(argv[1]) == "bitmap_average")
	{
		auto hrsel = [](const HistoryRepa& hr, const SizeList& ll)
		{
			return eventsHistoryRepasHistoryRepaSelection_u(ll.size(), (std::size_t*)ll.data(), hr);
		};

		std::ifstream in(string(argv[2]) + ".bin", std::ios::binary);
		auto qq = persistentsRecordList(in);
		in.close();

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
		std::unique_ptr<Alignment::HistoryRepa> hr;
		{
			auto xx = recordListsHistoryRepa(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			hr = std::move(std::get<2>(xx));
		}

		auto bm = historyRepasBitmapAverage((argc >= 4 ? atoi(argv[3]) : 10), 8, *hr);
		bmwrite(string(argv[2]) + "_average.bmp", bm);
	}

	if (argc >= 3 && string(argv[1]) == "induce" && string(argv[2]) == "model001")
	{
		auto uvars = systemsSetVar;
		auto applicationer = parametersSystemsFudRepasHistoryRepasApplicationerSubstrateEntropyMaxRollByMExcludedSelfHighestFmaxIORepa_p;

		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;

		std::vector<std::string> files{
			"data002_room1.bin",
			"data002_room2.bin",
			"data002_room2_2.bin",
			"data002_room3.bin",
			"data002_room4.bin",
			"data002_room5.bin",
			"data002_room5_2.bin"
		};
		HistoryRepaPtrList ll;
		for (auto& f : files)
		{
			std::ifstream in(f, std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_2(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			ll.push_back(std::move(std::get<2>(xx)));
		}
		auto hr = vectorHistoryRepasConcat_u(ll);

		EVAL(hr->dimension);
		EVAL(hr->size);

		Variable motor("motor");
		Variable location("location");
		Variable position("position");
		auto vv = *uvars(*uu);
		auto vvk = VarUSet(vv);
		vvk.erase(motor);
		vvk.erase(location);
		vvk.erase(position);

		auto& vvi = ur->mapVarSize();
		auto vvk0 = sorted(vvk);
		SizeList vvk1;
		for (auto& v : vvk0)
			vvk1.push_back(vvi[v]);

		size_t wmax = 18;
		size_t lmax = 8;
		size_t xmax = 128;
		double znnmax = (double)hr->size * 2.0 * 300.0 * 300.0;
		size_t omax = 10;
		size_t bmax = 10 * 3;
		size_t mmax = 3;
		size_t umax = 128;
		size_t pmax = 1;
		size_t fmax = 127;
		size_t mult = 1;
		size_t seed = 5;
		auto dr = applicationer(wmax, lmax, xmax, znnmax, omax, bmax, mmax, umax, pmax, fmax, mult, 0, seed, tint, vvk1, FudRepa(), *hr, 0, *ur);
		std::ofstream out("model001.dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr, out); cout << endl;
		out.close();
	}

	if (argc >= 3 && string(argv[1]) == "induce" && string(argv[2]) == "model002")
	{
		auto uvars = systemsSetVar;
		auto applicationer = parametersSystemsFudRepasHistoryRepasApplicationerSubstrateEntropyMaxRollByMExcludedSelfHighestFmaxIORepa_p;

		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;

		std::vector<std::string> files{
			"data002_room1.bin",
			"data002_room2.bin",
			"data002_room2_2.bin",
			"data002_room3.bin",
			"data002_room4.bin",
			"data002_room5.bin",
			"data002_room5_2.bin"
		};
		HistoryRepaPtrList ll;
		int s = 17;
		for (auto& f : files)
		{
			std::ifstream in(f, std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			for (int i = 0; i < 10; i++)
			{
				auto xx = recordListsHistoryRepaRegion(8, 60, s++, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
		}
		auto hr = vectorHistoryRepasConcat_u(ll);

		EVAL(hr->dimension);
		EVAL(hr->size);

		auto vvk = *uvars(*uu);

		auto& vvi = ur->mapVarSize();
		auto vvk0 = sorted(vvk);
		SizeList vvk1;
		for (auto& v : vvk0)
			vvk1.push_back(vvi[v]);

		size_t wmax = 18;
		size_t lmax = 8;
		size_t xmax = 128;
		double znnmax = (double)hr->size * 2.0 * 100.0 * 100.0;
		size_t omax = 10;
		size_t bmax = 10 * 3;
		size_t mmax = 3;
		size_t umax = 128;
		size_t pmax = 1;
		size_t fmax = 127;
		size_t mult = 1;
		size_t seed = 5;
		auto dr = applicationer(wmax, lmax, xmax, znnmax, omax, bmax, mmax, umax, pmax, fmax, mult, 0, seed, tint, vvk1, FudRepa(), *hr, 0, *ur);
		std::ofstream out("model002.dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr, out); cout << endl;
		out.close();
	}

	if (argc >= 3 && string(argv[1]) == "induce" && string(argv[2]) == "model004")
	{
		auto uvars = systemsSetVar;
		auto applicationer = parametersSystemsFudRepasHistoryRepasApplicationerSubstrateEntropyMaxRollByMExcludedSelfHighestFmaxIORepa_p;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;

		std::vector<std::string> files{
			"data002_room1.bin",
			"data002_room2.bin",
			"data002_room2_2.bin",
			"data002_room3.bin",
			"data002_room4.bin",
			"data002_room5.bin",
			"data002_room5_2.bin"
		};
		HistoryRepaPtrList ll;
		int s = 17;
		for (auto& f : files)
		{
			std::ifstream in(f, std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			for (int i = 0; i < 10; i++)
			{
				auto xx = recordListsHistoryRepaRegion(8, 60, s++, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
		}
		auto hr = vectorHistoryRepasConcat_u(ll);

		EVAL(hr->dimension);
		EVAL(hr->size);

		auto vvk = *uvars(*uu);

		auto& vvi = ur->mapVarSize();
		auto vvk0 = sorted(vvk);
		SizeList vvk1;
		for (auto& v : vvk0)
			vvk1.push_back(vvi[v]);

		size_t wmax = 9;
		size_t lmax = 8;
		size_t xmax = 128;
		double znnmax = (double)hr->size * 2.0 * 100.0 * 100.0;
		size_t omax = 10;
		size_t bmax = 10 * 3;
		size_t mmax = 3;
		size_t umax = 128;
		size_t pmax = 1;
		size_t fmax = 127;
		size_t mult = 1;
		size_t seed = 5;
		auto dr = applicationer(wmax, lmax, xmax, znnmax, omax, bmax, mmax, umax, pmax, fmax, mult, 0, seed, tint, vvk1, FudRepa(), *hr, 0, *ur);
		std::ofstream out(model+".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr, out); cout << endl;
		out.close();
	}

	if (argc >= 3 && string(argv[1]) == "bitmap_model" && (string(argv[2]) == "model002" || string(argv[2]) == "model004"))
	{
		auto uvars = systemsSetVar;
		auto single = histogramSingleton_u;
		auto aahr = [](const System& uu, const SystemRepa& ur, const Histogram& aa)
		{
			return systemsHistoriesHistoryRepa_u(uu, ur, *histogramsHistory_u(aa));
		};
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrhrred = setVarsHistoryRepasHistoryRepaReduced_u;
		auto hrred = setVarsHistoryRepasReduce_u;
		auto frmul = historyRepasFudRepasMultiply_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto frdep = fudRepasSetVarsDepends;
		auto hrbm = historyRepasBitmapAverage;

		string model = string(argv[2]);
		int zmin = argc >= 4 ? atoi(argv[3]) : 50;

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
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
			HistoryRepaPtrList ll;
			int s = 17;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				for (int i = 0; i < 10; i++)
				{
					auto xx = recordListsHistoryRepaRegion(8, 60, s++, *qq);
					uu = std::move(std::get<0>(xx));
					ur = std::move(std::get<1>(xx));
					ll.push_back(std::move(std::get<2>(xx)));
				}
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);

		auto vvk = *uvars(*uu);

		auto& vvi = ur->mapVarSize();
		auto vvk0 = sorted(vvk);
		SizeList vvk1;
		for (auto& v : vvk0)
			vvk1.push_back(vvi[v]);

		StrVarPtrMap m;
		std::ifstream in(model + ".dr", std::ios::binary);
		auto ur1 = persistentsSystemRepa(in, m);
		auto dr = persistentsApplicationRepa(in);
		in.close();

		EVAL(frder(*dr->fud)->size());
		EVAL(frund(*dr->fud)->size());
		EVAL(frvars(*dr->fud)->size());

		auto hr1 = frmul(*hr, *dr->fud);
		if (hr1->evient)
			hr1->transpose();
		auto z = hr1->size;
		auto& mvv = hr1->mapVarInt();
		auto sl = treesElements(*dr->slices);
		std::map<std::size_t, std::shared_ptr<HistoryRepa>> shr;
		for (auto s : *sl)
		{
			SizeList ev;
			auto pk = mvv[s];
			for (std::size_t j = 0; j < z; j++)
			{
				std::size_t u = hr1->arr[pk*z + j];
				if (u)
					ev.push_back(j);
			}
			shr[s] = move(hrhrred(vvk1.size(), vvk1.data(), *hrsel(ev.size(), ev.data(), *hr1)));
		}
		auto ll = treesPaths(*dr->slices);
		vector<vector<pair<double, size_t>>> ll1;
		for (auto pp : *ll)
		{
			vector<pair<double, size_t>> pp1;
			for (auto s : pp)
				if (shr[s]->size >= zmin)
					pp1.push_back(pair<double, size_t>((double)shr[s]->size, s));
			if (pp1.size())
				ll1.push_back(pp1);
		}
		auto ll0 = *treesPaths(*pathsTree(ll1));
		sort(ll0.begin(), ll0.end());
		reverse(ll0.begin(), ll0.end());
		std::vector<Bitmap> ll2;
		for (auto pp : ll0)
		{
			std::vector<Bitmap> pp1;
			for (auto p : pp)
				if (p.first > 0)
					pp1.push_back(hrbm(20, 8, *shr[p.second]));
			if (pp1.size())
				ll2.push_back(bmhstack(pp1));
		}
		bmwrite(model + ".bmp", bmvstack(ll2));
	}

	if (argc >= 3 && string(argv[1]) == "induce" && string(argv[2]) == "model003")
	{
		auto uvars = systemsSetVar;
		auto applicationer = parametersSystemsFudRepasHistoryRepasApplicationerSubstrateEntropyMaxRollByMExcludedSelfHighestFmaxIORepa_p;

		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;

		std::vector<std::string> files{
			"data002_room1.bin",
			"data002_room2.bin",
			"data002_room2_2.bin",
			"data002_room3.bin",
			"data002_room4.bin",
			"data002_room5.bin",
			"data002_room5_2.bin"
		};
		HistoryRepaPtrList ll;
		int s = 17;
		for (auto& f : files)
		{
			std::ifstream in(f, std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			for (int i = 0; i < 10; i++)
			{
				auto xx = recordListsHistoryRepaRegion(4, 60, s++, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
		}
		auto hr = vectorHistoryRepasConcat_u(ll);

		EVAL(hr->dimension);
		EVAL(hr->size);

		auto vvk = *uvars(*uu);

		auto& vvi = ur->mapVarSize();
		auto vvk0 = sorted(vvk);
		SizeList vvk1;
		for (auto& v : vvk0)
			vvk1.push_back(vvi[v]);

		size_t wmax = 18;
		size_t lmax = 8;
		size_t xmax = 128;
		double znnmax = (double)hr->size * 2.0 * 100.0 * 100.0;
		size_t omax = 10;
		size_t bmax = 10 * 3;
		size_t mmax = 3;
		size_t umax = 128;
		size_t pmax = 1;
		size_t fmax = 127;
		size_t mult = 1;
		size_t seed = 5;
		auto dr = applicationer(wmax, lmax, xmax, znnmax, omax, bmax, mmax, umax, pmax, fmax, mult, 0, seed, tint, vvk1, FudRepa(), *hr, 0, *ur);
		std::ofstream out("model003.dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr, out); cout << endl;
		out.close();
	}

	if (argc >= 3 && string(argv[1]) == "bitmap_model" && string(argv[2]) == "model003")
	{
		auto uvars = systemsSetVar;
		auto single = histogramSingleton_u;
		auto aahr = [](const System& uu, const SystemRepa& ur, const Histogram& aa)
		{
			return systemsHistoriesHistoryRepa_u(uu, ur, *histogramsHistory_u(aa));
		};
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrhrred = setVarsHistoryRepasHistoryRepaReduced_u;
		auto hrred = setVarsHistoryRepasReduce_u;
		auto frmul = historyRepasFudRepasMultiply_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto frdep = fudRepasSetVarsDepends;
		auto hrbm = historyRepasBitmapAverage;

		string model = string(argv[2]);
		int zmin = argc >= 4 ? atoi(argv[3]) : 50;

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
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
			HistoryRepaPtrList ll;
			int s = 17;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				for (int i = 0; i < 10; i++)
				{
					auto xx = recordListsHistoryRepaRegion(4, 60, s++, *qq);
					uu = std::move(std::get<0>(xx));
					ur = std::move(std::get<1>(xx));
					ll.push_back(std::move(std::get<2>(xx)));
				}
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);

		auto vvk = *uvars(*uu);

		auto& vvi = ur->mapVarSize();
		auto vvk0 = sorted(vvk);
		SizeList vvk1;
		for (auto& v : vvk0)
			vvk1.push_back(vvi[v]);

		StrVarPtrMap m;
		std::ifstream in(model + ".dr", std::ios::binary);
		auto ur1 = persistentsSystemRepa(in, m);
		auto dr = persistentsApplicationRepa(in);
		in.close();

		EVAL(frder(*dr->fud)->size());
		EVAL(frund(*dr->fud)->size());
		EVAL(frvars(*dr->fud)->size());

		auto hr1 = frmul(*hr, *dr->fud);
		if (hr1->evient)
			hr1->transpose();
		auto z = hr1->size;
		auto& mvv = hr1->mapVarInt();
		auto sl = treesElements(*dr->slices);
		std::map<std::size_t, std::shared_ptr<HistoryRepa>> shr;
		for (auto s : *sl)
		{
			SizeList ev;
			auto pk = mvv[s];
			for (std::size_t j = 0; j < z; j++)
			{
				std::size_t u = hr1->arr[pk*z + j];
				if (u)
					ev.push_back(j);
			}
			shr[s] = move(hrhrred(vvk1.size(), vvk1.data(), *hrsel(ev.size(), ev.data(), *hr1)));
		}
		auto ll = treesPaths(*dr->slices);
		vector<vector<pair<double, size_t>>> ll1;
		for (auto pp : *ll)
		{
			vector<pair<double, size_t>> pp1;
			for (auto s : pp)
				if (shr[s]->size >= zmin)
					pp1.push_back(pair<double, size_t>((double)shr[s]->size, s));
			if (pp1.size())
				ll1.push_back(pp1);
		}
		auto ll0 = *treesPaths(*pathsTree(ll1));
		sort(ll0.begin(), ll0.end());
		reverse(ll0.begin(), ll0.end());
		std::vector<Bitmap> ll2;
		for (auto pp : ll0)
		{
			std::vector<Bitmap> pp1;
			for (auto p : pp)
				if (p.first > 0)
					pp1.push_back(hrbm(20, 4, *shr[p.second]));
			if (pp1.size())
				ll2.push_back(bmhstack(pp1));
		}
		bmwrite(model + ".bmp", bmvstack(ll2));
	}

	if (argc >= 3 && string(argv[1]) == "induce" && string(argv[2]) == "model005")
	{
		auto uvars = systemsSetVar;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;
		auto applicationer = parametersSystemsFudRepasHistoryRepasApplicationerSubstrateEntropyMaxRollByMExcludedSelfHighestFmaxIORepa_p;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;

		std::vector<std::string> files{
			"data002_room1.bin",
			"data002_room2.bin",
			"data002_room2_2.bin",
			"data002_room3.bin",
			"data002_room4.bin",
			"data002_room5.bin",
			"data002_room5_2.bin"
		};
		HistoryRepaPtrList ll;
		for (auto& f : files)
		{
			std::ifstream in(f, std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_2(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			ll.push_back(std::move(std::get<2>(xx)));
		}
		auto hr = vectorHistoryRepasConcat_u(ll);

		EVAL(hr->dimension);
		EVAL(hr->size);

		Variable motor("motor");
		Variable location("location");
		Variable position("position");
		auto vv = *uvars(*uu);
		auto vvk = VarUSet(vv);
		vvk.erase(motor);
		vvk.erase(location);
		vvk.erase(position);

		auto& vvi = ur->mapVarSize();
		auto vvk0 = sorted(vvk);
		SizeList vvk1;
		for (auto& v : vvk0)
			vvk1.push_back(vvi[v]);

		ApplicationRepa dr;
		{
			StrVarPtrMap m;
			std::ifstream in("model004.dr", std::ios::binary);
			auto ur1 = persistentsSystemRepa(in, m);
			auto dr1 = persistentsApplicationRepa(in);
			in.close();
			auto& llu1 = ur1->listVarSizePair;
			VarSizeUMap ur0 = ur->mapVarSize();
			auto n = fudRepasSize(*dr1->fud);
			size_t a = 360;
			size_t b = 60;
			auto& llu = ur->listVarSizePair;
			llu.reserve(n*a/b + a);
			dr.slices = std::make_shared<SizeTree>();
			dr.slices->_list.reserve(dr1->slices->_list.size() * a / b);
			dr.fud = std::make_shared<FudRepa>();
			dr.fud->layers.reserve(dr1->fud->layers.size());
			dr.substrate.reserve(dr1->substrate.size() * a / b);
			auto vframe = std::make_shared<Variable>("f");
			for (int i = 0; i < a*2/b; i++)
			{
				auto dr2 = drcopy(*dr1);
				SizeSizeUMap nn;
				nn.reserve(n + b);
				for (auto x1 : dr1->substrate)
				{
					auto& p = llu1[x1];
					auto v1 = p.first->_var0;
					auto v2 = std::make_shared<Variable>(((int)(p.first->_var1->_int + i*b/2 - 1)) % a + 1);
					auto v = std::make_shared<Variable>(v1, v2);
					nn[x1] = ur0[*v];
				}
				auto v3 = std::make_shared<Variable>((int)i+1);
				auto vd1 = std::make_shared<Variable>(vframe, v3);
				for (auto& ll : dr1->fud->layers)
					for (auto& tr : ll)
					{
						auto x1 = tr->derived;
						auto& p = llu1[x1];
						auto vdfl = p.first->_var0;
						auto vb = p.first->_var1;
						auto vdf = vdfl->_var0;
						auto vl = vdfl->_var1;
						auto vf = vdf->_var1;
						auto vdf1 = std::make_shared<Variable>(vd1, vf);
						auto vdfl1 = std::make_shared<Variable>(vdf1, vl);
						auto vdflb1 = std::make_shared<Variable>(vdfl1, vb);
						llu.push_back(VarSizePair(vdflb1, p.second));
						nn[x1] = llu.size() - 1;
					}
				dr2->reframe_u(nn);
				dr.slices->_list.insert(dr.slices->_list.end(), dr2->slices->_list.begin(), dr2->slices->_list.end());
				for (std::size_t l = 0; l < dr2->fud->layers.size(); l++)
				{
					if (l < dr.fud->layers.size())
						dr.fud->layers[l].insert(dr.fud->layers[l].end(), dr2->fud->layers[l].begin(), dr2->fud->layers[l].end());
					else
						dr.fud->layers.push_back(dr2->fud->layers[l]);
				}
				dr.substrate.insert(dr.substrate.end(), dr2->substrate.begin(), dr2->substrate.end());
			}
		}

		size_t wmax = 18;
		size_t lmax = 8;
		size_t xmax = 128;
		double znnmax = 60000.0 * 2.0 * 100.0 * 100.0 * tint;
		size_t omax = 10;
		size_t bmax = 10 * 3;
		size_t mmax = 3;
		size_t umax = 128;
		size_t pmax = 1;
		size_t fmax = 127;
		size_t mult = 1;
		size_t seed = 5;
		auto sl = treesElements(*dr.slices);
		auto dr2 = applicationer(wmax, lmax, xmax, znnmax, omax, bmax, mmax, umax, pmax, fmax, mult, 0, seed, tint, *sl, *dr.fud, *hr, 0, *ur);
		auto dr3 = drjoin(dr, *dr2);
		std::ofstream out(model + ".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr3, out); cout << endl;
		out.close();
	}

	if (argc >= 3 && string(argv[1]) == "condition" && string(argv[2]) == "model006")
	{
		auto uvars = systemsSetVar;
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto frmul = historyRepasFudRepasMultiply_u;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;
		auto applicationer = parametersSystemsHistoryRepasApplicationerCondMultinomialFmaxIORepa_up;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;
		string label = argc >= 5 ? string(argv[4]) : "motor";

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
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
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				auto xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);

		Variable motor("motor");
		Variable location("location");
		Variable position("position");
		auto vv = *uvars(*uu);
		auto vvk = VarUSet(vv);
		vvk.erase(motor);		
		vvk.erase(location);
		vvk.erase(position);

		auto& vvi = ur->mapVarSize();
		SizeList vvk1;
		for (auto& v : sorted(vvk))
			vvk1.push_back(vvi[v]);

		size_t fmax = 1024;
		auto dr = applicationer(fmax, tint, vvk1, vvi[Variable(label)], *hr, 1, *ur);
		std::ofstream out(model + "_" + label + ".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr, out); cout << endl;
		out.close();
	}

	if (argc >= 3 && string(argv[1]) == "condition" && string(argv[2]) == "model007")
	{
		auto uvars = systemsSetVar;
		auto frmul = historyRepasFudRepasMultiply_up;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;
		auto applicationer = parametersSystemsHistoryRepasApplicationerCondMultinomialFmaxIORepa_up;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;
		string label = argc >= 5 ? string(argv[4]) : "motor";

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
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
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				auto xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);

		Variable motor("motor");
		Variable location("location");
		Variable position("position");
		auto vv = *uvars(*uu);
		auto vvk = VarUSet(vv);
		vvk.erase(motor);
		vvk.erase(location);
		vvk.erase(position);

		auto& vvi = ur->mapVarSize();
		SizeList vvk1;
		for (auto& v : sorted(vvk))
			vvk1.push_back(vvi[v]);

		ApplicationRepa dr;
		{
			StrVarPtrMap m;
			std::ifstream in("model004.dr", std::ios::binary);
			auto ur1 = persistentsSystemRepa(in, m);
			auto dr1 = persistentsApplicationRepa(in);
			in.close();
			auto& llu1 = ur1->listVarSizePair;
			VarSizeUMap ur0 = ur->mapVarSize();
			auto n = fudRepasSize(*dr1->fud);
			size_t a = 360;
			size_t b = 60;
			auto& llu = ur->listVarSizePair;
			llu.reserve(n*a / b + a);
			dr.slices = std::make_shared<SizeTree>();
			dr.slices->_list.reserve(dr1->slices->_list.size() * a / b);
			dr.fud = std::make_shared<FudRepa>();
			dr.fud->layers.reserve(dr1->fud->layers.size());
			dr.substrate.reserve(dr1->substrate.size() * a / b);
			auto vframe = std::make_shared<Variable>("f");
			for (int i = 0; i < a * 2 / b; i++)
			{
				auto dr2 = drcopy(*dr1);
				SizeSizeUMap nn;
				nn.reserve(n + b);
				for (auto x1 : dr1->substrate)
				{
					auto& p = llu1[x1];
					auto v1 = p.first->_var0;
					auto v2 = std::make_shared<Variable>((int)(p.first->_var1->_int + i*b / 2 - 1));
					auto v = std::make_shared<Variable>(v1, v2);
					nn[x1] = ur0[*v];
				}
				auto v3 = std::make_shared<Variable>((int)i + 1);
				auto vd1 = std::make_shared<Variable>(vframe, v3);
				for (auto& ll : dr1->fud->layers)
					for (auto& tr : ll)
					{
						auto x1 = tr->derived;
						auto& p = llu1[x1];
						auto vdfl = p.first->_var0;
						auto vb = p.first->_var1;
						auto vdf = vdfl->_var0;
						auto vl = vdfl->_var1;
						auto vf = vdf->_var1;
						auto vdf1 = std::make_shared<Variable>(vd1, vf);
						auto vdfl1 = std::make_shared<Variable>(vdf1, vl);
						auto vdflb1 = std::make_shared<Variable>(vdfl1, vb);
						llu.push_back(VarSizePair(vdflb1, p.second));
						nn[x1] = llu.size() - 1;
					}
				dr2->reframe_u(nn);
				dr.slices->_list.insert(dr.slices->_list.end(), dr2->slices->_list.begin(), dr2->slices->_list.end());
				for (std::size_t l = 0; l < dr2->fud->layers.size(); l++)
				{
					if (l < dr.fud->layers.size())
						dr.fud->layers[l].insert(dr.fud->layers[l].end(), dr2->fud->layers[l].begin(), dr2->fud->layers[l].end());
					else
						dr.fud->layers.push_back(dr2->fud->layers[l]);
				}
				dr.substrate.insert(dr.substrate.end(), dr2->substrate.begin(), dr2->substrate.end());
			}
		}

		auto hr1 = frmul(tint, *hr, *dr.fud);
		auto sl = treesElements(*dr.slices);
		size_t fmax = 1024;
		auto dr2 = applicationer(fmax, tint, *sl, vvi[Variable(label)], *hr1, 1, *ur);
		auto dr3 = drjoin(dr, *dr2);
		std::ofstream out(model + "_" + label + ".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr3, out); cout << endl;
		out.close();
	}

	if (argc >= 3 && string(argv[1]) == "bitmap_model" && string(argv[2]) != "model002" && string(argv[2]) != "model003" && string(argv[2]) != "model004" && string(argv[2]) != "model024")
	{
		auto uvars = systemsSetVar;
		auto single = histogramSingleton_u;
		auto aahr = [](const System& uu, const SystemRepa& ur, const Histogram& aa)
		{
			return systemsHistoriesHistoryRepa_u(uu, ur, *histogramsHistory_u(aa));
		};
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrhrred = setVarsHistoryRepasHistoryRepaReduced_u;
		auto hrred = setVarsHistoryRepasReduce_u;
		auto frmul = historyRepasFudRepasMultiply_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto frdep = fudRepasSetVarsDepends;
		auto hrbm = historyRepasBitmapAverage;

		string model = string(argv[2]);
		int zmin = argc >= 4 ? atoi(argv[3]) : 5;
		string dataset = string(argc >= 5 ? argv[4] : "data002");

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
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
				auto xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);

		Variable motor("motor");
		Variable location("location");
		Variable position("position");
		auto vv = *uvars(*uu);
		auto vvk = VarUSet(vv);
		vvk.erase(motor);
		vvk.erase(location);
		vvk.erase(position);

		auto& vvi = ur->mapVarSize();
		auto vvk0 = sorted(vvk);
		SizeList vvk1;
		for (auto& v : vvk0)
			vvk1.push_back(vvi[v]);

		StrVarPtrMap m;
		std::ifstream in(model + ".dr", std::ios::binary);
		auto ur1 = persistentsSystemRepa(in, m);
		auto dr = persistentsApplicationRepa(in);
		in.close();

		EVAL(frder(*dr->fud)->size());
		EVAL(frund(*dr->fud)->size());
		EVAL(frvars(*dr->fud)->size());

		auto hr1 = frmul(*hr, *dr->fud);
		if (hr1->evient)
			hr1->transpose();
		auto z = hr1->size;
		auto& mvv = hr1->mapVarInt();
		auto sl = treesElements(*dr->slices);
		std::map<std::size_t, std::shared_ptr<HistoryRepa>> shr;
		for (auto s : *sl)
		{
			SizeList ev;
			auto pk = mvv[s];
			for (std::size_t j = 0; j < z; j++)
			{
				std::size_t u = hr1->arr[pk*z + j];
				if (u)
					ev.push_back(j);
			}
			shr[s] = move(hrhrred(vvk1.size(), vvk1.data(), *hrsel(ev.size(), ev.data(), *hr1)));
		}
		auto ll = treesPaths(*dr->slices);
		vector<vector<pair<double, size_t>>> ll1;
		for (auto pp : *ll)
		{
			vector<pair<double, size_t>> pp1;
			for (auto s : pp)
				if (shr[s]->size >= zmin)
					pp1.push_back(pair<double, size_t>((double)shr[s]->size, s));
			if (pp1.size())
				ll1.push_back(pp1);
		}
		auto ll0 = *treesPaths(*pathsTree(ll1));
		sort(ll0.begin(), ll0.end());
		reverse(ll0.begin(), ll0.end());
		std::vector<Bitmap> ll2;
		for (auto pp : ll0)
		{
			std::vector<Bitmap> pp1;
			for (auto p : pp)
				if (p.first > 0)
					pp1.push_back(hrbm(20, 8, *shr[p.second]));
			if (pp1.size())
				ll2.push_back(bmhstack(pp1));
		}
		bmwrite(model + ".bmp", bmvstack(ll2));
	}

	if (argc >= 3 && string(argv[1]) == "induce" && string(argv[2]) == "model008")
	{
		auto uvars = systemsSetVar;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto applicationer = parametersSystemsFudRepasHistoryRepasApplicationerSubstrateEntropyMaxRollByMExcludedSelfHighestFmaxIORepa_p;
		//		auto applicationer = parametersSystemsFudRepasHistoryRepasApplicationerSubstrateEntropyMaxRollByMExcludedSelfHighestFmaxIORepa;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
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
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				auto xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);

		auto vvk = *uvars(*uu);

		auto& vvi = ur->mapVarSize();
		auto vvk0 = sorted(vvk);
		SizeList vvk1;
		for (auto& v : vvk0)
			vvk1.push_back(vvi[v]);

		size_t wmax = 18;
		size_t lmax = 8;
		size_t xmax = 128;
		double znnmax = 60000.0 * 2.0 * 100.0 * 100.0 * 4;
		size_t omax = 10;
		size_t bmax = 10 * 3;
		size_t mmax = 3;
		size_t umax = 128;
		size_t pmax = 1;
		size_t fmax = 127;
		size_t mult = 1;
		size_t seed = 5;
		auto dr = applicationer(wmax, lmax, xmax, znnmax, omax, bmax, mmax, umax, pmax, fmax, mult, 0, seed, tint, vvk1, FudRepa(), *hr, 0, *ur);
		//		auto dr = applicationer(wmax, lmax, xmax, znnmax, omax, bmax, mmax, umax, pmax, fmax, mult, 0, seed, vvk1, FudRepa(), *hr, 0, *ur);
		std::ofstream out(model + ".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr, out); cout << endl;
		out.close();

		EVAL(frder(*dr->fud)->size());
		EVAL(frund(*dr->fud)->size());
		EVAL(frvars(*dr->fud)->size());
		EVAL(sorted(*frund(*dr->fud)));
	}

	if (argc >= 3 && string(argv[1]) == "test")
	{
		auto uvars = systemsSetVar;
		auto hrhrred = setVarsHistoryRepasHistoryRepaReduced_u;
		auto frmul = historyRepasFudRepasMultiply_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;

		string model = string(argv[2]);
		string label = argc >= 4 ? string(argv[3]) : "motor";

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
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
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				auto xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);

		auto& vvi = ur->mapVarSize();

		StrVarPtrMap m;
		std::ifstream in(model + ".dr", std::ios::binary);
		auto ur1 = persistentsSystemRepa(in, m);
		auto dr = persistentsApplicationRepa(in);
		in.close();

		EVAL(frder(*dr->fud)->size());
		EVAL(frund(*dr->fud)->size());
		EVAL(frvars(*dr->fud)->size());

		std::map<std::size_t, std::size_t> sp;
		{
			auto hr1 = frmul(*hr, *dr->fud);
			if (hr1->evient)
				hr1->transpose();
			auto z = hr1->size;
			auto& mvv = hr1->mapVarInt();
			auto sh = hr1->shape;
			auto rr = hr1->arr;
			auto pl = mvv[vvi[Variable(label)]];
			auto sl = sh[pl];
			auto nn = treesLeafNodes(*dr->slices);
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
						std::size_t w = rr[pl*z + j];
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
				sp[s.first] = cl;
			}
		}

		std::size_t a = 0;
		std::size_t x = 0;
		{
			SizeList ww{ vvi[Variable(label)] };
			auto nn = treesLeafNodes(*dr->slices);
			for (auto& s : *nn)
				ww.push_back(s.first);
			auto hr1 = hrhrred(ww.size(), ww.data(), *frmul(*hr, *dr->fud));
			if (!hr1->evient)
				hr1->transpose();
			auto n = hr1->dimension;
			auto vv = hr1->vectorVar;
			auto sh = hr1->shape;
			auto z = hr1->size;
			auto rr = hr1->arr;
			auto sl = sh[0];
			for (std::size_t j = 0; j < z; j++)
			{
				std::size_t cl = rr[j*n];
				for (std::size_t i = 1; i < n; i++)
				{
					std::size_t u = rr[j*n + i];
					if (u)
					{
						std::size_t l = sp[vv[i]];
						if (l == cl)
							a++;
						else if (l == sl)
							x++;
						break;
					}
				}
			}

		}
		EVAL(model);
		EVAL(label);
		cout << "effective size: " << hr->size - x << endl;
		cout << "matches: " << a << endl;

	}

	if (argc >= 2 && string(argv[1]) == "analyse")
	{
		auto aall = histogramsList;
		auto araa = systemsHistogramRepasHistogram_u;
		auto hrred = [](const HistoryRepa& hr, const SystemRepa& ur, const VarList& kk)
		{
			auto& vvi = ur.mapVarSize();
			std::size_t m = kk.size();
			SizeList kk1;
			for (std::size_t i = 0; i < m; i++)
				kk1.push_back(vvi[kk[i]]);
			return setVarsHistoryRepasReduce_u(1.0, m, kk1.data(), hr);
		};
		
		string dataset = string(argc >= 3 ? argv[2] : "data002");
		string substrate = string(argc >= 4 ? argv[3] : "substrate002");
		
		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
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
				if (substrate == "substrate006")
					xx = recordListsHistoryRepa_6(8, *qq);
				else if (substrate == "substrate005")
					xx = recordListsHistoryRepa_5(8, *qq);
				else if (substrate == "substrate004")
					xx = recordListsHistoryRepa_4(8, *qq);
				else if (substrate == "substrate003")
					xx = recordListsHistoryRepa_3(8, *qq);
				else
					xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);

		rpln(cout, sorted(*aall(*araa(*uu, *ur, *hrred(*hr, *ur, VarList{ Variable(Variable("scan"),Variable(1)) })))));
		cout << endl;
		rpln(cout, sorted(*aall(*araa(*uu, *ur, *hrred(*hr, *ur, VarList{ Variable(Variable("scan"),Variable(180)) })))));
		cout << endl;
		rpln(cout, sorted(*aall(*araa(*uu, *ur, *hrred(*hr, *ur, VarList{ Variable("motor") })))));
		cout << endl;
		rpln(cout, sorted(*aall(*araa(*uu, *ur, *hrred(*hr, *ur, VarList{ Variable("location") })))));
		cout << endl;
		if (substrate != "substrate005" || substrate != "substrate006")
		{
			rpln(cout, sorted(*aall(*araa(*uu, *ur, *hrred(*hr, *ur, VarList{ Variable("position") })))));
			cout << endl;			
		}
		if (substrate == "substrate003" || substrate == "substrate004")
		{
			rpln(cout, sorted(*aall(*araa(*uu, *ur, *hrred(*hr, *ur, VarList{ Variable("location_next") })))));
			cout << endl;
			rpln(cout, sorted(*aall(*araa(*uu, *ur, *hrred(*hr, *ur, VarList{ Variable("position_next") })))));			
			cout << endl;
		}
		if (substrate == "substrate004" || substrate == "substrate006")
		{
			rpln(cout, sorted(*aall(*araa(*uu, *ur, *hrred(*hr, *ur, VarList{ Variable("room_next") })))));
			cout << endl;		
		}	
	}

	if (argc >= 3 && string(argv[1]) == "condition" && string(argv[2]) == "model006_1")
	{
		auto uvars = systemsSetVar;
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto frmul = historyRepasFudRepasMultiply_u;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;
		auto applicationer = parametersSystemsHistoryRepasApplicationerCondMultinomialFmaxIORepa_up;
		auto applicationerDelta = parametersSystemsHistoryRepasApplicationerCondMultinomialFmaxDeltaIORepa_up;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;
		string label = argc >= 5 ? string(argv[4]) : "motor";

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
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
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				auto xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);

		Variable motor("motor");
		Variable location("location");
		Variable position("position");
		auto vv = *uvars(*uu);
		auto vvk = VarUSet(vv);
		vvk.erase(motor);
		vvk.erase(location);
		vvk.erase(position);

		auto& vvi = ur->mapVarSize();
		SizeList vvk1;
		for (auto& v : sorted(vvk))
			vvk1.push_back(vvi[v]);

		auto dr = applicationer(1, tint, vvk1, vvi[Variable(label)], *hr, 1, *ur);
		applicationerDelta(2, 127, tint, vvk1, vvi[Variable(label)], *hr, 1, *ur, *dr);
		std::ofstream out(model + "_" + label + ".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr, out); cout << endl;
		out.close();
	}

	if (argc >= 3 && string(argv[1]) == "condition" && string(argv[2]) == "model006_2")
	{
		auto uvars = systemsSetVar;
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto frmul = historyRepasFudRepasMultiply_u;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;
		auto applicationer = parametersSystemsHistoryRepasApplicationerCondMultinomialFmaxIORepa_up;
		auto applicationerDelta = parametersSystemsHistoryRepasApplicationerCondMultinomialFmaxDeltaIORepa_up;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;
		string label = argc >= 5 ? string(argv[4]) : "motor";

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
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
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				auto xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);

		Variable motor("motor");
		Variable location("location");
		Variable position("position");
		auto vv = *uvars(*uu);
		auto vvk = VarUSet(vv);
		vvk.erase(motor);
		vvk.erase(location);
		vvk.erase(position);

		auto& vvi = ur->mapVarSize();
		SizeList vvk1;
		for (auto& v : sorted(vvk))
			vvk1.push_back(vvi[v]);

		auto dr = std::make_unique<ApplicationRepa>();
		dr->substrate = vvk1;
		dr->fud = std::make_shared<FudRepa>();
		dr->slices = std::make_shared<SizeTree>();
		applicationerDelta(1, 1, tint, vvk1, vvi[Variable(label)], *hr, 1, *ur, *dr);
		applicationerDelta(2, 127, tint, vvk1, vvi[Variable(label)], *hr, 1, *ur, *dr);
		std::ofstream out(model + "_" + label + ".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr, out); cout << endl;
		out.close();
	}

	if (argc >= 3 && string(argv[1]) == "condition" && string(argv[2]) == "model006_3")
	{
		auto uvars = systemsSetVar;
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto frmul = historyRepasFudRepasMultiply_u;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;
		auto applicationer = parametersSystemsHistoryRepasApplicationerCondMultinomialFmaxIORepa_up;
		auto applicationerDelta = parametersSystemsHistoryRepasApplicationerCondMultinomialFmaxDeltaIORepa_up;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;
		string label = argc >= 5 ? string(argv[4]) : "motor";

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
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
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				auto xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);

		Variable motor("motor");
		Variable location("location");
		Variable position("position");
		auto vv = *uvars(*uu);
		auto vvk = VarUSet(vv);
		vvk.erase(motor);
		vvk.erase(location);
		vvk.erase(position);

		auto& vvi = ur->mapVarSize();
		SizeList vvk1;
		for (auto& v : sorted(vvk))
			vvk1.push_back(vvi[v]);

		auto dr = applicationer(126, tint, vvk1, vvi[Variable(label)], *hr, 1, *ur);
		applicationerDelta(127, 127, tint, vvk1, vvi[Variable(label)], *hr, 1, *ur, *dr);
		std::ofstream out(model + "_" + label + ".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr, out); cout << endl;
		out.close();
	}

	if (argc >= 3 && string(argv[1]) == "induce" && string(argv[2]) == "model001_1")
	{
		auto uvars = systemsSetVar;
		auto applicationer = parametersSystemsFudRepasHistoryRepasApplicationerSubstrateEntropyMaxRollByMExcludedSelfHighestFmaxIORepa_p;
		auto applicationerDelta = parametersSystemsFudRepasHistoryRepasApplicationerSubstrateEntropyMaxRollByMExcludedSelfHighestFmaxDeltaIORepa_p;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;

		std::vector<std::string> files{
			"data002_room1.bin",
			"data002_room2.bin",
			"data002_room2_2.bin",
			"data002_room3.bin",
			"data002_room4.bin",
			"data002_room5.bin",
			"data002_room5_2.bin"
		};
		HistoryRepaPtrList ll;
		for (auto& f : files)
		{
			std::ifstream in(f, std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_2(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			ll.push_back(std::move(std::get<2>(xx)));
		}
		auto hr = vectorHistoryRepasConcat_u(ll);

		EVAL(hr->dimension);
		EVAL(hr->size);

		Variable motor("motor");
		Variable location("location");
		Variable position("position");
		auto vv = *uvars(*uu);
		auto vvk = VarUSet(vv);
		vvk.erase(motor);
		vvk.erase(location);
		vvk.erase(position);

		auto& vvi = ur->mapVarSize();
		auto vvk0 = sorted(vvk);
		SizeList vvk1;
		for (auto& v : vvk0)
			vvk1.push_back(vvi[v]);

		size_t wmax = 18;
		size_t lmax = 8;
		size_t xmax = 128;
		double znnmax = (double)hr->size * 2.0 * 300.0 * 300.0;
		size_t omax = 10;
		size_t bmax = 10 * 3;
		size_t mmax = 3;
		size_t umax = 128;
		size_t pmax = 1;
		size_t fmax = 127;
		size_t mult = 1;
		size_t seed = 5;
		auto dr = applicationer(wmax, lmax, xmax, znnmax, omax, bmax, mmax, umax, pmax, 1, mult, 0, seed, tint, vvk1, FudRepa(), *hr, 0, *ur);
		applicationerDelta(wmax, lmax, xmax, znnmax, omax, bmax, mmax, umax, pmax, 2, fmax, mult, 0, seed, tint, vvk1, FudRepa(), *hr, 0, *ur, *dr);
		std::ofstream out(model + ".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr, out); cout << endl;
		out.close();
	}

	if (argc >= 3 && string(argv[1]) == "induce" && string(argv[2]) == "model001_2")
	{
		auto uvars = systemsSetVar;
		auto applicationer = parametersSystemsFudRepasHistoryRepasApplicationerSubstrateEntropyMaxRollByMExcludedSelfHighestFmaxIORepa_p;
		auto applicationerDelta = parametersSystemsFudRepasHistoryRepasApplicationerSubstrateEntropyMaxRollByMExcludedSelfHighestFmaxDeltaIORepa_p;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;

		std::vector<std::string> files{
			"data002_room1.bin",
			"data002_room2.bin",
			"data002_room2_2.bin",
			"data002_room3.bin",
			"data002_room4.bin",
			"data002_room5.bin",
			"data002_room5_2.bin"
		};
		HistoryRepaPtrList ll;
		for (auto& f : files)
		{
			std::ifstream in(f, std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_2(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			ll.push_back(std::move(std::get<2>(xx)));
		}
		auto hr = vectorHistoryRepasConcat_u(ll);

		EVAL(hr->dimension);
		EVAL(hr->size);

		Variable motor("motor");
		Variable location("location");
		Variable position("position");
		auto vv = *uvars(*uu);
		auto vvk = VarUSet(vv);
		vvk.erase(motor);
		vvk.erase(location);
		vvk.erase(position);

		auto& vvi = ur->mapVarSize();
		auto vvk0 = sorted(vvk);
		SizeList vvk1;
		for (auto& v : vvk0)
			vvk1.push_back(vvi[v]);

		size_t wmax = 18;
		size_t lmax = 8;
		size_t xmax = 128;
		double znnmax = (double)hr->size * 2.0 * 300.0 * 300.0;
		size_t omax = 10;
		size_t bmax = 10 * 3;
		size_t mmax = 3;
		size_t umax = 128;
		size_t pmax = 1;
		size_t fmax = 127;
		size_t mult = 1;
		size_t seed = 5;
		auto dr = std::make_unique<ApplicationRepa>();
		dr->substrate = vvk1;
		dr->fud = std::make_shared<FudRepa>();
		dr->slices = std::make_shared<SizeTree>();
		applicationerDelta(wmax, lmax, xmax, znnmax, omax, bmax, mmax, umax, pmax, 1, 64, mult, 0, seed, tint, vvk1, FudRepa(), *hr, 0, *ur, *dr);
		applicationerDelta(wmax, lmax, xmax, znnmax, omax, bmax, mmax, umax, pmax, 65, fmax, mult, 0, seed, tint, vvk1, FudRepa(), *hr, 0, *ur, *dr);
		std::ofstream out(model + ".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr, out); cout << endl;
		out.close();
	}

	if (argc >= 3 && string(argv[1]) == "induce" && string(argv[2]) == "model005_1")
	{
		auto uvars = systemsSetVar;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;
		auto applicationer = parametersSystemsFudRepasHistoryRepasApplicationerSubstrateEntropyMaxRollByMExcludedSelfHighestFmaxIORepa_p;
		auto applicationerDelta = parametersSystemsFudRepasHistoryRepasApplicationerSubstrateEntropyMaxRollByMExcludedSelfHighestFmaxDeltaIORepa_p;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;

		std::vector<std::string> files{
			"data002_room1.bin",
			"data002_room2.bin",
			"data002_room2_2.bin",
			"data002_room3.bin",
			"data002_room4.bin",
			"data002_room5.bin",
			"data002_room5_2.bin"
		};
		HistoryRepaPtrList ll;
		for (auto& f : files)
		{
			std::ifstream in(f, std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_2(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			ll.push_back(std::move(std::get<2>(xx)));
		}
		auto hr = vectorHistoryRepasConcat_u(ll);

		EVAL(hr->dimension);
		EVAL(hr->size);

		Variable motor("motor");
		Variable location("location");
		Variable position("position");
		auto vv = *uvars(*uu);
		auto vvk = VarUSet(vv);
		vvk.erase(motor);
		vvk.erase(location);
		vvk.erase(position);

		auto& vvi = ur->mapVarSize();
		auto vvk0 = sorted(vvk);
		SizeList vvk1;
		for (auto& v : vvk0)
			vvk1.push_back(vvi[v]);

		ApplicationRepa dr;
		{
			StrVarPtrMap m;
			std::ifstream in("model004.dr", std::ios::binary);
			auto ur1 = persistentsSystemRepa(in, m);
			auto dr1 = persistentsApplicationRepa(in);
			in.close();
			auto& llu1 = ur1->listVarSizePair;
			VarSizeUMap ur0 = ur->mapVarSize();
			auto n = fudRepasSize(*dr1->fud);
			size_t a = 360;
			size_t b = 60;
			auto& llu = ur->listVarSizePair;
			llu.reserve(n*a / b + a);
			dr.slices = std::make_shared<SizeTree>();
			dr.slices->_list.reserve(dr1->slices->_list.size() * a / b);
			dr.fud = std::make_shared<FudRepa>();
			dr.fud->layers.reserve(dr1->fud->layers.size());
			dr.substrate.reserve(dr1->substrate.size() * a / b);
			auto vframe = std::make_shared<Variable>("f");
			for (int i = 0; i < a * 2 / b; i++)
			{
				auto dr2 = drcopy(*dr1);
				SizeSizeUMap nn;
				nn.reserve(n + b);
				for (auto x1 : dr1->substrate)
				{
					auto& p = llu1[x1];
					auto v1 = p.first->_var0;
					auto v2 = std::make_shared<Variable>((int)(p.first->_var1->_int + i*b / 2 - 1));
					auto v = std::make_shared<Variable>(v1, v2);
					nn[x1] = ur0[*v];
				}
				auto v3 = std::make_shared<Variable>((int)i + 1);
				auto vd1 = std::make_shared<Variable>(vframe, v3);
				for (auto& ll : dr1->fud->layers)
					for (auto& tr : ll)
					{
						auto x1 = tr->derived;
						auto& p = llu1[x1];
						auto vdfl = p.first->_var0;
						auto vb = p.first->_var1;
						auto vdf = vdfl->_var0;
						auto vl = vdfl->_var1;
						auto vf = vdf->_var1;
						auto vdf1 = std::make_shared<Variable>(vd1, vf);
						auto vdfl1 = std::make_shared<Variable>(vdf1, vl);
						auto vdflb1 = std::make_shared<Variable>(vdfl1, vb);
						llu.push_back(VarSizePair(vdflb1, p.second));
						nn[x1] = llu.size() - 1;
					}
				dr2->reframe_u(nn);
				dr.slices->_list.insert(dr.slices->_list.end(), dr2->slices->_list.begin(), dr2->slices->_list.end());
				for (std::size_t l = 0; l < dr2->fud->layers.size(); l++)
				{
					if (l < dr.fud->layers.size())
						dr.fud->layers[l].insert(dr.fud->layers[l].end(), dr2->fud->layers[l].begin(), dr2->fud->layers[l].end());
					else
						dr.fud->layers.push_back(dr2->fud->layers[l]);
				}
				dr.substrate.insert(dr.substrate.end(), dr2->substrate.begin(), dr2->substrate.end());
			}
		}

		size_t wmax = 18;
		size_t lmax = 8;
		size_t xmax = 128;
		double znnmax = 60000.0 * 2.0 * 100.0 * 100.0 * tint;
		size_t omax = 10;
		size_t bmax = 10 * 3;
		size_t mmax = 3;
		size_t umax = 128;
		size_t pmax = 1;
		size_t fmax = 127;
		size_t mult = 1;
		size_t seed = 5;
		auto sl = treesElements(*dr.slices);
		auto dr2 = applicationer(wmax, lmax, xmax, znnmax, omax, bmax, mmax, umax, pmax, 64, mult, 0, seed, tint, *sl, *dr.fud, *hr, 0, *ur);
		applicationerDelta(wmax, lmax, xmax, znnmax, omax, bmax, mmax, umax, pmax, 65, fmax, mult, 0, seed, tint, *sl, *dr.fud, *hr, 0, *ur, *dr2);
		auto dr3 = drjoin(dr, *dr2);
		std::ofstream out(model + ".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr3, out); cout << endl;
		out.close();
	}

	if (false)
	{
		auto uvars = systemsSetVar;
		auto hrhrred = setVarsHistoryRepasHistoryRepaReduced_u;
		auto frmul = historyRepasFudRepasMultiply_u;
		auto hrhs = historyRepasHistorySparse;
		auto drcopy = applicationRepasApplicationRepa_u;

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;

		std::vector<std::string> files{
			"data002_room1.bin",
			"data002_room2.bin",
			"data002_room2_2.bin",
			"data002_room3.bin",
			"data002_room4.bin",
			"data002_room5.bin",
			"data002_room5_2.bin"
		};
		HistoryRepaPtrList ll;
		for (auto& f : files)
		{
			std::ifstream in(f, std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_2(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			ll.push_back(std::move(std::get<2>(xx)));
		}
		auto hr = vectorHistoryRepasConcat_u(ll);

		EVAL(hr->dimension);
		EVAL(hr->size);

		Variable motor("motor");
		Variable location("location");
		Variable position("position");
		auto vv = *uvars(*uu);
		auto vvk = VarUSet(vv);
		vvk.erase(motor);
		vvk.erase(location);
		vvk.erase(position);

		auto& vvi = ur->mapVarSize();
		auto vvk0 = sorted(vvk);
		SizeList vvk1;
		for (auto& v : vvk0)
			vvk1.push_back(vvi[v]);

		ApplicationRepa dr;
		{
			StrVarPtrMap m;
			std::ifstream in("model004.dr", std::ios::binary);
			auto ur1 = persistentsSystemRepa(in, m);
			auto dr1 = persistentsApplicationRepa(in);
			in.close();
			auto& llu1 = ur1->listVarSizePair;
			VarSizeUMap ur0 = ur->mapVarSize();
			auto n = fudRepasSize(*dr1->fud);
			size_t a = 360;
			size_t b = 60;
			auto& llu = ur->listVarSizePair;
			llu.reserve(n*a / b + a);
			dr.slices = std::make_shared<SizeTree>();
			dr.slices->_list.reserve(dr1->slices->_list.size() * a / b);
			dr.fud = std::make_shared<FudRepa>();
			dr.fud->layers.reserve(dr1->fud->layers.size());
			dr.substrate.reserve(dr1->substrate.size() * a / b);
			auto vframe = std::make_shared<Variable>("f");
			for (int i = 0; i < a * 2 / b; i++)
			{
				auto dr2 = drcopy(*dr1);
				SizeSizeUMap nn;
				nn.reserve(n + b);
				for (auto x1 : dr1->substrate)
				{
					auto& p = llu1[x1];
					auto v1 = p.first->_var0;
					auto v2 = std::make_shared<Variable>((int)(p.first->_var1->_int + i*b / 2 - 1));
					auto v = std::make_shared<Variable>(v1, v2);
					nn[x1] = ur0[*v];
				}
				auto v3 = std::make_shared<Variable>((int)i + 1);
				auto vd1 = std::make_shared<Variable>(vframe, v3);
				for (auto& ll : dr1->fud->layers)
					for (auto& tr : ll)
					{
						auto x1 = tr->derived;
						auto& p = llu1[x1];
						auto vdfl = p.first->_var0;
						auto vb = p.first->_var1;
						auto vdf = vdfl->_var0;
						auto vl = vdfl->_var1;
						auto vf = vdf->_var1;
						auto vdf1 = std::make_shared<Variable>(vd1, vf);
						auto vdfl1 = std::make_shared<Variable>(vdf1, vl);
						auto vdflb1 = std::make_shared<Variable>(vdfl1, vb);
						llu.push_back(VarSizePair(vdflb1, p.second));
						nn[x1] = llu.size() - 1;
					}
				dr2->reframe_u(nn);
				dr.slices->_list.insert(dr.slices->_list.end(), dr2->slices->_list.begin(), dr2->slices->_list.end());
				for (std::size_t l = 0; l < dr2->fud->layers.size(); l++)
				{
					if (l < dr.fud->layers.size())
						dr.fud->layers[l].insert(dr.fud->layers[l].end(), dr2->fud->layers[l].begin(), dr2->fud->layers[l].end());
					else
						dr.fud->layers.push_back(dr2->fud->layers[l]);
				}
				dr.substrate.insert(dr.substrate.end(), dr2->substrate.begin(), dr2->substrate.end());
			}
		}

		auto sl = treesElements(*dr.slices);
		auto hr1 = hrhrred(sl->size(), sl->data(), *frmul(*hr, *dr.fud));

		EVAL(hr1->dimension);
		EVAL(hr1->size);

		std::ofstream out("test.bin", std::ios::binary);
		ECHO(historyRepasPersistent(*hr1, out));
		out.close();

		auto hs1 = hrhs(*hr1);
		std::ofstream out1("test.hs", std::ios::binary);
		ECHO(historySparsesPersistent(*hs1, out1));
		out1.close();
	}
	
	if (argc >= 3 && string(argv[1]) == "entropy")
	{
		auto uvars = systemsSetVar;
		auto uruu = systemsRepasSystem;
		auto aall = histogramsList;
		auto add = pairHistogramsAdd_u;
		auto ent = histogramsEntropy;
		auto araa = systemsHistogramRepasHistogram_u;
		auto hrred = [](const HistoryRepa& hr, const SystemRepa& ur, const VarList& kk)
		{
			auto& vvi = ur.mapVarSize();
			std::size_t m = kk.size();
			SizeList kk1;
			for (std::size_t i = 0; i < m; i++)
				kk1.push_back(vvi[kk[i]]);
			return setVarsHistoryRepasReduce_u(1.0, m, kk1.data(), hr);
		};
		auto hrconcat = vectorHistoryRepasConcat_u;
		auto hrshuffle = historyRepasShuffle_u;
		auto hrpart = systemsHistoryRepasApplicationsHistoryHistoryPartitionedRepa_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		
		string model = string(argv[2]);
		size_t mult = argc >= 4 ? atoi(argv[3]) : 1;
		string dataset = string(argc >= 5 ? argv[4] : "data002");
		string substrate = string(argc >= 6 ? argv[5] : "substrate002");
		
		EVAL(model);
		EVAL(mult);
		EVAL(dataset);

		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
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
				if (substrate == "substrate004")
					xx = recordListsHistoryRepa_4(8, *qq);
				else if (substrate == "substrate003")
					xx = recordListsHistoryRepa_3(8, *qq);
				else
					xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		ECHO(auto z = hr->size);
		EVAL(z);
		ECHO(auto v = z * mult);
		EVAL(v);
		
		StrVarPtrMap m;
		std::ifstream in(model + ".dr", std::ios::binary);
		auto ur1 = persistentsSystemRepa(in, m);
		auto dr = persistentsApplicationRepa(in);
		in.close();

		EVAL(fudRepasSize(*dr->fud));
		EVAL(frder(*dr->fud)->size());
		EVAL(frund(*dr->fud)->size());
		EVAL(treesSize(*dr->slices));
		EVAL(treesLeafElements(*dr->slices)->size());
		auto d = treesLeafElements(*dr->slices)->size();
		// EVAL(d);

		auto hrp = hrpart(*hr, *dr, *ur);
		// EVAL(*hrp);
		uruu(*ur, *uu);
		// EVAL(*uu);
		auto aa = araa(*uu, *ur, *hrred(*hrp, *ur, VarList{ Variable("partition0"), Variable("partition1") }));
		// EVAL(*aa);
		// EVAL(ent(*aa));
		EVAL(ent(*aa) * z);
		// EVAL((1.0-exp(ent(*aa))/d)*100.0);
		
		HistoryRepaPtrList qq;
		qq.reserve(mult);
		for (std::size_t i = 1; i <= mult; i++)
			qq.push_back(hrshuffle(*hr, (unsigned int)(12345+i*z)));
		auto hrs = hrconcat(qq);
		
		auto hrsp = hrpart(*hrs, *dr, *ur);
		auto bb = araa(*uu, *ur, *hrred(*hrsp, *ur, VarList{ Variable("partition0"), Variable("partition1") }));
		// EVAL(ent(*bb));
		EVAL(ent(*bb) * v);
		// EVAL((1.0-exp(ent(*bb))/d)*100.0);
		
		auto cc = add(*aa,*bb);
		
		// EVAL(ent(*cc));
		// EVAL(ent(*cc) * (z+v));
		// EVAL((1.0-exp(ent(*cc))/d)*100.0);		
		EVAL(ent(*add(*aa,*bb)) * (z+v));

		// EVAL((ent(*cc) * (z+v) - ent(*aa) * z - ent(*bb) * v)/z);
		// EVAL(ent(*cc) * (z+v) - ent(*aa) * z - ent(*bb) * v);
		// EVAL(exp((ent(*cc) * (z+v) - ent(*aa) * z - ent(*bb) * v)/z)/d*100.0);
		EVAL(ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v);

	}
	
	if (argc >= 3 && string(argv[1]) == "entropy_region")
	{
		auto uvars = systemsSetVar;
		auto uruu = systemsRepasSystem;
		auto aall = histogramsList;
		auto add = pairHistogramsAdd_u;
		auto ent = histogramsEntropy;
		auto araa = systemsHistogramRepasHistogram_u;
		auto hrred = [](const HistoryRepa& hr, const SystemRepa& ur, const VarList& kk)
		{
			auto& vvi = ur.mapVarSize();
			std::size_t m = kk.size();
			SizeList kk1;
			for (std::size_t i = 0; i < m; i++)
				kk1.push_back(vvi[kk[i]]);
			return setVarsHistoryRepasReduce_u(1.0, m, kk1.data(), hr);
		};
		auto hrconcat = vectorHistoryRepasConcat_u;
		auto hrshuffle = historyRepasShuffle_u;
		auto hrpart = systemsHistoryRepasApplicationsHistoryHistoryPartitionedRepa_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		
		string model = string(argv[2]);
		size_t mult = argc >= 4 ? atoi(argv[3]) : 1;
		string dataset = string(argc >= 5 ? argv[4] : "data002");
		size_t scale = argc >= 6 ? atoi(argv[5]) : 10;
		
		EVAL(model);
		EVAL(mult);
		EVAL(dataset);

		std::unique_ptr<System> uu;
		std::unique_ptr<SystemRepa> ur;
		std::unique_ptr<HistoryRepa> hr;
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
			int s = 17;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				for (int i = 0; i < scale; i++)
				{
					auto xx = recordListsHistoryRepaRegion(8, 60, s++, *qq);
					uu = std::move(std::get<0>(xx));
					ur = std::move(std::get<1>(xx));
					ll.push_back(std::move(std::get<2>(xx)));
				}
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		ECHO(auto z = hr->size);
		EVAL(z);
		ECHO(auto v = z * mult);
		EVAL(v);
		
		StrVarPtrMap m;
		std::ifstream in(model + ".dr", std::ios::binary);
		auto ur1 = persistentsSystemRepa(in, m);
		auto dr = persistentsApplicationRepa(in);
		in.close();

		EVAL(fudRepasSize(*dr->fud));
		EVAL(frder(*dr->fud)->size());
		EVAL(frund(*dr->fud)->size());
		EVAL(treesSize(*dr->slices));
		EVAL(treesLeafElements(*dr->slices)->size());

		auto hrp = hrpart(*hr, *dr, *ur);
		uruu(*ur, *uu);
		auto aa = araa(*uu, *ur, *hrred(*hrp, *ur, VarList{ Variable("partition0"), Variable("partition1") }));
		EVAL(ent(*aa) * z);
		
		HistoryRepaPtrList qq;
		qq.reserve(mult);
		for (std::size_t i = 1; i <= mult; i++)
			qq.push_back(hrshuffle(*hr, (unsigned int)(12345+i*z)));
		auto hrs = hrconcat(qq);
		
		auto hrsp = hrpart(*hrs, *dr, *ur);
		auto bb = araa(*uu, *ur, *hrred(*hrsp, *ur, VarList{ Variable("partition0"), Variable("partition1") }));
		EVAL(ent(*bb) * v);
		
		EVAL(ent(*add(*aa,*bb)) * (z+v));
		EVAL(ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v);
	}
	
	if (argc >= 3 && string(argv[1]) == "induce" && string(argv[2]) == "model009")
	{
		auto uvars = systemsSetVar;
		auto applicationer = parametersSystemsFudRepasHistoryRepasApplicationerSubstrateEntropyMaxRollByMExcludedSelfHighestFmaxIORepa_p;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;

		std::vector<std::string> files{
			"data003.bin"
		};
		HistoryRepaPtrList ll;
		for (auto& f : files)
		{
			std::ifstream in(f, std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_2(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			ll.push_back(std::move(std::get<2>(xx)));
		}
		auto hr = vectorHistoryRepasConcat_u(ll);

		EVAL(hr->dimension);
		EVAL(hr->size);

		Variable motor("motor");
		Variable location("location");
		Variable position("position");
		auto vv = *uvars(*uu);
		auto vvk = VarUSet(vv);
		vvk.erase(motor);
		vvk.erase(location);
		vvk.erase(position);

		auto& vvi = ur->mapVarSize();
		auto vvk0 = sorted(vvk);
		SizeList vvk1;
		for (auto& v : vvk0)
			vvk1.push_back(vvi[v]);

		size_t wmax = 18;
		size_t lmax = 8;
		size_t xmax = 128;
		double znnmax = (double)hr->size * 2.0 * 300.0 * 300.0;
		size_t omax = 10;
		size_t bmax = 10 * 3;
		size_t mmax = 3;
		size_t umax = 128;
		size_t pmax = 1;
		size_t fmax = 127;
		size_t mult = 1;
		size_t seed = 5;
		auto dr = applicationer(wmax, lmax, xmax, znnmax, omax, bmax, mmax, umax, pmax, fmax, mult, 0, seed, tint, vvk1, FudRepa(), *hr, 0, *ur);
		std::ofstream out(model + ".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr, out); cout << endl;
		out.close();
	}
	
	if (argc >= 3 && string(argv[1]) == "induce" && string(argv[2]) == "model010")
	{
		auto uvars = systemsSetVar;
		auto applicationer = parametersSystemsFudRepasHistoryRepasApplicationerSubstrateEntropyMaxRollByMExcludedSelfHighestFmaxIORepa_p;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;

		std::vector<std::string> files{
			"data003.bin"
		};
		HistoryRepaPtrList ll;
		for (auto& f : files)
		{
			std::ifstream in(f, std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_2(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			ll.push_back(std::move(std::get<2>(xx)));
		}
		auto hr = vectorHistoryRepasConcat_u(ll);

		EVAL(hr->dimension);
		EVAL(hr->size);

		Variable motor("motor");
		Variable location("location");
		Variable position("position");
		auto vv = *uvars(*uu);
		auto vvk = VarUSet(vv);
		vvk.erase(motor);
		vvk.erase(location);
		vvk.erase(position);

		auto& vvi = ur->mapVarSize();
		auto vvk0 = sorted(vvk);
		SizeList vvk1;
		for (auto& v : vvk0)
			vvk1.push_back(vvi[v]);

		size_t wmax = 18;
		size_t lmax = 8;
		size_t xmax = 128;
		double znnmax = (double)hr->size * 2.0 * 300.0 * 300.0;
		size_t omax = 10;
		size_t bmax = 10 * 3;
		size_t mmax = 3;
		size_t umax = 128;
		size_t pmax = 1;
		size_t fmax = 127*4;
		size_t mult = 1;
		size_t seed = 5;
		auto dr = applicationer(wmax, lmax, xmax, znnmax, omax, bmax, mmax, umax, pmax, fmax, mult, 0, seed, tint, vvk1, FudRepa(), *hr, 0, *ur);
		std::ofstream out(model + ".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr, out); cout << endl;
		out.close();
	}
	
	if (argc >= 3 && string(argv[1]) == "induce" && string(argv[2]) == "model011")
	{
		auto uvars = systemsSetVar;
		auto applicationer = parametersSystemsFudRepasHistoryRepasApplicationerSubstrateEntropyMaxRollByMExcludedSelfHighestFmaxIORepa_p;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;

		std::vector<std::string> files{
			"data003.bin"
		};
		HistoryRepaPtrList ll;
		int s = 17;
		for (auto& f : files)
		{
			std::ifstream in(f, std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			for (int i = 0; i < 10; i++)
			{
				auto xx = recordListsHistoryRepaRegion(8, 60, s++, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
		}
		auto hr = vectorHistoryRepasConcat_u(ll);

		EVAL(hr->dimension);
		EVAL(hr->size);

		auto vvk = *uvars(*uu);

		auto& vvi = ur->mapVarSize();
		auto vvk0 = sorted(vvk);
		SizeList vvk1;
		for (auto& v : vvk0)
			vvk1.push_back(vvi[v]);

		size_t wmax = 9;
		size_t lmax = 8;
		size_t xmax = 128;
		double znnmax = (double)hr->size * 2.0 * 300.0 * 300.0;
		size_t omax = 10;
		size_t bmax = 10 * 3;
		size_t mmax = 3;
		size_t umax = 128;
		size_t pmax = 1;
		size_t fmax = 127;
		size_t mult = 1;
		size_t seed = 5;
		auto dr = applicationer(wmax, lmax, xmax, znnmax, omax, bmax, mmax, umax, pmax, fmax, mult, 0, seed, tint, vvk1, FudRepa(), *hr, 0, *ur);
		std::ofstream out(model+".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr, out); cout << endl;
		out.close();
	}
	
	if (argc >= 3 && string(argv[1]) == "induce" && string(argv[2]) == "model012")
	{
		auto uvars = systemsSetVar;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;
		auto applicationer = parametersSystemsFudRepasHistoryRepasApplicationerSubstrateEntropyMaxRollByMExcludedSelfHighestFmaxIORepa_p;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;

		std::vector<std::string> files{
			"data003.bin"
		};
		HistoryRepaPtrList ll;
		for (auto& f : files)
		{
			std::ifstream in(f, std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			auto xx = recordListsHistoryRepa_2(8, *qq);
			uu = std::move(std::get<0>(xx));
			ur = std::move(std::get<1>(xx));
			ll.push_back(std::move(std::get<2>(xx)));
		}
		auto hr = vectorHistoryRepasConcat_u(ll);

		EVAL(hr->dimension);
		EVAL(hr->size);

		Variable motor("motor");
		Variable location("location");
		Variable position("position");
		auto vv = *uvars(*uu);
		auto vvk = VarUSet(vv);
		vvk.erase(motor);
		vvk.erase(location);
		vvk.erase(position);

		auto& vvi = ur->mapVarSize();
		auto vvk0 = sorted(vvk);
		SizeList vvk1;
		for (auto& v : vvk0)
			vvk1.push_back(vvi[v]);

		ApplicationRepa dr;
		{
			StrVarPtrMap m;
			std::ifstream in("model011.dr", std::ios::binary);
			auto ur1 = persistentsSystemRepa(in, m);
			auto dr1 = persistentsApplicationRepa(in);
			in.close();
			auto& llu1 = ur1->listVarSizePair;
			VarSizeUMap ur0 = ur->mapVarSize();
			auto n = fudRepasSize(*dr1->fud);
			size_t a = 360;
			size_t b = 60;
			auto& llu = ur->listVarSizePair;
			llu.reserve(n*a/b + a);
			dr.slices = std::make_shared<SizeTree>();
			dr.slices->_list.reserve(dr1->slices->_list.size() * a / b);
			dr.fud = std::make_shared<FudRepa>();
			dr.fud->layers.reserve(dr1->fud->layers.size());
			dr.substrate.reserve(dr1->substrate.size() * a / b);
			auto vframe = std::make_shared<Variable>("f");
			for (int i = 0; i < a*2/b; i++)
			{
				auto dr2 = drcopy(*dr1);
				SizeSizeUMap nn;
				nn.reserve(n + b);
				for (auto x1 : dr1->substrate)
				{
					auto& p = llu1[x1];
					auto v1 = p.first->_var0;
					auto v2 = std::make_shared<Variable>(((int)(p.first->_var1->_int + i*b/2 - 1)) % a + 1);
					auto v = std::make_shared<Variable>(v1, v2);
					nn[x1] = ur0[*v];
				}
				auto v3 = std::make_shared<Variable>((int)i+1);
				auto vd1 = std::make_shared<Variable>(vframe, v3);
				for (auto& ll : dr1->fud->layers)
					for (auto& tr : ll)
					{
						auto x1 = tr->derived;
						auto& p = llu1[x1];
						auto vdfl = p.first->_var0;
						auto vb = p.first->_var1;
						auto vdf = vdfl->_var0;
						auto vl = vdfl->_var1;
						auto vf = vdf->_var1;
						auto vdf1 = std::make_shared<Variable>(vd1, vf);
						auto vdfl1 = std::make_shared<Variable>(vdf1, vl);
						auto vdflb1 = std::make_shared<Variable>(vdfl1, vb);
						llu.push_back(VarSizePair(vdflb1, p.second));
						nn[x1] = llu.size() - 1;
					}
				dr2->reframe_u(nn);
				dr.slices->_list.insert(dr.slices->_list.end(), dr2->slices->_list.begin(), dr2->slices->_list.end());
				for (std::size_t l = 0; l < dr2->fud->layers.size(); l++)
				{
					if (l < dr.fud->layers.size())
						dr.fud->layers[l].insert(dr.fud->layers[l].end(), dr2->fud->layers[l].begin(), dr2->fud->layers[l].end());
					else
						dr.fud->layers.push_back(dr2->fud->layers[l]);
				}
				dr.substrate.insert(dr.substrate.end(), dr2->substrate.begin(), dr2->substrate.end());
			}
		}

		size_t wmax = 18;
		size_t lmax = 8;
		size_t xmax = 128;
		double znnmax = 60000.0 * 2.0 * 100.0 * 100.0 * tint;
		size_t omax = 10;
		size_t bmax = 10 * 3;
		size_t mmax = 3;
		size_t umax = 128;
		size_t pmax = 1;
		size_t fmax = 127;
		size_t mult = 1;
		size_t seed = 5;
		auto sl = treesElements(*dr.slices);
		auto dr2 = applicationer(wmax, lmax, xmax, znnmax, omax, bmax, mmax, umax, pmax, fmax, mult, 0, seed, tint, *sl, *dr.fud, *hr, 0, *ur);
		auto dr3 = drjoin(dr, *dr2);
		std::ofstream out(model + ".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr3, out); cout << endl;
		out.close();
	}
	
	if (argc >= 3 && string(argv[1]) == "condition" && string(argv[2]) == "model013")
	{
		auto uvars = systemsSetVar;
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto frmul = historyRepasFudRepasMultiply_u;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;
		auto applicationer = parametersSystemsHistoryRepasApplicationerCondMultinomialFmaxIORepa_up;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;
		string label = argc >= 5 ? string(argv[4]) : "motor";

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
		std::unique_ptr<Alignment::HistoryRepa> hr;

		{
			std::vector<std::string> files{
				"data003.bin"
			};
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				auto xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);

		Variable motor("motor");
		Variable location("location");
		Variable position("position");
		auto vv = *uvars(*uu);
		auto vvk = VarUSet(vv);
		vvk.erase(motor);		
		vvk.erase(location);
		vvk.erase(position);

		auto& vvi = ur->mapVarSize();
		SizeList vvk1;
		for (auto& v : sorted(vvk))
			vvk1.push_back(vvi[v]);

		size_t fmax = 1024;
		auto dr = applicationer(fmax, tint, vvk1, vvi[Variable(label)], *hr, 1, *ur);
		std::ofstream out(model + "_" + label + ".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr, out); cout << endl;
		out.close();
	}
	
	if (argc >= 3 && string(argv[1]) == "condition" && string(argv[2]) == "model014")
	{
		auto uvars = systemsSetVar;
		auto frmul = historyRepasFudRepasMultiply_up;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;
		auto applicationer = parametersSystemsHistoryRepasApplicationerCondMultinomialFmaxIORepa_up;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;
		string label = argc >= 5 ? string(argv[4]) : "motor";

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
		std::unique_ptr<Alignment::HistoryRepa> hr;

		{
			std::vector<std::string> files{
				"data003.bin"
			};
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				auto xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);

		Variable motor("motor");
		Variable location("location");
		Variable position("position");
		auto vv = *uvars(*uu);
		auto vvk = VarUSet(vv);
		vvk.erase(motor);
		vvk.erase(location);
		vvk.erase(position);

		auto& vvi = ur->mapVarSize();
		SizeList vvk1;
		for (auto& v : sorted(vvk))
			vvk1.push_back(vvi[v]);

		ApplicationRepa dr;
		{
			StrVarPtrMap m;
			std::ifstream in("model011.dr", std::ios::binary);
			auto ur1 = persistentsSystemRepa(in, m);
			auto dr1 = persistentsApplicationRepa(in);
			in.close();
			auto& llu1 = ur1->listVarSizePair;
			VarSizeUMap ur0 = ur->mapVarSize();
			auto n = fudRepasSize(*dr1->fud);
			size_t a = 360;
			size_t b = 60;
			auto& llu = ur->listVarSizePair;
			llu.reserve(n*a / b + a);
			dr.slices = std::make_shared<SizeTree>();
			dr.slices->_list.reserve(dr1->slices->_list.size() * a / b);
			dr.fud = std::make_shared<FudRepa>();
			dr.fud->layers.reserve(dr1->fud->layers.size());
			dr.substrate.reserve(dr1->substrate.size() * a / b);
			auto vframe = std::make_shared<Variable>("f");
			for (int i = 0; i < a * 2 / b; i++)
			{
				auto dr2 = drcopy(*dr1);
				SizeSizeUMap nn;
				nn.reserve(n + b);
				for (auto x1 : dr1->substrate)
				{
					auto& p = llu1[x1];
					auto v1 = p.first->_var0;
					auto v2 = std::make_shared<Variable>((int)(p.first->_var1->_int + i*b / 2 - 1));
					auto v = std::make_shared<Variable>(v1, v2);
					nn[x1] = ur0[*v];
				}
				auto v3 = std::make_shared<Variable>((int)i + 1);
				auto vd1 = std::make_shared<Variable>(vframe, v3);
				for (auto& ll : dr1->fud->layers)
					for (auto& tr : ll)
					{
						auto x1 = tr->derived;
						auto& p = llu1[x1];
						auto vdfl = p.first->_var0;
						auto vb = p.first->_var1;
						auto vdf = vdfl->_var0;
						auto vl = vdfl->_var1;
						auto vf = vdf->_var1;
						auto vdf1 = std::make_shared<Variable>(vd1, vf);
						auto vdfl1 = std::make_shared<Variable>(vdf1, vl);
						auto vdflb1 = std::make_shared<Variable>(vdfl1, vb);
						llu.push_back(VarSizePair(vdflb1, p.second));
						nn[x1] = llu.size() - 1;
					}
				dr2->reframe_u(nn);
				dr.slices->_list.insert(dr.slices->_list.end(), dr2->slices->_list.begin(), dr2->slices->_list.end());
				for (std::size_t l = 0; l < dr2->fud->layers.size(); l++)
				{
					if (l < dr.fud->layers.size())
						dr.fud->layers[l].insert(dr.fud->layers[l].end(), dr2->fud->layers[l].begin(), dr2->fud->layers[l].end());
					else
						dr.fud->layers.push_back(dr2->fud->layers[l]);
				}
				dr.substrate.insert(dr.substrate.end(), dr2->substrate.begin(), dr2->substrate.end());
			}
		}

		auto hr1 = frmul(tint, *hr, *dr.fud);
		auto sl = treesElements(*dr.slices);
		size_t fmax = 1024;
		auto dr2 = applicationer(fmax, tint, *sl, vvi[Variable(label)], *hr1, 1, *ur);
		auto dr3 = drjoin(dr, *dr2);
		std::ofstream out(model + "_" + label + ".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr3, out); cout << endl;
		out.close();
	}
	
	if (argc >= 3 && string(argv[1]) == "induce" && string(argv[2]) == "model015")
	{
		auto uvars = systemsSetVar;
		auto applicationer = parametersSystemsFudRepasHistoryRepasApplicationerSubstrateEntropyMaxRollByMExcludedSelfHighestFmaxIORepa_p;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;

		std::vector<std::string> files{
			"data003.bin",
			"data004_01.bin",
			"data004_02.bin",
			"data004_03.bin",
			"data004_04.bin",
			"data004_05.bin"
		};
		HistoryRepaPtrList ll;
		int s = 17;
		for (auto& f : files)
		{
			std::ifstream in(f, std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			for (int i = 0; i < 10; i++)
			{
				auto xx = recordListsHistoryRepaRegion(8, 60, s++, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
		}
		auto hr = vectorHistoryRepasConcat_u(ll);

		EVAL(hr->dimension);
		EVAL(hr->size);

		auto vvk = *uvars(*uu);

		auto& vvi = ur->mapVarSize();
		auto vvk0 = sorted(vvk);
		SizeList vvk1;
		for (auto& v : vvk0)
			vvk1.push_back(vvi[v]);

		size_t wmax = 9;
		size_t lmax = 8;
		size_t xmax = 128;
		double znnmax = (double)hr->size * 2.0 * 300.0 * 300.0;
		size_t omax = 10;
		size_t bmax = 10 * 3;
		size_t mmax = 3;
		size_t umax = 128;
		size_t pmax = 1;
		size_t fmax = 384;
		size_t mult = 1;
		size_t seed = 5;
		auto dr = applicationer(wmax, lmax, xmax, znnmax, omax, bmax, mmax, umax, pmax, fmax, mult, 0, seed, tint, vvk1, FudRepa(), *hr, 0, *ur);
		std::ofstream out(model+".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr, out); cout << endl;
		out.close();
	}
	
	if (argc >= 3 && string(argv[1]) == "condition" && string(argv[2]) == "model016")
	{
		auto uvars = systemsSetVar;
		auto frmul = historyRepasFudRepasMultiply_up;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;
		auto applicationer = parametersSystemsHistoryRepasApplicationerCondMultinomialFmaxIORepa_up;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;
		string label = argc >= 5 ? string(argv[4]) : "motor";

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
		std::unique_ptr<Alignment::HistoryRepa> hr;

		{
			std::vector<std::string> files{
				"data003.bin",
				"data004_01.bin",
				"data004_02.bin",
				"data004_03.bin",
				"data004_04.bin",
				"data004_05.bin"
			};
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				auto xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);

		Variable motor("motor");
		Variable location("location");
		Variable position("position");
		auto vv = *uvars(*uu);
		auto vvk = VarUSet(vv);
		vvk.erase(motor);
		vvk.erase(location);
		vvk.erase(position);

		auto& vvi = ur->mapVarSize();
		SizeList vvk1;
		for (auto& v : sorted(vvk))
			vvk1.push_back(vvi[v]);

		ApplicationRepa dr;
		{
			StrVarPtrMap m;
			std::ifstream in("model015.dr", std::ios::binary);
			auto ur1 = persistentsSystemRepa(in, m);
			auto dr1 = persistentsApplicationRepa(in);
			in.close();
			auto& llu1 = ur1->listVarSizePair;
			VarSizeUMap ur0 = ur->mapVarSize();
			auto n = fudRepasSize(*dr1->fud);
			size_t a = 360;
			size_t b = 60;
			auto& llu = ur->listVarSizePair;
			llu.reserve(n*a / b + a);
			dr.slices = std::make_shared<SizeTree>();
			dr.slices->_list.reserve(dr1->slices->_list.size() * a / b);
			dr.fud = std::make_shared<FudRepa>();
			dr.fud->layers.reserve(dr1->fud->layers.size());
			dr.substrate.reserve(dr1->substrate.size() * a / b);
			auto vframe = std::make_shared<Variable>("f");
			for (int i = 0; i < a * 2 / b; i++)
			{
				auto dr2 = drcopy(*dr1);
				SizeSizeUMap nn;
				nn.reserve(n + b);
				for (auto x1 : dr1->substrate)
				{
					auto& p = llu1[x1];
					auto v1 = p.first->_var0;
					auto v2 = std::make_shared<Variable>((int)(p.first->_var1->_int + i*b / 2 - 1));
					auto v = std::make_shared<Variable>(v1, v2);
					nn[x1] = ur0[*v];
				}
				auto v3 = std::make_shared<Variable>((int)i + 1);
				auto vd1 = std::make_shared<Variable>(vframe, v3);
				for (auto& ll : dr1->fud->layers)
					for (auto& tr : ll)
					{
						auto x1 = tr->derived;
						auto& p = llu1[x1];
						auto vdfl = p.first->_var0;
						auto vb = p.first->_var1;
						auto vdf = vdfl->_var0;
						auto vl = vdfl->_var1;
						auto vf = vdf->_var1;
						auto vdf1 = std::make_shared<Variable>(vd1, vf);
						auto vdfl1 = std::make_shared<Variable>(vdf1, vl);
						auto vdflb1 = std::make_shared<Variable>(vdfl1, vb);
						llu.push_back(VarSizePair(vdflb1, p.second));
						nn[x1] = llu.size() - 1;
					}
				dr2->reframe_u(nn);
				dr.slices->_list.insert(dr.slices->_list.end(), dr2->slices->_list.begin(), dr2->slices->_list.end());
				for (std::size_t l = 0; l < dr2->fud->layers.size(); l++)
				{
					if (l < dr.fud->layers.size())
						dr.fud->layers[l].insert(dr.fud->layers[l].end(), dr2->fud->layers[l].begin(), dr2->fud->layers[l].end());
					else
						dr.fud->layers.push_back(dr2->fud->layers[l]);
				}
				dr.substrate.insert(dr.substrate.end(), dr2->substrate.begin(), dr2->substrate.end());
			}
		}

		auto hr1 = frmul(tint, *hr, *dr.fud);
		auto sl = treesElements(*dr.slices);
		size_t fmax = 4096;
		auto dr2 = applicationer(fmax, tint, *sl, vvi[Variable(label)], *hr1, 1, *ur);
		auto dr3 = drjoin(dr, *dr2);
		std::ofstream out(model + "_" + label + ".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr3, out); cout << endl;
		out.close();
	}
	
	if (argc >= 3 && string(argv[1]) == "observe_bitmap")
	{
		auto uvars = systemsSetVar;
		auto single = histogramSingleton_u;
		auto aahr = [](const System& uu, const SystemRepa& ur, const Histogram& aa)
		{
			return systemsHistoriesHistoryRepa_u(uu, ur, *histogramsHistory_u(aa));
		};
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrhrred = setVarsHistoryRepasHistoryRepaReduced_u;
		auto hrred = setVarsHistoryRepasReduce_u;
		auto frmul = historyRepasFudRepasMultiply_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto frdep = fudRepasSetVarsDepends;
		auto hrbm = historyRepasBitmapAverage;

		string records = string(argv[2]);
		string model = string(argv[3]);
		string dataset = string(argc >= 5 ? argv[4] : "data002");
		size_t scale_vertical = argc >= 6 ? atoi(argv[5]) : 1;
		size_t event_start = argc >= 7 ? atoi(argv[6]) : 0;
		size_t event_end = argc >= 8 ? atoi(argv[7]) : 0;

		std::map<std::size_t, Bitmap> sbm;
	
		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
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
				auto xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		Variable motor("motor");
		Variable location("location");
		Variable position("position");
		auto vv = *uvars(*uu);
		auto vvk = VarUSet(vv);
		vvk.erase(motor);
		vvk.erase(location);
		vvk.erase(position);

		auto& vvi = ur->mapVarSize();
		auto vvk0 = sorted(vvk);
		SizeList vvk1;
		for (auto& v : vvk0)
			vvk1.push_back(vvi[v]);

		std::unique_ptr<Alignment::SystemRepa> ur1;
		std::unique_ptr<Alignment::ApplicationRepa> dr;
		
		{
			StrVarPtrMap m;
			std::ifstream in(model + ".dr", std::ios::binary);
			ur1 = persistentsSystemRepa(in, m);
			dr = persistentsApplicationRepa(in);
			in.close();

			auto hr1 = frmul(*hr, *dr->fud);
			auto hr2 = hrhrred(vvk1.size(), vvk1.data(), *hr);
			if (hr1->evient)
				hr1->transpose();
			auto z = hr1->size;
			auto& mvv = hr1->mapVarInt();
			auto sl = treesLeafElements(*dr->slices);
			for (auto s : *sl)
			{
				SizeList ev;
				auto pk = mvv[s];
				for (std::size_t j = 0; j < z; j++)
				{
					std::size_t u = hr1->arr[pk*z + j];
					if (u)
						ev.push_back(j);
				}
				if (ev.size())
					sbm[s] = historyRepasBitmapAverage(scale_vertical,8,*hrsel(ev.size(), ev.data(), *hr2));
				else
					sbm[s] = Bitmap(scale_vertical,360);
			}
		}
		{
			std::ifstream in(records + ".bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			
			auto xx = recordListsHistoryRepa_2(8, *qq);
			hr = std::move(std::get<2>(xx));
			if (event_end > 0)
			{
				SizeList ev;
				for (size_t i = event_start; i <= event_end; i++)
					ev.push_back(i);
				hr = hrsel(ev.size(), ev.data(), *hr);
			}
			auto sl = treesLeafElements(*dr->slices);
			auto hr1 = hrhrred(sl->size(), sl->data(), *frmul(*hr, *dr->fud));
			if (!hr1->evient)
				hr1->transpose();	
			auto z = hr1->size;
			auto n = hr1->dimension;
			auto vv = hr1->vectorVar;
			auto sh = hr1->shape;
			auto rr = hr1->arr;
			std::vector<Bitmap> ll;
			for (std::size_t j = 0; j < z; j++)
				for (std::size_t i = 0; i < n; i++)
				{
					std::size_t u = rr[j*n + i];
					if (u)
					{
						ll.push_back(sbm[vv[i]]);
						break;
					}
				}
			bmwrite(records + "_" + model + "_" + dataset + ".bmp", bmvstack(ll));
		}
	}
	
	if (argc >= 3 && string(argv[1]) == "induce" && string(argv[2]) == "model017")
	{
		auto uvars = systemsSetVar;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;
		auto applicationer = parametersSystemsFudRepasHistoryRepasApplicationerSubstrateEntropyMaxRollByMExcludedSelfHighestFmaxIORepa_p;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
		std::unique_ptr<Alignment::HistoryRepa> hr;
		
		{
			std::vector<std::string> files{
				"data003.bin",
				"data004_01.bin",
				"data004_02.bin",
				"data004_03.bin",
				"data004_04.bin",
				"data004_05.bin"
			};
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				auto xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);

		Variable motor("motor");
		Variable location("location");
		Variable position("position");
		auto vv = *uvars(*uu);
		auto vvk = VarUSet(vv);
		vvk.erase(motor);
		vvk.erase(location);
		vvk.erase(position);

		auto& vvi = ur->mapVarSize();
		auto vvk0 = sorted(vvk);
		SizeList vvk1;
		for (auto& v : vvk0)
			vvk1.push_back(vvi[v]);

		ApplicationRepa dr;
		{
			StrVarPtrMap m;
			std::ifstream in("model015.dr", std::ios::binary);
			auto ur1 = persistentsSystemRepa(in, m);
			auto dr1 = persistentsApplicationRepa(in);
			in.close();
			auto& llu1 = ur1->listVarSizePair;
			VarSizeUMap ur0 = ur->mapVarSize();
			auto n = fudRepasSize(*dr1->fud);
			size_t a = 360;
			size_t b = 60;
			auto& llu = ur->listVarSizePair;
			llu.reserve(n*a/b + a);
			dr.slices = std::make_shared<SizeTree>();
			dr.slices->_list.reserve(dr1->slices->_list.size() * a / b);
			dr.fud = std::make_shared<FudRepa>();
			dr.fud->layers.reserve(dr1->fud->layers.size());
			dr.substrate.reserve(dr1->substrate.size() * a / b);
			auto vframe = std::make_shared<Variable>("f");
			for (int i = 0; i < a*2/b; i++)
			{
				auto dr2 = drcopy(*dr1);
				SizeSizeUMap nn;
				nn.reserve(n + b);
				for (auto x1 : dr1->substrate)
				{
					auto& p = llu1[x1];
					auto v1 = p.first->_var0;
					auto v2 = std::make_shared<Variable>(((int)(p.first->_var1->_int + i*b/2 - 1)) % a + 1);
					auto v = std::make_shared<Variable>(v1, v2);
					nn[x1] = ur0[*v];
				}
				auto v3 = std::make_shared<Variable>((int)i+1);
				auto vd1 = std::make_shared<Variable>(vframe, v3);
				for (auto& ll : dr1->fud->layers)
					for (auto& tr : ll)
					{
						auto x1 = tr->derived;
						auto& p = llu1[x1];
						auto vdfl = p.first->_var0;
						auto vb = p.first->_var1;
						auto vdf = vdfl->_var0;
						auto vl = vdfl->_var1;
						auto vf = vdf->_var1;
						auto vdf1 = std::make_shared<Variable>(vd1, vf);
						auto vdfl1 = std::make_shared<Variable>(vdf1, vl);
						auto vdflb1 = std::make_shared<Variable>(vdfl1, vb);
						llu.push_back(VarSizePair(vdflb1, p.second));
						nn[x1] = llu.size() - 1;
					}
				dr2->reframe_u(nn);
				dr.slices->_list.insert(dr.slices->_list.end(), dr2->slices->_list.begin(), dr2->slices->_list.end());
				for (std::size_t l = 0; l < dr2->fud->layers.size(); l++)
				{
					if (l < dr.fud->layers.size())
						dr.fud->layers[l].insert(dr.fud->layers[l].end(), dr2->fud->layers[l].begin(), dr2->fud->layers[l].end());
					else
						dr.fud->layers.push_back(dr2->fud->layers[l]);
				}
				dr.substrate.insert(dr.substrate.end(), dr2->substrate.begin(), dr2->substrate.end());
			}
		}

		size_t wmax = 18;
		size_t lmax = 8;
		size_t xmax = 128;
		double znnmax = 60000.0 * 2.0 * 100.0 * 100.0 * tint;
		size_t omax = 10;
		size_t bmax = 10 * 3;
		size_t mmax = 3;
		size_t umax = 128;
		size_t pmax = 1;
		size_t fmax = 4096;
		size_t mult = 1;
		size_t seed = 5;
		auto sl = treesElements(*dr.slices);
		auto dr2 = applicationer(wmax, lmax, xmax, znnmax, omax, bmax, mmax, umax, pmax, fmax, mult, 0, seed, tint, *sl, *dr.fud, *hr, 0, *ur);
		auto dr3 = drjoin(dr, *dr2);
		std::ofstream out(model + ".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr3, out); cout << endl;
		out.close();
	}

	if (argc >= 3 && string(argv[1]) == "observe")
	{
		auto uvars = systemsSetVar;
		auto single = histogramSingleton_u;
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrhrred = setVarsHistoryRepasHistoryRepaReduced_u;
		auto hrred = setVarsHistoryRepasReduce_u;
		auto frmul = historyRepasFudRepasMultiply_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto frdep = fudRepasSetVarsDepends;
		auto hrbm = historyRepasBitmapAverage;

		string records = string(argv[2]);
		string model = string(argv[3]);
		string dataset = string(argc >= 5 ? argv[4] : "data002");
		string label = string(argc >= 6 ? argv[5] : "location");
		size_t event_start = argc >= 7 ? atoi(argv[6]) : 0;
		size_t event_end = argc >= 8 ? atoi(argv[7]) : 0;
		string substrate = string(argc >= 9 ? argv[8] : "substrate002");		
		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
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
				if (substrate == "substrate004")
					xx = recordListsHistoryRepa_4(8, *qq);
				else if (substrate == "substrate003")
					xx = recordListsHistoryRepa_3(8, *qq);
				else
					xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		std::unique_ptr<Alignment::SystemRepa> ur1;
		std::unique_ptr<Alignment::ApplicationRepa> dr;	
		std::map<std::size_t, std::size_t> su;
		std::size_t pl;	
		{
			auto& vvi = ur->mapVarSize();
			
			StrVarPtrMap m;
			std::ifstream in(model + ".dr", std::ios::binary);
			ur1 = persistentsSystemRepa(in, m);
			dr = persistentsApplicationRepa(in);
			in.close();

			auto hr1 = frmul(*hr, *dr->fud);
			if (hr1->evient)
				hr1->transpose();
			auto z = hr1->size;
			auto& mvv = hr1->mapVarInt();
			auto sh = hr1->shape;
			auto rr = hr1->arr;
			pl = mvv[vvi[Variable(label)]];
			auto sl = sh[pl];
			auto nn = treesLeafNodes(*dr->slices);
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
						std::size_t w = rr[pl*z + j];
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
				su[s.first] = cl;
			}
		}
		
		{
			std::ifstream in(records + ".bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			SystemHistoryRepaTuple xx;
			if (substrate == "substrate004")
				xx = recordListsHistoryRepa_4(8, *qq);
			else if (substrate == "substrate003")
				xx = recordListsHistoryRepa_3(8, *qq);
			else
				xx = recordListsHistoryRepa_2(8, *qq);		
			hr = std::move(std::get<2>(xx));		
		}
		if (event_end > 0)
		{
			SizeList ev;
			for (size_t i = event_start; i <= event_end; i++)
				ev.push_back(i);
			hr = hrsel(ev.size(), ev.data(), *hr);
		}
		std::vector<std::string> locations{ 
			"door12", "door13", "door14", "door45", "door56", 
			"room1", "room2", "room3", "room4", "room5", "room6", "unknown" };
		std::vector<std::string> positions{ "centre", "corner", "side", "unknown" };
		SizeList ww{ pl };
		auto nn = treesLeafNodes(*dr->slices);
		for (auto& s : *nn)
			ww.push_back(s.first);
		auto hr1 = hrhrred(ww.size(), ww.data(), *frmul(*hr, *dr->fud));
		if (!hr1->evient)
			hr1->transpose();
		auto z = hr1->size;
		auto n = hr1->dimension;
		auto vv = hr1->vectorVar;
		auto sh = hr1->shape;
		auto rr = hr1->arr;
		auto sl = sh[0];
		SizeSet slice_unique;
		std::size_t consecutive_unique_count = 0;
		std::size_t slice_last = 0;
		std::size_t match_count = 0;
		cout << "event|slice|location|guess|match?" << endl;
		cout << "---|---|---|---|---" << endl;
		for (std::size_t j = 0; j < z; j++)
		{
			std::size_t cl = rr[j*n];
			std::size_t l = sl;
			std::size_t i = 1;
			for (; i < n; i++)
			{
				std::size_t u = rr[j*n + i];
				if (u)
				{
					auto s = vv[i];
					l = su[s];
					if (slice_last != s)
					{
						consecutive_unique_count++;
						slice_last = s;
						slice_unique.insert(s);
					}
					break;
				}
			}
			if (l == cl)
				match_count++;
			cout << j;
			if (i<n)
				cout << "|" << *ur1->listVarSizePair[vv[i]].first;
			else
				cout << ",no var";
			cout << "|" << (label == "location" || "location_next" || "room_next" ? locations[cl] : positions[cl]);
			cout << "|" << (label == "location" || "location_next" || "room_next" ? locations[l] : positions[l]);
			cout << "|" << (l == cl ? "match" : "fail") << endl;
		}
		EVAL(z);
		EVAL(slice_unique.size());
		EVAL(consecutive_unique_count);
		EVAL(match_count);
		EVAL(100.0*match_count/z);
	}
	
	if (argc >= 3 && string(argv[1]) == "induce" && string(argv[2]) == "model018")
	{
		auto uvars = systemsSetVar;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;
		auto applicationer = parametersSystemsFudRepasHistoryRepasApplicationerSubstrateEntropyMaxRollByMExcludedSelfHighestFmaxIORepa_p;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
		std::unique_ptr<Alignment::HistoryRepa> hr;
		
		{
			std::vector<std::string> files{
				"data003.bin",
				"data004_01.bin",
				"data004_02.bin",
				"data004_03.bin",
				"data004_04.bin",
				"data004_05.bin"
			};
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				auto xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);

		Variable motor("motor");
		Variable location("location");
		Variable position("position");
		auto vv = *uvars(*uu);
		auto vvk = VarUSet(vv);
		vvk.erase(motor);
		vvk.erase(location);
		vvk.erase(position);

		auto& vvi = ur->mapVarSize();
		auto vvk0 = sorted(vvk);
		SizeList vvk1;
		for (auto& v : vvk0)
			vvk1.push_back(vvi[v]);

		ApplicationRepa dr;
		{
			StrVarPtrMap m;
			std::ifstream in("model011.dr", std::ios::binary);
			auto ur1 = persistentsSystemRepa(in, m);
			auto dr1 = persistentsApplicationRepa(in);
			in.close();
			auto& llu1 = ur1->listVarSizePair;
			VarSizeUMap ur0 = ur->mapVarSize();
			auto n = fudRepasSize(*dr1->fud);
			size_t a = 360;
			size_t b = 60;
			auto& llu = ur->listVarSizePair;
			llu.reserve(n*a/b + a);
			dr.slices = std::make_shared<SizeTree>();
			dr.slices->_list.reserve(dr1->slices->_list.size() * a / b);
			dr.fud = std::make_shared<FudRepa>();
			dr.fud->layers.reserve(dr1->fud->layers.size());
			dr.substrate.reserve(dr1->substrate.size() * a / b);
			auto vframe = std::make_shared<Variable>("f");
			for (int i = 0; i < a*2/b; i++)
			{
				auto dr2 = drcopy(*dr1);
				SizeSizeUMap nn;
				nn.reserve(n + b);
				for (auto x1 : dr1->substrate)
				{
					auto& p = llu1[x1];
					auto v1 = p.first->_var0;
					auto v2 = std::make_shared<Variable>(((int)(p.first->_var1->_int + i*b/2 - 1)) % a + 1);
					auto v = std::make_shared<Variable>(v1, v2);
					nn[x1] = ur0[*v];
				}
				auto v3 = std::make_shared<Variable>((int)i+1);
				auto vd1 = std::make_shared<Variable>(vframe, v3);
				for (auto& ll : dr1->fud->layers)
					for (auto& tr : ll)
					{
						auto x1 = tr->derived;
						auto& p = llu1[x1];
						auto vdfl = p.first->_var0;
						auto vb = p.first->_var1;
						auto vdf = vdfl->_var0;
						auto vl = vdfl->_var1;
						auto vf = vdf->_var1;
						auto vdf1 = std::make_shared<Variable>(vd1, vf);
						auto vdfl1 = std::make_shared<Variable>(vdf1, vl);
						auto vdflb1 = std::make_shared<Variable>(vdfl1, vb);
						llu.push_back(VarSizePair(vdflb1, p.second));
						nn[x1] = llu.size() - 1;
					}
				dr2->reframe_u(nn);
				dr.slices->_list.insert(dr.slices->_list.end(), dr2->slices->_list.begin(), dr2->slices->_list.end());
				for (std::size_t l = 0; l < dr2->fud->layers.size(); l++)
				{
					if (l < dr.fud->layers.size())
						dr.fud->layers[l].insert(dr.fud->layers[l].end(), dr2->fud->layers[l].begin(), dr2->fud->layers[l].end());
					else
						dr.fud->layers.push_back(dr2->fud->layers[l]);
				}
				dr.substrate.insert(dr.substrate.end(), dr2->substrate.begin(), dr2->substrate.end());
			}
		}

		size_t wmax = 18;
		size_t lmax = 8;
		size_t xmax = 128;
		double znnmax = 60000.0 * 2.0 * 100.0 * 100.0 * tint;
		size_t omax = 10;
		size_t bmax = 10 * 3;
		size_t mmax = 3;
		size_t umax = 128;
		size_t pmax = 1;
		size_t fmax = 4096;
		size_t mult = 1;
		size_t seed = 5;
		auto sl = treesElements(*dr.slices);
		auto dr2 = applicationer(wmax, lmax, xmax, znnmax, omax, bmax, mmax, umax, pmax, fmax, mult, 0, seed, tint, *sl, *dr.fud, *hr, 0, *ur);
		auto dr3 = drjoin(dr, *dr2);
		std::ofstream out(model + ".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr3, out); cout << endl;
		out.close();
	}
	
	if (argc >= 3 && string(argv[1]) == "condition" && string(argv[2]) == "model019")
	{
		auto uvars = systemsSetVar;
		auto frmul = historyRepasFudRepasMultiply_up;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;
		auto applicationer = parametersSystemsHistoryRepasApplicationerCondMultinomialFmaxIORepa_up;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;
		string label = argc >= 5 ? string(argv[4]) : "motor";

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
		std::unique_ptr<Alignment::HistoryRepa> hr;

		{
			std::vector<std::string> files{
				"data003.bin",
				"data004_01.bin",
				"data004_02.bin",
				"data004_03.bin",
				"data004_04.bin",
				"data004_05.bin"
			};
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				auto xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);

		Variable motor("motor");
		Variable location("location");
		Variable position("position");
		auto vv = *uvars(*uu);
		auto vvk = VarUSet(vv);
		vvk.erase(motor);
		vvk.erase(location);
		vvk.erase(position);

		auto& vvi = ur->mapVarSize();
		SizeList vvk1;
		for (auto& v : sorted(vvk))
			vvk1.push_back(vvi[v]);

		ApplicationRepa dr;
		{
			StrVarPtrMap m;
			std::ifstream in("model011.dr", std::ios::binary);
			auto ur1 = persistentsSystemRepa(in, m);
			auto dr1 = persistentsApplicationRepa(in);
			in.close();
			auto& llu1 = ur1->listVarSizePair;
			VarSizeUMap ur0 = ur->mapVarSize();
			auto n = fudRepasSize(*dr1->fud);
			size_t a = 360;
			size_t b = 60;
			auto& llu = ur->listVarSizePair;
			llu.reserve(n*a / b + a);
			dr.slices = std::make_shared<SizeTree>();
			dr.slices->_list.reserve(dr1->slices->_list.size() * a / b);
			dr.fud = std::make_shared<FudRepa>();
			dr.fud->layers.reserve(dr1->fud->layers.size());
			dr.substrate.reserve(dr1->substrate.size() * a / b);
			auto vframe = std::make_shared<Variable>("f");
			for (int i = 0; i < a * 2 / b; i++)
			{
				auto dr2 = drcopy(*dr1);
				SizeSizeUMap nn;
				nn.reserve(n + b);
				for (auto x1 : dr1->substrate)
				{
					auto& p = llu1[x1];
					auto v1 = p.first->_var0;
					auto v2 = std::make_shared<Variable>((int)(p.first->_var1->_int + i*b / 2 - 1));
					// TODO - re-run with corrected last frame
					//auto v2 = std::make_shared<Variable>(((int)(p.first->_var1->_int + i*b/2 - 1)) % a + 1);
					auto v = std::make_shared<Variable>(v1, v2);
					nn[x1] = ur0[*v];
				}
				auto v3 = std::make_shared<Variable>((int)i + 1);
				auto vd1 = std::make_shared<Variable>(vframe, v3);
				for (auto& ll : dr1->fud->layers)
					for (auto& tr : ll)
					{
						auto x1 = tr->derived;
						auto& p = llu1[x1];
						auto vdfl = p.first->_var0;
						auto vb = p.first->_var1;
						auto vdf = vdfl->_var0;
						auto vl = vdfl->_var1;
						auto vf = vdf->_var1;
						auto vdf1 = std::make_shared<Variable>(vd1, vf);
						auto vdfl1 = std::make_shared<Variable>(vdf1, vl);
						auto vdflb1 = std::make_shared<Variable>(vdfl1, vb);
						llu.push_back(VarSizePair(vdflb1, p.second));
						nn[x1] = llu.size() - 1;
					}
				dr2->reframe_u(nn);
				dr.slices->_list.insert(dr.slices->_list.end(), dr2->slices->_list.begin(), dr2->slices->_list.end());
				for (std::size_t l = 0; l < dr2->fud->layers.size(); l++)
				{
					if (l < dr.fud->layers.size())
						dr.fud->layers[l].insert(dr.fud->layers[l].end(), dr2->fud->layers[l].begin(), dr2->fud->layers[l].end());
					else
						dr.fud->layers.push_back(dr2->fud->layers[l]);
				}
				dr.substrate.insert(dr.substrate.end(), dr2->substrate.begin(), dr2->substrate.end());
			}
		}

		auto hr1 = frmul(tint, *hr, *dr.fud);
		auto sl = treesElements(*dr.slices);
		size_t fmax = 4096;
		auto dr2 = applicationer(fmax, tint, *sl, vvi[Variable(label)], *hr1, 1, *ur);
		auto dr3 = drjoin(dr, *dr2);
		std::ofstream out(model + "_" + label + ".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr3, out); cout << endl;
		out.close();
	}
	
	if (argc >= 3 && string(argv[1]) == "condition" && string(argv[2]) == "model020")
	{
		auto uvars = systemsSetVar;
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrjoin = vectorHistoryRepasJoin_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto frmul = historyRepasFudRepasMultiply_up;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;		
		auto applicationer = parametersSystemsHistoryRepasApplicationerCondMultinomialFmaxIORepa_up;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;
		string label = argc >= 5 ? string(argv[4]) : "motor";
		size_t sequence_count = argc >= 6 ? atoi(argv[5]) : 1;
		size_t sequence_interval = argc >= 7 ? atoi(argv[6]) : 1;
		
		EVAL(model);
		EVAL(tint);
		EVAL(label);
		EVAL(sequence_count);
		EVAL(sequence_interval);
		
		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
		std::unique_ptr<Alignment::HistoryRepa> hr;
		{
			std::vector<std::string> files{
				"data003.bin",
				"data004_01.bin",
				"data004_02.bin",
				"data004_03.bin",
				"data004_04.bin",
				"data004_05.bin"
			};
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				auto xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);
		
		auto& llu = ur->listVarSizePair;
		auto n0 = llu.size();
		
		ApplicationRepa dr;
		{
			StrVarPtrMap m;
			std::ifstream in("model011.dr", std::ios::binary);
			auto ur1 = persistentsSystemRepa(in, m);
			auto dr1 = persistentsApplicationRepa(in);
			in.close();
			auto& llu1 = ur1->listVarSizePair;
			VarSizeUMap ur0 = ur->mapVarSize();
			auto n = fudRepasSize(*dr1->fud);
			size_t a = 360;
			size_t b = 60;
			llu.reserve(n*a / b + a);
			dr.slices = std::make_shared<SizeTree>();
			dr.slices->_list.reserve(dr1->slices->_list.size() * a / b);
			dr.fud = std::make_shared<FudRepa>();
			dr.fud->layers.reserve(dr1->fud->layers.size());
			dr.substrate.reserve(dr1->substrate.size() * a / b);
			auto vframe = std::make_shared<Variable>("f");
			for (int i = 0; i < a * 2 / b; i++)
			{
				auto dr2 = drcopy(*dr1);
				SizeSizeUMap nn;
				nn.reserve(n + b);
				for (auto x1 : dr1->substrate)
				{
					auto& p = llu1[x1];
					auto v1 = p.first->_var0;
					auto v2 = std::make_shared<Variable>((int)(p.first->_var1->_int + i*b / 2 - 1));
					auto v = std::make_shared<Variable>(v1, v2);
					nn[x1] = ur0[*v];
				}
				auto v3 = std::make_shared<Variable>((int)i + 1);
				auto vd1 = std::make_shared<Variable>(vframe, v3);
				for (auto& ll : dr1->fud->layers)
					for (auto& tr : ll)
					{
						auto x1 = tr->derived;
						auto& p = llu1[x1];
						auto vdfl = p.first->_var0;
						auto vb = p.first->_var1;
						auto vdf = vdfl->_var0;
						auto vl = vdfl->_var1;
						auto vf = vdf->_var1;
						auto vdf1 = std::make_shared<Variable>(vd1, vf);
						auto vdfl1 = std::make_shared<Variable>(vdf1, vl);
						auto vdflb1 = std::make_shared<Variable>(vdfl1, vb);
						llu.push_back(VarSizePair(vdflb1, p.second));
						nn[x1] = llu.size() - 1;
					}
				dr2->reframe_u(nn);
				dr.slices->_list.insert(dr.slices->_list.end(), dr2->slices->_list.begin(), dr2->slices->_list.end());
				for (std::size_t l = 0; l < dr2->fud->layers.size(); l++)
				{
					if (l < dr.fud->layers.size())
						dr.fud->layers[l].insert(dr.fud->layers[l].end(), dr2->fud->layers[l].begin(), dr2->fud->layers[l].end());
					else
						dr.fud->layers.push_back(dr2->fud->layers[l]);
				}
				dr.substrate.insert(dr.substrate.end(), dr2->substrate.begin(), dr2->substrate.end());
			}
		}
		
		auto ur1 = std::make_unique<SystemRepa>();
		auto& llu1 = ur1->listVarSizePair;	
		std::unique_ptr<HistoryRepa> hr1;
		{
			auto z = hr->size;
			HistoryRepaPtrList llh;
			auto vframe = std::make_shared<Variable>("t");
			for (size_t k = 0; k < sequence_count*sequence_interval; k += sequence_interval)
			{
				auto vk = std::make_shared<Variable>(vframe, std::make_shared<Variable>((int)k));
				SizeSizeUMap nn;
				nn.reserve(n0);
				for (std::size_t i = 0; i < n0; i++)
				{
					auto& p = llu[i];
					auto v = std::make_shared<Variable>(vk, p.first);
					llu1.push_back(VarSizePair(v, p.second));
					nn[i] = llu1.size() - 1;
				}
				SizeList ll;
				for (size_t j = (sequence_count-1)*sequence_interval-k; j < z-k; j++)
					ll.push_back(j);
				auto hr0 = hrsel(ll.size(), ll.data(), *hr);
				hr0->reframe_u(nn);
				llh.push_back(std::move(hr0));
			}
			hr1 = hrjoin(llh);
		}
		
		EVAL(hr1->dimension);
		EVAL(hr1->size);

		std::vector<std::shared_ptr<ApplicationRepa>> lld;
		{
			auto z = hr->size;
			HistoryRepaPtrList llh;
			auto vframe = std::make_shared<Variable>("t");
			for (size_t k = 0; k < sequence_count*sequence_interval; k += sequence_interval)
			{
				auto vk = std::make_shared<Variable>(vframe, std::make_shared<Variable>((int)k));
				SizeSizeUMap nn;
				nn.reserve(llu.size());
				for (std::size_t i = 0; i < n0; i++)
					nn[i] = k / sequence_interval * n0 + i;
				for (std::size_t i = n0; i < llu.size(); i++)
				{
					auto& p = llu[i];
					auto v = std::make_shared<Variable>(vk, p.first);
					llu1.push_back(VarSizePair(v, p.second));
					nn[i] = llu1.size() - 1;
				}
				auto dr0 = drcopy(dr);
				dr0->reframe_u(nn);
				lld.push_back(std::move(dr0));
			}
		}
		
		ApplicationRepa dr1;
		{
			dr1.slices = std::make_shared<SizeTree>();
			dr1.slices->_list.reserve(dr.slices->_list.size() * sequence_count);
			dr1.fud = std::make_shared<FudRepa>();
			dr1.fud->layers.reserve(dr.fud->layers.size());
			dr1.substrate.reserve(dr.substrate.size() * sequence_count);
			for (auto& dr0 : lld)
			{
				dr1.slices->_list.insert(dr1.slices->_list.end(), dr0->slices->_list.begin(), dr0->slices->_list.end());
				dr1.fud->layers.insert(dr1.fud->layers.end(), dr0->fud->layers.begin(), dr0->fud->layers.end());
				dr1.substrate.insert(dr1.substrate.end(), dr0->substrate.begin(), dr0->substrate.end());
			}
		}
		EVAL(treesSize(*dr1.slices));
		EVAL(treesLeafElements(*dr1.slices)->size());
		EVAL(frder(*dr1.fud)->size());
		EVAL(frund(*dr1.fud)->size());
		EVAL(frvars(*dr1.fud)->size());	
		
		auto hr2 = frmul(tint, *hr1, *dr1.fud);

		EVAL(hr2->dimension);
		EVAL(hr2->size);
		
		auto sl = treesElements(*dr1.slices);
		auto vt = std::make_shared<Variable>(std::make_shared<Variable>("t"),std::make_shared<Variable>(0));
		auto vl = std::make_shared<Variable>(label);
		auto l = ur1->mapVarSize()[Variable(vt,vl)];
		size_t fmax = 4096;
		auto dr2 = applicationer(fmax, tint, *sl, l, *hr2, 1, *ur1);
		auto dr3 = drjoin(dr1, *dr2);
		std::ofstream out(model + "_" + label + "_" + std::to_string(sequence_count) + "_" + std::to_string(sequence_interval) + ".dr", std::ios::binary);
		systemRepasPersistent(*ur1, out); cout << endl;
		applicationRepasPersistent(*dr3, out); cout << endl;
		out.close();
	}
	
	if (argc >= 3 && string(argv[1]) == "entropy_sequence")
	{
		auto uvars = systemsSetVar;
		auto uruu = systemsRepasSystem;
		auto aall = histogramsList;
		auto add = pairHistogramsAdd_u;
		auto ent = histogramsEntropy;
		auto araa = systemsHistogramRepasHistogram_u;
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrjoin = vectorHistoryRepasJoin_u;
		auto hrred = [](const HistoryRepa& hr, const SystemRepa& ur, const VarList& kk)
		{
			auto& vvi = ur.mapVarSize();
			std::size_t m = kk.size();
			SizeList kk1;
			for (std::size_t i = 0; i < m; i++)
				kk1.push_back(vvi[kk[i]]);
			return setVarsHistoryRepasReduce_u(1.0, m, kk1.data(), hr);
		};
		auto hrconcat = vectorHistoryRepasConcat_u;
		auto hrshuffle = historyRepasShuffle_u;
		auto hrpart = systemsHistoryRepasApplicationsHistoryHistoryPartitionedRepa_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		
		string model = string(argv[2]);
		size_t mult = argc >= 4 ? atoi(argv[3]) : 1;
		string dataset = string(argc >= 5 ? argv[4] : "data002");
		size_t sequence_count = argc >= 6 ? atoi(argv[5]) : 1;
		size_t sequence_interval = argc >= 7 ? atoi(argv[6]) : 1;
		
		EVAL(model);
		EVAL(mult);
		EVAL(dataset);
		EVAL(sequence_count);
		EVAL(sequence_interval);

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
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
				auto xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}
		
		std::unique_ptr<Alignment::HistoryRepa> hrs;
		{
			HistoryRepaPtrList qq;
			qq.reserve(mult);
			for (std::size_t i = 1; i <= mult; i++)
				qq.push_back(hrshuffle(*hr, (unsigned int)(12345+i*hr->size)));
			hrs = hrconcat(qq);
		}
		
		auto& llu = ur->listVarSizePair;
		auto n0 = llu.size();			
		{
			auto ur1 = std::make_unique<SystemRepa>();
			auto& llu1 = ur1->listVarSizePair;	
			auto z = hr->size;
			HistoryRepaPtrList llh;
			HistoryRepaPtrList llhs;
			auto vframe = std::make_shared<Variable>("t");
			for (size_t k = 0; k < sequence_count*sequence_interval; k+=sequence_interval)
			{
				auto vk = std::make_shared<Variable>(vframe, std::make_shared<Variable>((int)k));
				if (model.substr(0,8)=="model025")
					vk = std::make_shared<Variable>(std::string("t") + std::to_string(k));
				SizeSizeUMap nn;
				nn.reserve(n0);
				for (std::size_t i = 0; i < n0; i++)
				{
					auto& p = llu[i];
					auto v = std::make_shared<Variable>(vk, p.first);
					if (model.substr(0,8)=="model025")
						v = std::make_shared<Variable>(p.first, vk);
					llu1.push_back(VarSizePair(v, p.second));
					nn[i] = llu1.size() - 1;
				}
				SizeList ll;
				for (size_t j = (sequence_count-1)*sequence_interval-k; j < z-k; j++)
					ll.push_back(j);
				auto hr0 = hrsel(ll.size(), ll.data(), *hr);
				hr0->reframe_u(nn);
				llh.push_back(std::move(hr0));
				hr0 = hrsel(ll.size(), ll.data(), *hrs);
				hr0->reframe_u(nn);
				llhs.push_back(std::move(hr0));
			}
			hr = hrjoin(llh);
			hrs = hrjoin(llhs);
		}
		
		ECHO(auto z = hr->size);
		EVAL(z);
		ECHO(auto v = z * mult);
		EVAL(v);
		
		StrVarPtrMap m;
		std::ifstream in(model + ".dr", std::ios::binary);
		auto ur1 = persistentsSystemRepa(in, m);
		auto dr = persistentsApplicationRepa(in);
		in.close();

		EVAL(fudRepasSize(*dr->fud));
		EVAL(frder(*dr->fud)->size());
		EVAL(frund(*dr->fud)->size());
		EVAL(treesSize(*dr->slices));
		EVAL(treesLeafElements(*dr->slices)->size());

		auto hrp = hrpart(*hr, *dr, *ur1);
		uruu(*ur1, *uu);
		auto aa = araa(*uu, *ur1, *hrred(*hrp, *ur1, VarList{ Variable("partition0"), Variable("partition1") }));
		EVAL(ent(*aa) * z);
		auto hrsp = hrpart(*hrs, *dr, *ur1);
		auto bb = araa(*uu, *ur1, *hrred(*hrsp, *ur1, VarList{ Variable("partition0"), Variable("partition1") }));
		EVAL(ent(*bb) * v);
		EVAL(ent(*add(*aa,*bb)) * (z+v));
		EVAL(ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v);
	}
	
	if (argc >= 3 && string(argv[1]) == "observe_sequence")
	{
		auto uvars = systemsSetVar;
		auto single = histogramSingleton_u;
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrhrred = setVarsHistoryRepasHistoryRepaReduced_u;
		auto hrred = setVarsHistoryRepasReduce_u;
		auto hrjoin = vectorHistoryRepasJoin_u;
		auto frmul = historyRepasFudRepasMultiply_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto frdep = fudRepasSetVarsDepends;

		string records = string(argv[2]);
		string model = string(argv[3]);
		string dataset = string(argc >= 5 ? argv[4] : "data002");
		string label = string(argc >= 6 ? argv[5] : "location");
		size_t sequence_count = argc >= 7 ? atoi(argv[6]) : 1;
		size_t sequence_interval = argc >= 8 ? atoi(argv[7]) : 1;	
		size_t event_start = argc >= 9 ? atoi(argv[8]) : 0;
		size_t event_end = argc >= 10 ? atoi(argv[9]) : 0;
		
		EVAL(records);
		EVAL(model);
		EVAL(dataset);
		EVAL(label);
		EVAL(sequence_count);
		EVAL(sequence_interval);			
		EVAL(event_start);
		EVAL(event_end);
		
		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
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
				auto xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		auto& llu = ur->listVarSizePair;
		auto n0 = llu.size();			
		{
			auto ur1 = std::make_unique<SystemRepa>();
			auto& llu1 = ur1->listVarSizePair;	
			auto z = hr->size;
			HistoryRepaPtrList llh;
			auto vframe = std::make_shared<Variable>("t");
			for (size_t k = 0; k < sequence_count*sequence_interval; k+=sequence_interval)
			{
				auto vk = std::make_shared<Variable>(vframe, std::make_shared<Variable>((int)k));
				if (model.substr(0,8)=="model025")
					vk = std::make_shared<Variable>(std::string("t") + std::to_string(k));				
				SizeSizeUMap nn;
				nn.reserve(n0);
				for (std::size_t i = 0; i < n0; i++)
				{
					auto& p = llu[i];
					auto v = std::make_shared<Variable>(vk, p.first);
					if (model.substr(0,8)=="model025")
						v = std::make_shared<Variable>(p.first, vk);
					llu1.push_back(VarSizePair(v, p.second));
					nn[i] = llu1.size() - 1;
				}
				SizeList ll;
				for (size_t j = (sequence_count-1)*sequence_interval-k; j < z-k; j++)
					ll.push_back(j);
				auto hr0 = hrsel(ll.size(), ll.data(), *hr);
				hr0->reframe_u(nn);
				llh.push_back(std::move(hr0));
			}
			hr = hrjoin(llh);
		}
		
		std::unique_ptr<Alignment::SystemRepa> ur1;
		std::unique_ptr<Alignment::ApplicationRepa> dr;	
		std::map<std::size_t, std::size_t> su;
		std::size_t pl;		
		{
			auto& vvi = ur->mapVarSize();
			
			StrVarPtrMap m;
			std::ifstream in(model + ".dr", std::ios::binary);
			ur1 = persistentsSystemRepa(in, m);
			dr = persistentsApplicationRepa(in);
			in.close();

			auto hr1 = frmul(*hr, *dr->fud);
			if (hr1->evient)
				hr1->transpose();
			auto z = hr1->size;
			auto& mvv = hr1->mapVarInt();
			auto sh = hr1->shape;
			auto rr = hr1->arr;
			pl = mvv[vvi[Variable(label)]];
			auto sl = sh[pl];
			auto nn = treesLeafNodes(*dr->slices);
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
						std::size_t w = rr[pl*z + j];
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
				su[s.first] = cl;
			}
		}
		{
			std::ifstream in(records + ".bin", std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			
			auto xx = recordListsHistoryRepa_2(8, *qq);
			hr = std::move(std::get<2>(xx));
			{
				auto ur1 = std::make_unique<SystemRepa>();
				auto& llu1 = ur1->listVarSizePair;	
				auto z = hr->size;
				HistoryRepaPtrList llh;
				auto vframe = std::make_shared<Variable>("t");
				for (size_t k = 0; k < sequence_count*sequence_interval; k+=sequence_interval)
				{
					auto vk = std::make_shared<Variable>(vframe, std::make_shared<Variable>((int)k));
				if (model.substr(0,8)=="model025")
					vk = std::make_shared<Variable>(std::string("t") + std::to_string(k));					
					SizeSizeUMap nn;
					nn.reserve(n0);
					for (std::size_t i = 0; i < n0; i++)
					{
						auto& p = llu[i];
						auto v = std::make_shared<Variable>(vk, p.first);
						if (model.substr(0,8)=="model025")
							v = std::make_shared<Variable>(p.first, vk);
						llu1.push_back(VarSizePair(v, p.second));
						nn[i] = llu1.size() - 1;
					}
					SizeList ll;
					for (size_t j = (sequence_count-1)*sequence_interval-k; j < z-k; j++)
						ll.push_back(j);
					auto hr0 = hrsel(ll.size(), ll.data(), *hr);
					hr0->reframe_u(nn);
					llh.push_back(std::move(hr0));
				}
				hr = hrjoin(llh);
			}
			if (event_end > 0)
			{
				SizeList ev;
				for (size_t i = event_start; i <= event_end; i++)
					ev.push_back(i);
				hr = hrsel(ev.size(), ev.data(), *hr);
			}
			std::vector<std::string> locations{ 
				"door12", "door13", "door14", "door45", "door56", 
				"room1", "room2", "room3", "room4", "room5", "room6", "unknown" };
			std::vector<std::string> positions{ "centre", "corner", "side", "unknown" };
			SizeList ww{ pl };
			auto nn = treesLeafNodes(*dr->slices);
			for (auto& s : *nn)
				ww.push_back(s.first);
			auto hr1 = hrhrred(ww.size(), ww.data(), *frmul(*hr, *dr->fud));
			if (!hr1->evient)
				hr1->transpose();
			auto z = hr1->size;
			auto n = hr1->dimension;
			auto vv = hr1->vectorVar;
			auto sh = hr1->shape;
			auto rr = hr1->arr;
			auto sl = sh[0];
			SizeSet slice_unique;
			std::size_t consecutive_unique_count = 0;
			std::size_t slice_last = 0;
			std::size_t match_count = 0;
			cout << "event|slice|location|guess|match?" << endl;
			cout << "---|---|---|---|---" << endl;
			for (std::size_t j = 0; j < z; j++)
			{
				std::size_t cl = rr[j*n];
				std::size_t l = sl;
				std::size_t i = 1;
				for (; i < n; i++)
				{
					std::size_t u = rr[j*n + i];
					if (u)
					{
						auto s = vv[i];
						l = su[s];
						if (slice_last != s)
						{
							consecutive_unique_count++;
							slice_last = s;
							slice_unique.insert(s);
						}
						break;
					}
				}
				if (l == cl)
					match_count++;
				cout << j;
				if (i<n)
					cout << "|" << *ur1->listVarSizePair[vv[i]].first;
				else
					cout << ",no var";
				cout << "|" << (label == "location" ? locations[cl] : positions[cl]);
				cout << "|" << (label == "location" ? locations[l] : positions[l]);
				cout << "|" << (l == cl ? "match" : "fail") << endl;
			}
			EVAL(z);
			EVAL(slice_unique.size());
			EVAL(consecutive_unique_count);
			EVAL(match_count);
			EVAL(100.0*match_count/z);
		}
	}
	
	if (argc >= 3 && string(argv[1]) == "induce" && string(argv[2]) == "model021")
	{
		auto uvars = systemsSetVar;
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrjoin = vectorHistoryRepasJoin_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto frmul = historyRepasFudRepasMultiply_up;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;		
		auto applicationer = parametersSystemsFudRepasHistoryRepasApplicationerSubstrateEntropyMaxRollByMExcludedSelfHighestFmaxIORepa_p;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;
		size_t sequence_count = argc >= 5 ? atoi(argv[4]) : 1;
		size_t sequence_interval = argc >= 6 ? atoi(argv[5]) : 1;
		
		EVAL(model);
		EVAL(tint);
		EVAL(sequence_count);
		EVAL(sequence_interval);
		
		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
		std::unique_ptr<Alignment::HistoryRepa> hr;
		{
			std::vector<std::string> files{
				"data003.bin",
				"data004_01.bin",
				"data004_02.bin",
				"data004_03.bin",
				"data004_04.bin",
				"data004_05.bin"
			};
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				auto xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);
		
		auto& llu = ur->listVarSizePair;
		auto n0 = llu.size();
		
		ApplicationRepa dr;
		{
			StrVarPtrMap m;
			std::ifstream in("model011.dr", std::ios::binary);
			auto ur1 = persistentsSystemRepa(in, m);
			auto dr1 = persistentsApplicationRepa(in);
			in.close();
			auto& llu1 = ur1->listVarSizePair;
			VarSizeUMap ur0 = ur->mapVarSize();
			auto n = fudRepasSize(*dr1->fud);
			size_t a = 360;
			size_t b = 60;
			llu.reserve(n*a / b + a);
			dr.slices = std::make_shared<SizeTree>();
			dr.slices->_list.reserve(dr1->slices->_list.size() * a / b);
			dr.fud = std::make_shared<FudRepa>();
			dr.fud->layers.reserve(dr1->fud->layers.size());
			dr.substrate.reserve(dr1->substrate.size() * a / b);
			auto vframe = std::make_shared<Variable>("f");
			for (int i = 0; i < a * 2 / b; i++)
			{
				auto dr2 = drcopy(*dr1);
				SizeSizeUMap nn;
				nn.reserve(n + b);
				for (auto x1 : dr1->substrate)
				{
					auto& p = llu1[x1];
					auto v1 = p.first->_var0;
					auto v2 = std::make_shared<Variable>((int)(p.first->_var1->_int + i*b / 2 - 1));
					auto v = std::make_shared<Variable>(v1, v2);
					nn[x1] = ur0[*v];
				}
				auto v3 = std::make_shared<Variable>((int)i + 1);
				auto vd1 = std::make_shared<Variable>(vframe, v3);
				for (auto& ll : dr1->fud->layers)
					for (auto& tr : ll)
					{
						auto x1 = tr->derived;
						auto& p = llu1[x1];
						auto vdfl = p.first->_var0;
						auto vb = p.first->_var1;
						auto vdf = vdfl->_var0;
						auto vl = vdfl->_var1;
						auto vf = vdf->_var1;
						auto vdf1 = std::make_shared<Variable>(vd1, vf);
						auto vdfl1 = std::make_shared<Variable>(vdf1, vl);
						auto vdflb1 = std::make_shared<Variable>(vdfl1, vb);
						llu.push_back(VarSizePair(vdflb1, p.second));
						nn[x1] = llu.size() - 1;
					}
				dr2->reframe_u(nn);
				dr.slices->_list.insert(dr.slices->_list.end(), dr2->slices->_list.begin(), dr2->slices->_list.end());
				for (std::size_t l = 0; l < dr2->fud->layers.size(); l++)
				{
					if (l < dr.fud->layers.size())
						dr.fud->layers[l].insert(dr.fud->layers[l].end(), dr2->fud->layers[l].begin(), dr2->fud->layers[l].end());
					else
						dr.fud->layers.push_back(dr2->fud->layers[l]);
				}
				dr.substrate.insert(dr.substrate.end(), dr2->substrate.begin(), dr2->substrate.end());
			}
		}
		
		auto ur1 = std::make_unique<SystemRepa>();
		auto& llu1 = ur1->listVarSizePair;	
		std::unique_ptr<HistoryRepa> hr1;
		{
			auto z = hr->size;
			HistoryRepaPtrList llh;
			auto vframe = std::make_shared<Variable>("t");
			for (size_t k = 0; k < sequence_count*sequence_interval; k += sequence_interval)
			{
				auto vk = std::make_shared<Variable>(vframe, std::make_shared<Variable>((int)k));
				SizeSizeUMap nn;
				nn.reserve(n0);
				for (std::size_t i = 0; i < n0; i++)
				{
					auto& p = llu[i];
					auto v = std::make_shared<Variable>(vk, p.first);
					llu1.push_back(VarSizePair(v, p.second));
					nn[i] = llu1.size() - 1;
				}
				SizeList ll;
				for (size_t j = (sequence_count-1)*sequence_interval-k; j < z-k; j++)
					ll.push_back(j);
				auto hr0 = hrsel(ll.size(), ll.data(), *hr);
				hr0->reframe_u(nn);
				llh.push_back(std::move(hr0));
			}
			hr1 = hrjoin(llh);
		}
		
		EVAL(hr1->dimension);
		EVAL(hr1->size);

		std::vector<std::shared_ptr<ApplicationRepa>> lld;
		{
			auto z = hr->size;
			HistoryRepaPtrList llh;
			auto vframe = std::make_shared<Variable>("t");
			for (size_t k = 0; k < sequence_count*sequence_interval; k += sequence_interval)
			{
				auto vk = std::make_shared<Variable>(vframe, std::make_shared<Variable>((int)k));
				SizeSizeUMap nn;
				nn.reserve(llu.size());
				for (std::size_t i = 0; i < n0; i++)
					nn[i] = k / sequence_interval * n0 + i;
				for (std::size_t i = n0; i < llu.size(); i++)
				{
					auto& p = llu[i];
					auto v = std::make_shared<Variable>(vk, p.first);
					llu1.push_back(VarSizePair(v, p.second));
					nn[i] = llu1.size() - 1;
				}
				auto dr0 = drcopy(dr);
				dr0->reframe_u(nn);
				lld.push_back(std::move(dr0));
			}
		}
		
		ApplicationRepa dr1;
		{
			dr1.slices = std::make_shared<SizeTree>();
			dr1.slices->_list.reserve(dr.slices->_list.size() * sequence_count);
			dr1.fud = std::make_shared<FudRepa>();
			dr1.fud->layers.reserve(dr.fud->layers.size());
			dr1.substrate.reserve(dr.substrate.size() * sequence_count);
			for (auto& dr0 : lld)
			{
				dr1.slices->_list.insert(dr1.slices->_list.end(), dr0->slices->_list.begin(), dr0->slices->_list.end());
				dr1.fud->layers.insert(dr1.fud->layers.end(), dr0->fud->layers.begin(), dr0->fud->layers.end());
				dr1.substrate.insert(dr1.substrate.end(), dr0->substrate.begin(), dr0->substrate.end());
			}
		}
		EVAL(treesSize(*dr1.slices));
		EVAL(treesLeafElements(*dr1.slices)->size());
		EVAL(frder(*dr1.fud)->size());
		EVAL(frund(*dr1.fud)->size());
		EVAL(frvars(*dr1.fud)->size());	
		
		size_t wmax = 18;
		size_t lmax = 8;
		size_t xmax = 128;
		double znnmax = 60000.0 * 2.0 * 100.0 * 100.0 * tint;
		size_t omax = 10;
		size_t bmax = 10 * 3;
		size_t mmax = 3;
		size_t umax = 128;
		size_t pmax = 1;
		size_t fmax = 4096;
		size_t mult = 1;
		size_t seed = 5;
		auto sl = treesElements(*dr1.slices);
		auto dr2 = applicationer(wmax, lmax, xmax, znnmax, omax, bmax, mmax, umax, pmax, fmax, mult, 0, seed, tint, *sl, *dr1.fud, *hr1, 0, *ur1);
		auto dr3 = drjoin(dr1, *dr2);
		std::ofstream out(model + "_" + std::to_string(sequence_count) + "_" + std::to_string(sequence_interval) + ".dr", std::ios::binary);
		systemRepasPersistent(*ur1, out); cout << endl;
		applicationRepasPersistent(*dr3, out); cout << endl;
		out.close();
	}
	
	if (argc >= 3 && string(argv[1]) == "condition" && string(argv[2]) == "model022")
	{
		auto uvars = systemsSetVar;
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrjoin = vectorHistoryRepasJoin_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto frmul = historyRepasFudRepasMultiply_up;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;		
		auto applicationer = parametersSystemsHistoryRepasApplicationerCondMultinomialFmaxIORepa_up;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;
		string label = argc >= 5 ? string(argv[4]) : "motor";
		size_t sequence_count = argc >= 6 ? atoi(argv[5]) : 1;
		size_t sequence_interval = argc >= 7 ? atoi(argv[6]) : 1;
		
		EVAL(model);
		EVAL(tint);
		EVAL(label);
		EVAL(sequence_count);
		EVAL(sequence_interval);
		
		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
		std::unique_ptr<Alignment::HistoryRepa> hr;
		{
			std::vector<std::string> files{
				"data003.bin",
				"data004_01.bin",
				"data004_02.bin",
				"data004_03.bin",
				"data004_04.bin",
				"data004_05.bin"
			};
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				auto xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);
		
		auto n0 = ur->listVarSizePair.size();		
		std::unique_ptr<Alignment::ApplicationRepa> dr;
		{
			StrVarPtrMap m;
			std::ifstream in("model018.dr", std::ios::binary);
			ur = persistentsSystemRepa(in, m);
			dr = persistentsApplicationRepa(in);
			in.close();
		}
		
		auto& llu = ur->listVarSizePair;
		auto ur1 = std::make_unique<SystemRepa>();
		auto& llu1 = ur1->listVarSizePair;	
		std::unique_ptr<HistoryRepa> hr1;
		{
			auto z = hr->size;
			HistoryRepaPtrList llh;
			auto vframe = std::make_shared<Variable>("t");
			for (size_t k = 0; k < sequence_count*sequence_interval; k += sequence_interval)
			{
				auto vk = std::make_shared<Variable>(vframe, std::make_shared<Variable>((int)k));
				SizeSizeUMap nn;
				nn.reserve(n0);
				for (std::size_t i = 0; i < n0; i++)
				{
					auto& p = llu[i];
					auto v = std::make_shared<Variable>(vk, p.first);
					llu1.push_back(VarSizePair(v, p.second));
					nn[i] = llu1.size() - 1;
				}
				SizeList ll;
				for (size_t j = (sequence_count-1)*sequence_interval-k; j < z-k; j++)
					ll.push_back(j);
				auto hr0 = hrsel(ll.size(), ll.data(), *hr);
				hr0->reframe_u(nn);
				llh.push_back(std::move(hr0));
			}
			hr1 = hrjoin(llh);
		}
		
		EVAL(hr1->dimension);
		EVAL(hr1->size);

		std::vector<std::shared_ptr<ApplicationRepa>> lld;
		{
			auto z = hr->size;
			HistoryRepaPtrList llh;
			auto vframe = std::make_shared<Variable>("t");
			for (size_t k = 0; k < sequence_count*sequence_interval; k += sequence_interval)
			{
				auto vk = std::make_shared<Variable>(vframe, std::make_shared<Variable>((int)k));
				SizeSizeUMap nn;
				nn.reserve(llu.size());
				for (std::size_t i = 0; i < n0; i++)
					nn[i] = k / sequence_interval * n0 + i;
				for (std::size_t i = n0; i < llu.size(); i++)
				{
					auto& p = llu[i];
					auto v = std::make_shared<Variable>(vk, p.first);
					llu1.push_back(VarSizePair(v, p.second));
					nn[i] = llu1.size() - 1;
				}
				auto dr0 = drcopy(*dr);
				dr0->reframe_u(nn);
				lld.push_back(std::move(dr0));
			}
		}
		
		ApplicationRepa dr1;
		{
			dr1.slices = std::make_shared<SizeTree>();
			dr1.slices->_list.reserve(dr->slices->_list.size() * sequence_count);
			dr1.fud = std::make_shared<FudRepa>();
			dr1.fud->layers.reserve(dr->fud->layers.size());
			dr1.substrate.reserve(dr->substrate.size() * sequence_count);
			for (auto& dr0 : lld)
			{
				dr1.slices->_list.insert(dr1.slices->_list.end(), dr0->slices->_list.begin(), dr0->slices->_list.end());
				dr1.fud->layers.insert(dr1.fud->layers.end(), dr0->fud->layers.begin(), dr0->fud->layers.end());
				dr1.substrate.insert(dr1.substrate.end(), dr0->substrate.begin(), dr0->substrate.end());
			}
		}
		EVAL(treesSize(*dr1.slices));
		EVAL(treesLeafElements(*dr1.slices)->size());
		EVAL(frder(*dr1.fud)->size());
		EVAL(frund(*dr1.fud)->size());
		EVAL(frvars(*dr1.fud)->size());	
		
		auto hr2 = frmul(tint, *hr1, *dr1.fud);

		EVAL(hr2->dimension);
		EVAL(hr2->size);
		
		auto sl = treesElements(*dr1.slices);
		auto vt = std::make_shared<Variable>(std::make_shared<Variable>("t"),std::make_shared<Variable>(0));
		auto vl = std::make_shared<Variable>(label);
		auto l = ur1->mapVarSize()[Variable(vt,vl)];
		size_t fmax = 4096;
		auto dr2 = applicationer(fmax, tint, *sl, l, *hr2, 1, *ur1);
		auto dr3 = drjoin(dr1, *dr2);
		std::ofstream out(model + "_" + label + "_" + std::to_string(sequence_count) + "_" + std::to_string(sequence_interval) + ".dr", std::ios::binary);
		systemRepasPersistent(*ur1, out); cout << endl;
		applicationRepasPersistent(*dr3, out); cout << endl;
		out.close();
	}
	
	if (argc >= 3 && string(argv[1]) == "induce" && string(argv[2]) == "model023")
	{
		auto uvars = systemsSetVar;
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrjoin = vectorHistoryRepasJoin_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto frmul = historyRepasFudRepasMultiply_up;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;		
		auto applicationer = parametersSystemsFudRepasHistoryRepasApplicationerSubstrateEntropyMaxRollByMExcludedSelfHighestFmaxIORepa_p;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;
		size_t sequence_count = argc >= 5 ? atoi(argv[4]) : 1;
		size_t sequence_interval = argc >= 6 ? atoi(argv[5]) : 1;
		
		EVAL(model);
		EVAL(tint);
		EVAL(sequence_count);
		EVAL(sequence_interval);
		
		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
		std::unique_ptr<Alignment::HistoryRepa> hr;
		{
			std::vector<std::string> files{
				"data003.bin",
				"data004_01.bin",
				"data004_02.bin",
				"data004_03.bin",
				"data004_04.bin",
				"data004_05.bin"
			};
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				auto xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);
		
		auto n0 = ur->listVarSizePair.size();		
		std::unique_ptr<Alignment::ApplicationRepa> dr;
		{
			StrVarPtrMap m;
			std::ifstream in("model018.dr", std::ios::binary);
			ur = persistentsSystemRepa(in, m);
			dr = persistentsApplicationRepa(in);
			in.close();
		}
		
		auto& llu = ur->listVarSizePair;
		auto ur1 = std::make_unique<SystemRepa>();
		auto& llu1 = ur1->listVarSizePair;	
		std::unique_ptr<HistoryRepa> hr1;
		{
			auto z = hr->size;
			HistoryRepaPtrList llh;
			auto vframe = std::make_shared<Variable>("t");
			for (size_t k = 0; k < sequence_count*sequence_interval; k += sequence_interval)
			{
				auto vk = std::make_shared<Variable>(vframe, std::make_shared<Variable>((int)k));
				SizeSizeUMap nn;
				nn.reserve(n0);
				for (std::size_t i = 0; i < n0; i++)
				{
					auto& p = llu[i];
					auto v = std::make_shared<Variable>(vk, p.first);
					llu1.push_back(VarSizePair(v, p.second));
					nn[i] = llu1.size() - 1;
				}
				SizeList ll;
				for (size_t j = (sequence_count-1)*sequence_interval-k; j < z-k; j++)
					ll.push_back(j);
				auto hr0 = hrsel(ll.size(), ll.data(), *hr);
				hr0->reframe_u(nn);
				llh.push_back(std::move(hr0));
			}
			hr1 = hrjoin(llh);
		}
		
		EVAL(hr1->dimension);
		EVAL(hr1->size);

		std::vector<std::shared_ptr<ApplicationRepa>> lld;
		{
			auto z = hr->size;
			HistoryRepaPtrList llh;
			auto vframe = std::make_shared<Variable>("t");
			for (size_t k = 0; k < sequence_count*sequence_interval; k += sequence_interval)
			{
				auto vk = std::make_shared<Variable>(vframe, std::make_shared<Variable>((int)k));
				SizeSizeUMap nn;
				nn.reserve(llu.size());
				for (std::size_t i = 0; i < n0; i++)
					nn[i] = k / sequence_interval * n0 + i;
				for (std::size_t i = n0; i < llu.size(); i++)
				{
					auto& p = llu[i];
					auto v = std::make_shared<Variable>(vk, p.first);
					llu1.push_back(VarSizePair(v, p.second));
					nn[i] = llu1.size() - 1;
				}
				auto dr0 = drcopy(*dr);
				dr0->reframe_u(nn);
				lld.push_back(std::move(dr0));
			}
		}
		
		ApplicationRepa dr1;
		{
			dr1.slices = std::make_shared<SizeTree>();
			dr1.slices->_list.reserve(dr->slices->_list.size() * sequence_count);
			dr1.fud = std::make_shared<FudRepa>();
			dr1.fud->layers.reserve(dr->fud->layers.size());
			dr1.substrate.reserve(dr->substrate.size() * sequence_count);
			for (auto& dr0 : lld)
			{
				dr1.slices->_list.insert(dr1.slices->_list.end(), dr0->slices->_list.begin(), dr0->slices->_list.end());
				dr1.fud->layers.insert(dr1.fud->layers.end(), dr0->fud->layers.begin(), dr0->fud->layers.end());
				dr1.substrate.insert(dr1.substrate.end(), dr0->substrate.begin(), dr0->substrate.end());
			}
		}
		EVAL(treesSize(*dr1.slices));
		EVAL(treesLeafElements(*dr1.slices)->size());
		EVAL(frder(*dr1.fud)->size());
		EVAL(frund(*dr1.fud)->size());
		EVAL(frvars(*dr1.fud)->size());	
		
		size_t wmax = 18;
		size_t lmax = 8;
		size_t xmax = 128;
		double znnmax = 60000.0 * 2.0 * 100.0 * 100.0 * tint;
		size_t omax = 10;
		size_t bmax = 10 * 3;
		size_t mmax = 3;
		size_t umax = 128;
		size_t pmax = 1;
		size_t fmax = 4096;
		size_t mult = 1;
		size_t seed = 5;
		auto sl = treesElements(*dr1.slices);
		auto dr2 = applicationer(wmax, lmax, xmax, znnmax, omax, bmax, mmax, umax, pmax, fmax, mult, 0, seed, tint, *sl, *dr1.fud, *hr1, 0, *ur1);
		auto dr3 = drjoin(dr1, *dr2);
		std::ofstream out(model + "_" + std::to_string(sequence_count) + "_" + std::to_string(sequence_interval) + ".dr", std::ios::binary);
		systemRepasPersistent(*ur1, out); cout << endl;
		applicationRepasPersistent(*dr3, out); cout << endl;
		out.close();
	}
	
	if (argc >= 3 && string(argv[1]) == "induce" && string(argv[2]) == "model024")
	{
		auto uvars = systemsSetVar;
		auto hrsel = [](const HistoryRepa& hr, const SizeList& ll)
		{
			return eventsHistoryRepasHistoryRepaSelection_u(ll.size(), (std::size_t*)ll.data(), hr);
		};
		auto hrjoin = vectorHistoryRepasJoin_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;
		auto applicationer = parametersSystemsFudRepasHistoryRepasApplicationerSubstrateEntropyMaxRollByMExcludedSelfHighestFmaxIORepa_p;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;
		size_t sequence_count = argc >= 5 ? atoi(argv[4]) : 1;
		size_t sequence_interval = argc >= 6 ? atoi(argv[5]) : 1;
		
		EVAL(model);
		EVAL(tint);
		EVAL(sequence_count);
		EVAL(sequence_interval);

		auto ur1 = std::make_unique<SystemRepa>();
		auto& llu1 = ur1->listVarSizePair;
		std::unique_ptr<Alignment::HistoryRepa> hr;	
		{
			std::vector<std::string> files{
				"data003.bin",
				"data004_01.bin",
				"data004_02.bin",
				"data004_03.bin",
				"data004_04.bin",
				"data004_05.bin"
			};
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				auto z = qq->size();
				HistoryRepaPtrList llh;
				VarSizeUMap urm = ur1->mapVarSize();
				for (size_t k = 0; k < sequence_count*sequence_interval; k += sequence_interval)
				{
					auto xx = recordListsHistoryRepaRegion(8, 60, 17, *qq, (sequence_count-1)*sequence_interval-k, z-k-1);
					auto uu = std::move(std::get<0>(xx));
					auto ur = std::move(std::get<1>(xx));					
					auto hr0 = std::move(std::get<2>(xx));
					auto vk = std::make_shared<Variable>(std::string("t") + std::to_string(k));
					auto& llu = ur->listVarSizePair;
					SizeSizeUMap nn;
					nn.reserve(llu.size());
					for (std::size_t i = 0; i < llu.size(); i++)
					{
						auto& p = llu[i];
						auto v = std::make_shared<Variable>(p.first, vk);
						auto it = urm.find(*v);
						if (it != urm.end())
						{
							nn[i] = it->second;
						}
						else
						{
							llu1.push_back(VarSizePair(v, p.second));
							nn[i] = llu1.size() - 1;						
						}
					}
					hr0->reframe_u(nn);
					llh.push_back(std::move(hr0));
				}
				auto hr1 = hrjoin(llh);
				ll.push_back(std::move(hr1));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);
		
		if (false)
		{
			SizeList ll;
			for (size_t i = 0; i < 60; i++)
				ll.push_back(i);
			hr = hrsel(*hr, ll);
			auto bm = historyRepasBitmapUncentred(10, 8, *hr);
			bmwrite("/tmp/a.bmp", bm);		
		}

		ApplicationRepa dr;
		{
			StrVarPtrMap m;
			std::ifstream in("model011.dr", std::ios::binary);
			auto ur = persistentsSystemRepa(in, m);
			auto dr0 = persistentsApplicationRepa(in);
			in.close();
			dr.slices = std::make_shared<SizeTree>();
			dr.slices->_list.reserve(dr0->slices->_list.size() * sequence_count);
			dr.fud = std::make_shared<FudRepa>();
			dr.fud->layers.reserve(dr0->fud->layers.size());
			dr.substrate.reserve(dr0->substrate.size() * sequence_count);
			VarSizeUMap urm = ur1->mapVarSize();
			auto& llu = ur->listVarSizePair;
			for (size_t k = 0; k < sequence_count*sequence_interval; k += sequence_interval)
			{
				auto vk = std::make_shared<Variable>(std::string("t") + std::to_string(k));
				SizeSizeUMap nn;
				nn.reserve(llu.size());
				for (std::size_t i = 0; i < llu.size(); i++)
				{
					auto& p = llu[i];
					auto v = std::make_shared<Variable>(p.first, vk);
					auto it = urm.find(*v);
					if (it != urm.end())
					{
						nn[i] = it->second;
					}
					else
					{
						llu1.push_back(VarSizePair(v, p.second));
						nn[i] = llu1.size() - 1;						
					}
				}
				auto dr2 = drcopy(*dr0);
				dr2->reframe_u(nn);
				dr.slices->_list.insert(dr.slices->_list.end(), dr2->slices->_list.begin(), dr2->slices->_list.end());
				for (std::size_t l = 0; l < dr2->fud->layers.size(); l++)
				{
					if (l < dr.fud->layers.size())
						dr.fud->layers[l].insert(dr.fud->layers[l].end(), dr2->fud->layers[l].begin(), dr2->fud->layers[l].end());
					else
						dr.fud->layers.push_back(dr2->fud->layers[l]);
				}
				dr.substrate.insert(dr.substrate.end(), dr2->substrate.begin(), dr2->substrate.end());
			}			
		}
		
		EVAL(fudRepasSize(*dr.fud));
		EVAL(frder(*dr.fud)->size());
		EVAL(frund(*dr.fud)->size());
		EVAL(treesSize(*dr.slices));
		EVAL(treesLeafElements(*dr.slices)->size());

		// EVAL(hr->mapVarInt());
		// EVAL(*frund(*dr.fud));
		
		size_t wmax = 18;
		size_t lmax = 8;
		size_t xmax = 128;
		double znnmax = 60000.0 * 2.0 * 100.0 * 100.0 * tint;
		size_t omax = 10;
		size_t bmax = 10 * 3;
		size_t mmax = 3;
		size_t umax = 128;
		size_t pmax = 1;
		size_t fmax = 384;
		size_t mult = 1;
		size_t seed = 5;
		auto sl = treesElements(*dr.slices);
		auto dr2 = applicationer(wmax, lmax, xmax, znnmax, omax, bmax, mmax, umax, pmax, fmax, mult, 0, seed, tint, *sl, *dr.fud, *hr, 0, *ur1);
		auto dr3 = drjoin(dr, *dr2);
		std::ofstream out(model + ".dr", std::ios::binary);
		systemRepasPersistent(*ur1, out); cout << endl;
		applicationRepasPersistent(*dr3, out); cout << endl;
		out.close();
	}
	
	if (argc >= 3 && string(argv[1]) == "bitmap_reqion_sequence")
	{
		auto hrsel = [](const HistoryRepa& hr, const SizeList& ll)
		{
			return eventsHistoryRepasHistoryRepaSelection_u(ll.size(), (std::size_t*)ll.data(), hr);
		};
		auto hrjoin = vectorHistoryRepasJoin_u;
		
		size_t sequence_count = argc >= 3 ? atoi(argv[2]) : 1;
		size_t sequence_interval = argc >= 4 ? atoi(argv[3]) : 1;	
		size_t scale_vertical = argc >= 5 ? atoi(argv[4]) : 1;
		size_t event_start = argc >= 6 ? atoi(argv[5]) : 0;
		size_t event_end = argc >= 7 ? atoi(argv[6]) : 0;

		auto ur1 = std::make_unique<SystemRepa>();
		auto& llu1 = ur1->listVarSizePair;
		std::unique_ptr<Alignment::HistoryRepa> hr;	
		{
			std::vector<std::string> files{
				"data003.bin",
				"data004_01.bin",
				"data004_02.bin",
				"data004_03.bin",
				"data004_04.bin",
				"data004_05.bin"
			};
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				auto z = qq->size();
				HistoryRepaPtrList llh;
				VarSizeUMap urm = ur1->mapVarSize();
				for (size_t k = 0; k < sequence_count*sequence_interval; k += sequence_interval)
				{
					auto xx = recordListsHistoryRepaRegion(8, 60, 17, *qq, (sequence_count-1)*sequence_interval-k, z-k-1);
					auto uu = std::move(std::get<0>(xx));
					auto ur = std::move(std::get<1>(xx));					
					auto hr0 = std::move(std::get<2>(xx));
					auto vk = std::make_shared<Variable>(std::string("t") + std::to_string(k));
					auto& llu = ur->listVarSizePair;
					SizeSizeUMap nn;
					nn.reserve(llu.size());
					for (std::size_t i = 0; i < llu.size(); i++)
					{
						auto& p = llu[i];
						auto v = std::make_shared<Variable>(p.first, vk);
						auto it = urm.find(*v);
						if (it != urm.end())
						{
							nn[i] = it->second;
						}
						else
						{
							llu1.push_back(VarSizePair(v, p.second));
							nn[i] = llu1.size() - 1;						
						}
					}
					hr0->reframe_u(nn);
					llh.push_back(std::move(hr0));
				}
				auto hr1 = hrjoin(llh);
				ll.push_back(std::move(hr1));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}
		
		if (event_end > 0)
		{
			SizeList ll;
			for (size_t i = event_start; i <= event_end; i++)
				ll.push_back(i);
			hr = hrsel(*hr, ll);
		}

		auto bm = historyRepasBitmapUncentred(scale_vertical, 8, *hr);
		bmwrite(std::string("data004_") + std::to_string(sequence_count) + "_" + std::to_string(sequence_interval) + ".bmp", bm);
	}
	
	if (argc >= 3 && string(argv[1]) == "entropy_region_sequence")
	{
		auto uvars = systemsSetVar;
		auto uruu = systemsRepasSystem;
		auto aall = histogramsList;
		auto add = pairHistogramsAdd_u;
		auto ent = histogramsEntropy;
		auto araa = systemsHistogramRepasHistogram_u;
		auto hrred = [](const HistoryRepa& hr, const SystemRepa& ur, const VarList& kk)
		{
			auto& vvi = ur.mapVarSize();
			std::size_t m = kk.size();
			SizeList kk1;
			for (std::size_t i = 0; i < m; i++)
				kk1.push_back(vvi[kk[i]]);
			return setVarsHistoryRepasReduce_u(1.0, m, kk1.data(), hr);
		};
		auto hrjoin = vectorHistoryRepasJoin_u;		
		auto hrconcat = vectorHistoryRepasConcat_u;
		auto hrshuffle = historyRepasShuffle_u;
		auto hrpart = systemsHistoryRepasApplicationsHistoryHistoryPartitionedRepa_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		
		string model = string(argv[2]);
		size_t mult = argc >= 4 ? atoi(argv[3]) : 1;
		size_t sequence_count = argc >= 5 ? atoi(argv[4]) : 1;
		size_t sequence_interval = argc >= 6 ? atoi(argv[5]) : 1;	
		
		EVAL(model);
		EVAL(mult);
		EVAL("data004");
		EVAL(sequence_count);
		EVAL(sequence_interval);

		std::unique_ptr<Alignment::HistoryRepa> hr;	
		{
			auto ur1 = std::make_unique<SystemRepa>();
			auto& llu1 = ur1->listVarSizePair;

			std::vector<std::string> files{
				"data003.bin",
				"data004_01.bin",
				"data004_02.bin",
				"data004_03.bin",
				"data004_04.bin",
				"data004_05.bin"
			};
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				auto z = qq->size();
				HistoryRepaPtrList llh;
				VarSizeUMap urm = ur1->mapVarSize();
				for (size_t k = 0; k < sequence_count*sequence_interval; k += sequence_interval)
				{
					auto xx = recordListsHistoryRepaRegion(8, 60, 17, *qq, (sequence_count-1)*sequence_interval-k, z-k-1);
					auto uu = std::move(std::get<0>(xx));
					auto ur = std::move(std::get<1>(xx));					
					auto hr0 = std::move(std::get<2>(xx));
					auto vk = std::make_shared<Variable>(std::string("t") + std::to_string(k));
					auto& llu = ur->listVarSizePair;
					SizeSizeUMap nn;
					nn.reserve(llu.size());
					for (std::size_t i = 0; i < llu.size(); i++)
					{
						auto& p = llu[i];
						auto v = std::make_shared<Variable>(p.first, vk);
						auto it = urm.find(*v);
						if (it != urm.end())
						{
							nn[i] = it->second;
						}
						else
						{
							llu1.push_back(VarSizePair(v, p.second));
							nn[i] = llu1.size() - 1;						
						}
					}
					hr0->reframe_u(nn);
					llh.push_back(std::move(hr0));
				}
				auto hr1 = hrjoin(llh);
				ll.push_back(std::move(hr1));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		ECHO(auto z = hr->size);
		EVAL(z);
		ECHO(auto v = z * mult);
		EVAL(v);
		
		StrVarPtrMap m;
		std::ifstream in(model + ".dr", std::ios::binary);
		auto ur = persistentsSystemRepa(in, m);
		auto dr = persistentsApplicationRepa(in);
		in.close();

		EVAL(fudRepasSize(*dr->fud));
		EVAL(frder(*dr->fud)->size());
		EVAL(frund(*dr->fud)->size());
		EVAL(treesSize(*dr->slices));
		EVAL(treesLeafElements(*dr->slices)->size());

		auto hrp = hrpart(*hr, *dr, *ur);
		auto uu = std::make_unique<System>();
		uruu(*ur, *uu);
		auto aa = araa(*uu, *ur, *hrred(*hrp, *ur, VarList{ Variable("partition0"), Variable("partition1") }));
		EVAL(ent(*aa) * z);
		
		HistoryRepaPtrList qq;
		qq.reserve(mult);
		for (std::size_t i = 1; i <= mult; i++)
			qq.push_back(hrshuffle(*hr, (unsigned int)(12345+i*z)));
		auto hrs = hrconcat(qq);
		
		auto hrsp = hrpart(*hrs, *dr, *ur);
		auto bb = araa(*uu, *ur, *hrred(*hrsp, *ur, VarList{ Variable("partition0"), Variable("partition1") }));
		EVAL(ent(*bb) * v);
		
		EVAL(ent(*add(*aa,*bb)) * (z+v));
		EVAL(ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v);
	}

	if (argc >= 3 && string(argv[1]) == "condition" && string(argv[2]) == "model025")
	{
		auto uvars = systemsSetVar;
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrjoin = vectorHistoryRepasJoin_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto frmul = historyRepasFudRepasMultiply_up;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;		
		auto applicationer = parametersSystemsHistoryRepasApplicationerCondMultinomialFmaxIORepa_up;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;
		string label = argc >= 5 ? string(argv[4]) : "motor";
		size_t sequence_count = argc >= 6 ? atoi(argv[5]) : 1;
		size_t sequence_interval = argc >= 7 ? atoi(argv[6]) : 1;
		
		EVAL(model);
		EVAL(tint);
		EVAL(label);
		EVAL(sequence_count);
		EVAL(sequence_interval);
		
		auto ur1 = std::make_unique<SystemRepa>();
		auto& llu1 = ur1->listVarSizePair;
		std::unique_ptr<Alignment::HistoryRepa> hr;	
		{
			std::vector<std::string> files{
				"data003.bin",
				"data004_01.bin",
				"data004_02.bin",
				"data004_03.bin",
				"data004_04.bin",
				"data004_05.bin"
			};
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				auto z = qq->size();
				HistoryRepaPtrList llh;
				VarSizeUMap urm = ur1->mapVarSize();
				for (size_t k = 0; k < sequence_count*sequence_interval; k += sequence_interval)
				{
					auto xx = recordListsHistoryRepa_2(8, *qq);
					auto uu = std::move(std::get<0>(xx));
					auto ur = std::move(std::get<1>(xx));					
					auto hr0 = std::move(std::get<2>(xx));
					auto vk = std::make_shared<Variable>(std::string("t") + std::to_string(k));
					auto& llu = ur->listVarSizePair;
					SizeSizeUMap nn;
					nn.reserve(llu.size());
					for (std::size_t i = 0; i < llu.size(); i++)
					{
						auto& p = llu[i];
						auto v = std::make_shared<Variable>(p.first, vk);
						auto it = urm.find(*v);
						if (it != urm.end())
						{
							nn[i] = it->second;
						}
						else
						{
							llu1.push_back(VarSizePair(v, p.second));
							nn[i] = llu1.size() - 1;						
						}
					}
					SizeList ll;
					for (size_t j = (sequence_count-1)*sequence_interval-k; j < z-k; j++)
						ll.push_back(j);
					hr0 = hrsel(ll.size(), ll.data(), *hr0);
					hr0->reframe_u(nn);
					llh.push_back(std::move(hr0));
				}
				auto hr1 = hrjoin(llh);
				ll.push_back(std::move(hr1));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);

		ApplicationRepa dr;
		{
			StrVarPtrMap m;
			std::ifstream in("model024.dr", std::ios::binary);
			auto ur = persistentsSystemRepa(in, m);
			auto dr0 = persistentsApplicationRepa(in);
			in.close();
			dr.slices = std::make_shared<SizeTree>();
			dr.slices->_list.reserve(dr0->slices->_list.size() * sequence_count);
			dr.fud = std::make_shared<FudRepa>();
			dr.fud->layers.reserve(dr0->fud->layers.size());
			dr.substrate.reserve(dr0->substrate.size() * sequence_count);
			VarSizeUMap urm1 = ur1->mapVarSize();
			auto& llu = ur->listVarSizePair;
			auto n = fudRepasSize(*dr0->fud);
			size_t a = 360;
			size_t b = 60;
			llu1.reserve(n*a/b*2 + a*sequence_count);
			auto vframe = std::make_shared<Variable>("f");
			for (int i = 0; i < a * 2 / b; i++)
			{
				auto dr2 = drcopy(*dr0);
				SizeSizeUMap nn;
				nn.reserve(n + b);
				for (auto x1 : dr0->substrate)
				{
					auto& p = llu[x1];
					auto v0 = p.first->_var0;					
					auto v1 = p.first->_var1;	
					auto v2 = v0->_var0;			
					auto v3 = std::make_shared<Variable>((int)((v0->_var1->_int + i*b / 2 - 1) % a) + 1);				
					auto v = std::make_shared<Variable>(std::make_shared<Variable>(v2,v3), v1);
					nn[x1] = urm1[*v];
				}
				auto vk = std::make_shared<Variable>(vframe, std::make_shared<Variable>((int)i + 1));
				for (auto& ll : dr0->fud->layers)
					for (auto& tr : ll)
					{
						auto x1 = tr->derived;
						auto& p = llu[x1];
						auto v = std::make_shared<Variable>(vk, p.first);
						llu1.push_back(VarSizePair(v, p.second));
						nn[x1] = llu1.size() - 1;
					}
				dr2->reframe_u(nn);
				dr.slices->_list.insert(dr.slices->_list.end(), dr2->slices->_list.begin(), dr2->slices->_list.end());
				for (std::size_t l = 0; l < dr2->fud->layers.size(); l++)
				{
					if (l < dr.fud->layers.size())
						dr.fud->layers[l].insert(dr.fud->layers[l].end(), dr2->fud->layers[l].begin(), dr2->fud->layers[l].end());
					else
						dr.fud->layers.push_back(dr2->fud->layers[l]);
				}
				dr.substrate.insert(dr.substrate.end(), dr2->substrate.begin(), dr2->substrate.end());
			}
		}

		EVAL(treesSize(*dr.slices));
		EVAL(treesLeafElements(*dr.slices)->size());
		EVAL(frder(*dr.fud)->size());
		EVAL(frund(*dr.fud)->size());
		EVAL(frvars(*dr.fud)->size());	
		
		auto hr2 = frmul(tint, *hr, *dr.fud);

		EVAL(hr2->dimension);
		EVAL(hr2->size);
		
		auto sl = treesElements(*dr.slices);
		auto vt = std::make_shared<Variable>("t0");
		auto vl = std::make_shared<Variable>(label);
		auto l = ur1->mapVarSize()[Variable(vl,vt)];
		size_t fmax = 4096;
		auto dr2 = applicationer(fmax, tint, *sl, l, *hr2, 1, *ur1);
		auto dr3 = drjoin(dr, *dr2);
		std::ofstream out(model + "_" + label + "_" + std::to_string(sequence_count) + "_" + std::to_string(sequence_interval) + ".dr", std::ios::binary);
		systemRepasPersistent(*ur1, out); cout << endl;
		applicationRepasPersistent(*dr3, out); cout << endl;
		out.close();
	}

	if (argc >= 3 && string(argv[1]) == "induce" && string(argv[2]) == "model026")
	{
		auto uvars = systemsSetVar;
		auto applicationer = parametersSystemsFudRepasHistoryRepasApplicationerSubstrateEntropyMaxRollByMExcludedSelfHighestFmaxIORepa_p;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;

		std::vector<std::string> files{
			"data009.bin"
		};
		HistoryRepaPtrList ll;
		int s = 17;
		for (auto& f : files)
		{
			std::ifstream in(f, std::ios::binary);
			auto qq = persistentsRecordList(in);
			in.close();
			for (int i = 0; i < 5; i++)
			{
				auto xx = recordListsHistoryRepaRegion(8, 60, s++, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
		}
		auto hr = vectorHistoryRepasConcat_u(ll);

		EVAL(hr->dimension);
		EVAL(hr->size);

		auto vvk = *uvars(*uu);

		auto& vvi = ur->mapVarSize();
		auto vvk0 = sorted(vvk);
		SizeList vvk1;
		for (auto& v : vvk0)
			vvk1.push_back(vvi[v]);

		size_t wmax = 9;
		size_t lmax = 8;
		size_t xmax = 128;
		double znnmax = (double)hr->size * 2.0 * 300.0 * 300.0;
		size_t omax = 10;
		size_t bmax = 10 * 3;
		size_t mmax = 3;
		size_t umax = 128;
		size_t pmax = 1;
		size_t fmax = 127;
		size_t mult = 1;
		size_t seed = 5;
		auto dr = applicationer(wmax, lmax, xmax, znnmax, omax, bmax, mmax, umax, pmax, fmax, mult, 0, seed, tint, vvk1, FudRepa(), *hr, 0, *ur);
		std::ofstream out(model+".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr, out); cout << endl;
		out.close();
	}
	
	if (argc >= 3 && string(argv[1]) == "induce" && string(argv[2]) == "model027")
	{
		auto uvars = systemsSetVar;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;
		auto applicationer = parametersSystemsFudRepasHistoryRepasApplicationerSubstrateEntropyMaxRollByMExcludedSelfHighestFmaxIORepa_p;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
		std::unique_ptr<Alignment::HistoryRepa> hr;
		
		{
			std::vector<std::string> files{
				"data009.bin"
			};
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				auto xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);

		Variable motor("motor");
		Variable location("location");
		Variable position("position");
		auto vv = *uvars(*uu);
		auto vvk = VarUSet(vv);
		vvk.erase(motor);
		vvk.erase(location);
		vvk.erase(position);

		auto& vvi = ur->mapVarSize();
		auto vvk0 = sorted(vvk);
		SizeList vvk1;
		for (auto& v : vvk0)
			vvk1.push_back(vvi[v]);

		ApplicationRepa dr;
		{
			StrVarPtrMap m;
			std::ifstream in("model026.dr", std::ios::binary);
			auto ur1 = persistentsSystemRepa(in, m);
			auto dr1 = persistentsApplicationRepa(in);
			in.close();
			auto& llu1 = ur1->listVarSizePair;
			VarSizeUMap ur0 = ur->mapVarSize();
			auto n = fudRepasSize(*dr1->fud);
			size_t a = 360;
			size_t b = 60;
			auto& llu = ur->listVarSizePair;
			llu.reserve(n*a/b + a);
			dr.slices = std::make_shared<SizeTree>();
			dr.slices->_list.reserve(dr1->slices->_list.size() * a / b);
			dr.fud = std::make_shared<FudRepa>();
			dr.fud->layers.reserve(dr1->fud->layers.size());
			dr.substrate.reserve(dr1->substrate.size() * a / b);
			for (int i = 0; i < a*2/b; i++)
			{
				auto dr2 = drcopy(*dr1);
				SizeSizeUMap nn;
				nn.reserve(n + b);
				for (auto x1 : dr1->substrate)
				{
					auto& p = llu1[x1];
					auto v1 = p.first->_var0;
					auto v2 = std::make_shared<Variable>(((int)(p.first->_var1->_int + i*b/2 - 1)) % a + 1);
					auto v = std::make_shared<Variable>(v1, v2);
					nn[x1] = ur0[*v];
				}
				auto vk = std::make_shared<Variable>((int)i + 1);
				for (auto& ll : dr1->fud->layers)
					for (auto& tr : ll)
					{
						auto x1 = tr->derived;
						auto& p = llu1[x1];
						auto v = std::make_shared<Variable>(p.first, vk);
						llu.push_back(VarSizePair(v, p.second));			
						nn[x1] = llu.size() - 1;
					}
				dr2->reframe_u(nn);
				dr.slices->_list.insert(dr.slices->_list.end(), dr2->slices->_list.begin(), dr2->slices->_list.end());
				for (std::size_t l = 0; l < dr2->fud->layers.size(); l++)
				{
					if (l < dr.fud->layers.size())
						dr.fud->layers[l].insert(dr.fud->layers[l].end(), dr2->fud->layers[l].begin(), dr2->fud->layers[l].end());
					else
						dr.fud->layers.push_back(dr2->fud->layers[l]);
				}
				dr.substrate.insert(dr.substrate.end(), dr2->substrate.begin(), dr2->substrate.end());
			}
		}

		size_t wmax = 18;
		size_t lmax = 8;
		size_t xmax = 128;
		double znnmax = 60000.0 * 2.0 * 100.0 * 100.0 * tint;
		size_t omax = 10;
		size_t bmax = 10 * 3;
		size_t mmax = 3;
		size_t umax = 128;
		size_t pmax = 1;
		size_t fmax = 4096;
		size_t mult = 1;
		size_t seed = 5;
		auto sl = treesElements(*dr.slices);
		auto dr2 = applicationer(wmax, lmax, xmax, znnmax, omax, bmax, mmax, umax, pmax, fmax, mult, 0, seed, tint, *sl, *dr.fud, *hr, 0, *ur);
		auto dr3 = drjoin(dr, *dr2);
		std::ofstream out(model + ".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr3, out); cout << endl;
		out.close();
	}
	
	if (argc >= 3 && string(argv[1]) == "condition" && string(argv[2]) == "model028")
	{
		auto uvars = systemsSetVar;
		auto frmul = historyRepasFudRepasMultiply_up;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;
		auto applicationer = parametersSystemsHistoryRepasApplicationerCondMultinomialFmaxIORepa_up;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;
		string label = argc >= 5 ? string(argv[4]) : "motor";

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
		std::unique_ptr<Alignment::HistoryRepa> hr;

		{
			std::vector<std::string> files{
				"data009.bin"
			};
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				auto xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);

		Variable motor("motor");
		Variable location("location");
		Variable position("position");
		auto vv = *uvars(*uu);
		auto vvk = VarUSet(vv);
		vvk.erase(motor);
		vvk.erase(location);
		vvk.erase(position);

		auto& vvi = ur->mapVarSize();
		SizeList vvk1;
		for (auto& v : sorted(vvk))
			vvk1.push_back(vvi[v]);

		ApplicationRepa dr;
		{
			StrVarPtrMap m;
			std::ifstream in("model026.dr", std::ios::binary);
			auto ur1 = persistentsSystemRepa(in, m);
			auto dr1 = persistentsApplicationRepa(in);
			in.close();
			auto& llu1 = ur1->listVarSizePair;
			VarSizeUMap ur0 = ur->mapVarSize();
			auto n = fudRepasSize(*dr1->fud);
			size_t a = 360;
			size_t b = 60;
			auto& llu = ur->listVarSizePair;
			llu.reserve(n*a / b + a);
			dr.slices = std::make_shared<SizeTree>();
			dr.slices->_list.reserve(dr1->slices->_list.size() * a / b);
			dr.fud = std::make_shared<FudRepa>();
			dr.fud->layers.reserve(dr1->fud->layers.size());
			dr.substrate.reserve(dr1->substrate.size() * a / b);
			for (int i = 0; i < a * 2 / b; i++)
			{
				auto dr2 = drcopy(*dr1);
				SizeSizeUMap nn;
				nn.reserve(n + b);
				for (auto x1 : dr1->substrate)
				{
					auto& p = llu1[x1];
					auto v1 = p.first->_var0;
					auto v2 = std::make_shared<Variable>(((int)(p.first->_var1->_int + i*b/2 - 1)) % a + 1);
					auto v = std::make_shared<Variable>(v1, v2);
					nn[x1] = ur0[*v];
				}
				auto vk = std::make_shared<Variable>((int)i + 1);
				for (auto& ll : dr1->fud->layers)
					for (auto& tr : ll)
					{
						auto x1 = tr->derived;
						auto& p = llu1[x1];
						auto v = std::make_shared<Variable>(p.first, vk);
						llu.push_back(VarSizePair(v, p.second));
						nn[x1] = llu.size() - 1;
					}
				dr2->reframe_u(nn);
				dr.slices->_list.insert(dr.slices->_list.end(), dr2->slices->_list.begin(), dr2->slices->_list.end());
				for (std::size_t l = 0; l < dr2->fud->layers.size(); l++)
				{
					if (l < dr.fud->layers.size())
						dr.fud->layers[l].insert(dr.fud->layers[l].end(), dr2->fud->layers[l].begin(), dr2->fud->layers[l].end());
					else
						dr.fud->layers.push_back(dr2->fud->layers[l]);
				}
				dr.substrate.insert(dr.substrate.end(), dr2->substrate.begin(), dr2->substrate.end());
			}
		}

		auto hr1 = frmul(tint, *hr, *dr.fud);
		auto sl = treesElements(*dr.slices);
		size_t fmax = 4096;
		auto dr2 = applicationer(fmax, tint, *sl, vvi[Variable(label)], *hr1, 1, *ur);
		auto dr3 = drjoin(dr, *dr2);
		std::ofstream out(model + "_" + label + ".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr3, out); cout << endl;
		out.close();
	}
	
	/* 	if (argc >= 3 && string(argv[1]) == "label_event_count")
	{
		auto uvars = systemsSetVar;
		auto single = histogramSingleton_u;
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrhrred = setVarsHistoryRepasHistoryRepaReduced_u;
		auto hrred = setVarsHistoryRepasReduce_u;
		auto frmul = historyRepasFudRepasMultiply_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto frdep = fudRepasSetVarsDepends;
		auto hrbm = historyRepasBitmapAverage;

		string dataset = string(argc >= 3 ? argv[2] : "data002");
		string label = string(argc >= 4 ? argv[3] : "location");
		string value = string(argc >= 5 ? argv[4] : "room1");
		size_t event_start = argc >= 6 ? atoi(argv[5]) : 0;
		size_t event_end = argc >= 7 ? atoi(argv[6]) : 0;
		
		EVAL(dataset);
		EVAL(label);
		EVAL(value);
		EVAL(sequence_count);
		EVAL(sequence_interval);
			
		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
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
				auto xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		if (event_end > 0)
		{
			SizeList ev;
			for (size_t i = event_start; i <= event_end; i++)
				ev.push_back(i);
			hr = hrsel(ev.size(), ev.data(), *hr);
		}
		
		std::vector<std::string> locations{ 
			"door12", "door13", "door14", "door45", "door56", 
			"room1", "room2", "room3", "room4", "room5", "room6", "unknown" };
		std::vector<std::string> positions{ "centre", "corner", "side", "unknown" };
		SizeList ww{ pl };
		auto nn = treesLeafNodes(*dr->slices);
		for (auto& s : *nn)
			ww.push_back(s.first);
		auto hr1 = hrhrred(ww.size(), ww.data(), *frmul(*hr, *dr->fud));
		if (!hr1->evient)
			hr1->transpose();
		auto z = hr1->size;
		auto n = hr1->dimension;
		auto vv = hr1->vectorVar;
		auto sh = hr1->shape;
		auto rr = hr1->arr;
		auto sl = sh[0];
		SizeSet slice_unique;
		std::size_t consecutive_unique_count = 0;
		std::size_t slice_last = 0;
		std::size_t match_count = 0;
		cout << "event|slice|location|guess|match?" << endl;
		cout << "---|---|---|---|---" << endl;
		for (std::size_t j = 0; j < z; j++)
		{
			std::size_t cl = rr[j*n];
			std::size_t l = sl;
			std::size_t i = 1;
			for (; i < n; i++)
			{
				std::size_t u = rr[j*n + i];
				if (u)
				{
					auto s = vv[i];
					l = su[s];
					if (slice_last != s)
					{
						consecutive_unique_count++;
						slice_last = s;
						slice_unique.insert(s);
					}
					break;
				}
			}
			if (l == cl)
				match_count++;
			cout << j;
			if (i<n)
				cout << "|" << *ur1->listVarSizePair[vv[i]].first;
			else
				cout << ",no var";
			cout << "|" << (label == "location" ? locations[cl] : positions[cl]);
			cout << "|" << (label == "location" ? locations[l] : positions[l]);
			cout << "|" << (l == cl ? "match" : "fail") << endl;
		}
		EVAL(z);
		EVAL(slice_unique.size());
		EVAL(consecutive_unique_count);
		EVAL(match_count);
		EVAL(100.0*match_count/z);
	} */
	
	if (argc >= 3 && string(argv[1]) == "condition" && (string(argv[2]) == "model029" || string(argv[2]) == "model030" || string(argv[2]) == "model031"))
	{
		auto uvars = systemsSetVar;
		auto frmul = historyRepasFudRepasMultiply_up;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;
		auto applicationer = parametersSystemsHistoryRepasApplicationerCondMultinomialFmaxIORepa_up;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;
		string label = argc >= 5 ? string(argv[4]) : "location_next";

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
		std::unique_ptr<Alignment::HistoryRepa> hr;

		{
			std::vector<std::string> files{
				"data009.bin"
			};
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				auto xx = recordListsHistoryRepa_3(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);

		ApplicationRepa dr;
		{
			StrVarPtrMap m;
			std::ifstream in("model026.dr", std::ios::binary);
			auto ur1 = persistentsSystemRepa(in, m);
			auto dr1 = persistentsApplicationRepa(in);
			in.close();
			auto& llu1 = ur1->listVarSizePair;
			VarSizeUMap ur0 = ur->mapVarSize();
			auto n = fudRepasSize(*dr1->fud);
			size_t a = 360;
			size_t b = 60;
			auto& llu = ur->listVarSizePair;
			llu.reserve(n*a / b + a);
			dr.slices = std::make_shared<SizeTree>();
			dr.slices->_list.reserve(dr1->slices->_list.size() * a / b);
			dr.fud = std::make_shared<FudRepa>();
			dr.fud->layers.reserve(dr1->fud->layers.size());
			dr.substrate.reserve(dr1->substrate.size() * a / b);
			for (int i = 0; i < a * 2 / b; i++)
			{
				auto dr2 = drcopy(*dr1);
				SizeSizeUMap nn;
				nn.reserve(n + b);
				for (auto x1 : dr1->substrate)
				{
					auto& p = llu1[x1];
					auto v1 = p.first->_var0;
					auto v2 = std::make_shared<Variable>(((int)(p.first->_var1->_int + i*b/2 - 1)) % a + 1);
					auto v = std::make_shared<Variable>(v1, v2);
					nn[x1] = ur0[*v];
				}
				auto vk = std::make_shared<Variable>((int)i + 1);
				for (auto& ll : dr1->fud->layers)
					for (auto& tr : ll)
					{
						auto x1 = tr->derived;
						auto& p = llu1[x1];
						auto v = std::make_shared<Variable>(p.first, vk);
						llu.push_back(VarSizePair(v, p.second));
						nn[x1] = llu.size() - 1;
					}
				dr2->reframe_u(nn);
				dr.slices->_list.insert(dr.slices->_list.end(), dr2->slices->_list.begin(), dr2->slices->_list.end());
				for (std::size_t l = 0; l < dr2->fud->layers.size(); l++)
				{
					if (l < dr.fud->layers.size())
						dr.fud->layers[l].insert(dr.fud->layers[l].end(), dr2->fud->layers[l].begin(), dr2->fud->layers[l].end());
					else
						dr.fud->layers.push_back(dr2->fud->layers[l]);
				}
				dr.substrate.insert(dr.substrate.end(), dr2->substrate.begin(), dr2->substrate.end());
			}
		}

		auto& vvi = ur->mapVarSize();
		auto hr1 = frmul(tint, *hr, *dr.fud);
		auto sl = treesElements(*dr.slices);
		if (model == "model030" || model == "model031")
			sl->push_back(vvi[Variable("motor")]);		
		size_t fmax = model == "model031" ? 4096*4 : 4096;
		auto dr2 = applicationer(fmax, tint, *sl, vvi[Variable(label)], *hr1, 1, *ur);
		auto dr3 = drjoin(dr, *dr2);
		std::ofstream out(model + "_" + label + ".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr3, out); cout << endl;
		out.close();
	}
	
		if (argc >= 3 && string(argv[1]) == "condition" && (string(argv[2]) == "model032" || string(argv[2]) == "model033" || string(argv[2]) == "model034"))
	{
		auto uvars = systemsSetVar;
		auto frmul = historyRepasFudRepasMultiply_up;
		auto drcopy = applicationRepasApplicationRepa_u;
		auto drjoin = applicationRepaPairsJoin_u;
		auto applicationer = parametersSystemsHistoryRepasApplicationerCondMultinomialFmaxIORepa_up;

		string model = string(argv[2]);
		size_t tint = argc >= 4 ? atoi(argv[3]) : 1;
		string label = argc >= 5 ? string(argv[4]) : "room_next";

		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
		std::unique_ptr<Alignment::HistoryRepa> hr;

		{
			std::vector<std::string> files{
				"data009.bin"
			};
			HistoryRepaPtrList ll;
			for (auto& f : files)
			{
				std::ifstream in(f, std::ios::binary);
				auto qq = persistentsRecordList(in);
				in.close();
				auto xx = recordListsHistoryRepa_4(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}

		EVAL(hr->dimension);
		EVAL(hr->size);

		ApplicationRepa dr;
		{
			StrVarPtrMap m;
			std::ifstream in("model026.dr", std::ios::binary);
			auto ur1 = persistentsSystemRepa(in, m);
			auto dr1 = persistentsApplicationRepa(in);
			in.close();
			auto& llu1 = ur1->listVarSizePair;
			VarSizeUMap ur0 = ur->mapVarSize();
			auto n = fudRepasSize(*dr1->fud);
			size_t a = 360;
			size_t b = 60;
			auto& llu = ur->listVarSizePair;
			llu.reserve(n*a / b + a);
			dr.slices = std::make_shared<SizeTree>();
			dr.slices->_list.reserve(dr1->slices->_list.size() * a / b);
			dr.fud = std::make_shared<FudRepa>();
			dr.fud->layers.reserve(dr1->fud->layers.size());
			dr.substrate.reserve(dr1->substrate.size() * a / b);
			for (int i = 0; i < a * 2 / b; i++)
			{
				auto dr2 = drcopy(*dr1);
				SizeSizeUMap nn;
				nn.reserve(n + b);
				for (auto x1 : dr1->substrate)
				{
					auto& p = llu1[x1];
					auto v1 = p.first->_var0;
					auto v2 = std::make_shared<Variable>(((int)(p.first->_var1->_int + i*b/2 - 1)) % a + 1);
					auto v = std::make_shared<Variable>(v1, v2);
					nn[x1] = ur0[*v];
				}
				auto vk = std::make_shared<Variable>((int)i + 1);
				for (auto& ll : dr1->fud->layers)
					for (auto& tr : ll)
					{
						auto x1 = tr->derived;
						auto& p = llu1[x1];
						auto v = std::make_shared<Variable>(p.first, vk);
						llu.push_back(VarSizePair(v, p.second));
						nn[x1] = llu.size() - 1;
					}
				dr2->reframe_u(nn);
				dr.slices->_list.insert(dr.slices->_list.end(), dr2->slices->_list.begin(), dr2->slices->_list.end());
				for (std::size_t l = 0; l < dr2->fud->layers.size(); l++)
				{
					if (l < dr.fud->layers.size())
						dr.fud->layers[l].insert(dr.fud->layers[l].end(), dr2->fud->layers[l].begin(), dr2->fud->layers[l].end());
					else
						dr.fud->layers.push_back(dr2->fud->layers[l]);
				}
				dr.substrate.insert(dr.substrate.end(), dr2->substrate.begin(), dr2->substrate.end());
			}
		}

		auto& vvi = ur->mapVarSize();
		auto hr1 = frmul(tint, *hr, *dr.fud);
		auto sl = treesElements(*dr.slices);
		if (model == "model033" || model == "model034")
			sl->push_back(vvi[Variable("motor")]);		
		size_t fmax = model == "model034" ? 4096*4 : 4096;
		auto dr2 = applicationer(fmax, tint, *sl, vvi[Variable(label)], *hr1, 1, *ur);
		auto dr3 = drjoin(dr, *dr2);
		std::ofstream out(model + "_" + label + ".dr", std::ios::binary);
		systemRepasPersistent(*ur, out); cout << endl;
		applicationRepasPersistent(*dr3, out); cout << endl;
		out.close();
	}
	
	if (argc >= 3 && string(argv[1]) == "slice_labels")
	{
		auto uvars = systemsSetVar;
		auto single = histogramSingleton_u;
		auto trim = histogramsTrim;
		auto aall = histogramsList;
		auto araa = systemsHistogramRepasHistogram_u;
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrhrred = setVarsHistoryRepasHistoryRepaReduced_u;
		auto hrred = setVarsHistoryRepasReduce_u;
		auto frmul = historyRepasFudRepasMultiply_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto frdep = fudRepasSetVarsDepends;

		string model = string(argv[2]);
		string dataset = string(argc >= 4 ? argv[3] : "data002");
		string substrate = string(argc >= 5 ? argv[4] : "substrate002");		
		EVAL(model);
		EVAL(dataset);
		EVAL(substrate);
		
		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
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
				if (substrate == "substrate004")
					xx = recordListsHistoryRepa_4(8, *qq);
				else if (substrate == "substrate003")
					xx = recordListsHistoryRepa_3(8, *qq);
				else
					xx = recordListsHistoryRepa_2(8, *qq);
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}
			
		auto& llu = ur->listVarSizePair;
		std::unique_ptr<Alignment::ApplicationRepa> dr;	
		{
			std::unique_ptr<Alignment::SystemRepa> ur1;
			StrVarPtrMap m;
			std::ifstream in(model + ".dr", std::ios::binary);
			ur1 = persistentsSystemRepa(in, m);
			dr = persistentsApplicationRepa(in);
			in.close();
			auto& llu1 = ur1->listVarSizePair;			
			SizeSizeUMap nn;
			for (auto& ll : dr->fud->layers)
				for (auto& tr : ll)
				{
					auto x = tr->derived;
					auto& p = llu1[x];
					llu.push_back(VarSizePair(p.first, p.second));
					nn[x] = llu.size() - 1;
				}
			dr->reframe_u(nn);
		}	

		VarSet vvl;
		vvl.insert(Variable("motor"));
		vvl.insert(Variable("location"));
		vvl.insert(Variable("position"));
		if (substrate == "substrate003" || substrate == "substrate004")
		{
			vvl.insert(Variable("location_next"));
			vvl.insert(Variable("position_next"));			
		}
		if (substrate == "substrate004")
		{
			vvl.insert(Variable("room_next"));
		}

		auto& vvi = ur->mapVarSize();
		SizeList vvl1;
		for (auto& v : vvl)
			vvl1.push_back(vvi[v]);
		
		std::map<std::size_t, std::shared_ptr<Alignment::HistogramRepa>> sar;
		{
			auto hr1 = frmul(*hr, *dr->fud);
			auto hr2 = hrhrred(vvl1.size(), vvl1.data(), *hr);
			if (hr1->evient)
				hr1->transpose();
			auto z = hr1->size;
			auto& mvv = hr1->mapVarInt();
			auto sh = hr1->shape;
			auto rr = hr1->arr;
			auto nn = treesLeafNodes(*dr->slices);
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
				sar[s.first] = std::move(hrred(1.0, vvl1.size(), vvl1.data(), *hrsel(ev.size(), ev.data(), *hr2)));
			}
		}
		for (auto& p : sar)
		{
			cout << endl << *llu[p.first].first << endl;
			rpln(cout, sorted(*aall(*trim(*araa(*uu, *ur, *p.second)))));
		}
	}
	
	if (argc >= 3 && string(argv[1]) == "entropy_room_next")
	{
		typedef std::vector<State> SList;	
		
		auto uvars = systemsSetVar;
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
		auto ent = histogramsEntropy;
		auto araa = systemsHistogramRepasHistogram_u;
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrhrred = setVarsHistoryRepasHistoryRepaReduced_u;
		auto hrred = setVarsHistoryRepasReduce_u;
		auto frmul = historyRepasFudRepasMultiply_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto frdep = fudRepasSetVarsDepends;

		string model = string(argv[2]);
		string dataset = string(argc >= 4 ? argv[3] : "data002");
		string substrate = string(argc >= 5 ? argv[4] : "substrate004");	
		
		EVAL(model);
		EVAL(dataset);
		EVAL(substrate);
		
		std::unique_ptr<Alignment::System> uu;
		std::unique_ptr<Alignment::SystemRepa> ur;
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
				uu = std::move(std::get<0>(xx));
				ur = std::move(std::get<1>(xx));
				ll.push_back(std::move(std::get<2>(xx)));
			}
			hr = vectorHistoryRepasConcat_u(ll);
		}
//		EVAL(hr->size);
		
		auto& llu = ur->listVarSizePair;
		std::unique_ptr<Alignment::ApplicationRepa> dr;	
		{
			std::unique_ptr<Alignment::SystemRepa> ur1;
			StrVarPtrMap m;
			std::ifstream in(model + ".dr", std::ios::binary);
			ur1 = persistentsSystemRepa(in, m);
			dr = persistentsApplicationRepa(in);
			in.close();
			auto& llu1 = ur1->listVarSizePair;			
			SizeSizeUMap nn;
			for (auto& ll : dr->fud->layers)
				for (auto& tr : ll)
				{
					auto x = tr->derived;
					auto& p = llu1[x];
					llu.push_back(VarSizePair(p.first, p.second));
					nn[x] = llu.size() - 1;
				}
			dr->reframe_u(nn);
		}	

		Variable motor("motor");
		Variable location("location");
		Variable room_next("room_next");
			
		std::map<State, SList> mss;
		mss[state(location, Value("door12"))] = SList{
			state(room_next, Value("room1")),
			state(room_next, Value("room2"))};
		mss[state(location, Value("door13"))] = SList{
			state(room_next, Value("room1")),
			state(room_next, Value("room3"))};
		mss[state(location, Value("door14"))] = SList{
			state(room_next, Value("room1")),
			state(room_next, Value("room4"))};
		mss[state(location, Value("door45"))] = SList{
			state(room_next, Value("room4")),
			state(room_next, Value("room5"))};
		mss[state(location, Value("door56"))] = SList{
			state(room_next, Value("room5")),
			state(room_next, Value("room6"))};	
		mss[state(location, Value("room1"))] = SList{
			state(room_next, Value("room2")),
			state(room_next, Value("room3")),
			state(room_next, Value("room4"))};	
		mss[state(location, Value("room2"))] = SList{
			state(room_next, Value("room1"))};			
		mss[state(location, Value("room3"))] = SList{
			state(room_next, Value("room1"))};			
		mss[state(location, Value("room4"))] = SList{
			state(room_next, Value("room1")),
			state(room_next, Value("room5"))};	
		mss[state(location, Value("room5"))] = SList{
			state(room_next, Value("room4")),
			state(room_next, Value("room6"))};	
		mss[state(location, Value("room6"))] = SList{
			state(room_next, Value("room5"))};	
			
		VarSet vvl;
		vvl.insert(motor);
		vvl.insert(location);
		vvl.insert(room_next);

		auto& vvi = ur->mapVarSize();
		SizeList vvl1;
		for (auto& v : vvl)
			vvl1.push_back(vvi[v]);
		
		double entropy_total;
		{
			auto hr1 = frmul(*hr, *dr->fud);
//			EVAL(hr1->size);			
			auto hr2 = hrhrred(vvl1.size(), vvl1.data(), *hr);
//			EVAL(hr1->size);
			if (hr1->evient)
				hr1->transpose();
			auto z = hr1->size;
			auto& mvv = hr1->mapVarInt();
			auto sh = hr1->shape;
			auto rr = hr1->arr;
			auto nn = treesLeafNodes(*dr->slices);
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
				{
//					EVAL(*llu[s.first].first);
					auto aa = *trim(*araa(*uu, *ur, *hrred(1.0, vvl1.size(), vvl1.data(), *hrsel(ev.size(), ev.data(), *hr2))));
//					EVAL(size(aa));
//					EVAL(aa);
					auto ss = smax(*ared(aa,VarUSet{location}));			
//					EVAL(ss);
					auto aa1 = *ared(*mul(aa,*single(ss,1)),VarUSet{motor,room_next});							
//					EVAL(aa1);	
					Histogram xx;
					double e = 0.0;
					for (auto s : mss[ss])
					{
						auto yy = *mul(aa1,*single(s,1));
//						EVAL(yy);
//						EVAL(size(yy) * ent(yy));
						e += - size(yy) * ent(yy);
						xx = *add(xx,yy);
					}
//					EVAL(xx);
//					EVAL(size(xx) * ent(xx));					
					e += size(xx) * ent(xx);
//					EVAL(e);
					entropy_total += e;
				}	
			}
		}
		EVAL(entropy_total);
	}
	
	if (argc >= 3 && string(argv[1]) == "room_expected")
	{
		typedef std::vector<State> SList;	
		
		auto uvars = systemsSetVar;
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
		auto ent = histogramsEntropy;
		auto araa = systemsHistogramRepasHistogram_u;
		auto hrsel = eventsHistoryRepasHistoryRepaSelection_u;
		auto hrhrred = setVarsHistoryRepasHistoryRepaReduced_u;
		auto hrred = setVarsHistoryRepasReduce_u;
		auto frmul = historyRepasFudRepasMultiply_u;
		auto frvars = fudRepasSetVar;
		auto frder = fudRepasDerived;
		auto frund = fudRepasUnderlying;
		auto frdep = fudRepasSetVarsDepends;

		string dataset = string(argc >= 3 ? argv[2] : "data002");
		string room_initial = string(argc >= 4 ? argv[3] : "room1");	
		size_t room_seed = argc >= 5 ? atoi(argv[4]) : 17;
		string dataset2 = string(argc >= 6 ? argv[5] : "");
		
		EVAL(dataset);
		EVAL(room_initial);
		EVAL(room_seed);
		EVAL(dataset2);
		
		double mean1 = 0.0;
		double mean2 = 0.0;
		double dev1 = 0.0;
		double dev2 = 0.0;
		double err1 = 0.0;
		double err2 = 0.0;
		{
			std::unique_ptr<Alignment::System> uu;
			std::unique_ptr<Alignment::SystemRepa> ur;
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
					uu = std::move(std::get<0>(xx));
					ur = std::move(std::get<1>(xx));
					ll.push_back(std::move(std::get<2>(xx)));
				}
				hr = vectorHistoryRepasConcat_u(ll);
			}
	//		EVAL(hr->size);

			vector<string> locations{ "door12", "door13", "door14", "door45", "door56", "room1", "room2", "room3", "room4", "room5", "room6", "unknown" };
			size_t ug = 0;
			while (ug < locations.size()-1)
				if (locations[ug] == room_initial)
					break;
				else
					ug++;
				
			auto& vvi = ur->mapVarSize();
			auto z = hr->size;
			auto rr = hr->arr;
			auto& mvv = hr->mapVarInt();
			auto p = mvv[vvi[Variable("location")]];		
			srand(room_seed);
			vector<size_t> counts {0};
			for (size_t j = 0; j < z; j++)
			{
				auto u = rr[p*z + j];
				if ((size_t)u == ug)
				{
					while ((size_t)u == ug)
						ug = (rand() % 6) + 5; 
					// EVAL(j);
					// EVAL(locations[(size_t)u]);					
					// EVAL(locations[ug]);
					counts.push_back(0);
				}
				counts.back()++;
			}
			counts.pop_back();
			EVAL(dataset);
			EVAL(z);
			auto n = counts.size();
			EVAL(n);
			EVAL(counts);
			double mean = 0.0;
			for (auto a : counts)
				mean += a;
			mean /= n;
			EVAL(mean);
			double variance = 0.0;
			for (auto a : counts)
				variance += ((double)a - mean)*((double)a - mean);
			variance /= n;
			EVAL(sqrt(variance));
			EVAL(sqrt(variance/n));
			mean1 = mean;
			dev1 = sqrt(variance);
			err1 = sqrt(variance/n);
		}

		if (dataset2 != "")
		{
			std::unique_ptr<Alignment::System> uu;
			std::unique_ptr<Alignment::SystemRepa> ur;
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
				if (dataset2 == "data003")
				{
					files.clear();
					files.push_back("data003.bin");
				}
				else if (dataset2 == "data004")
				{
					files.clear();
					files.push_back("data003.bin");
					files.push_back("data004_01.bin");
					files.push_back("data004_02.bin");
					files.push_back("data004_03.bin");
					files.push_back("data004_04.bin");
					files.push_back("data004_05.bin");
				}
				else if (dataset2 != "data002")
				{
					files.clear();
					files.push_back(dataset2+".bin");
				}			
				HistoryRepaPtrList ll;
				for (auto& f : files)
				{
					std::ifstream in(f, std::ios::binary);
					auto qq = persistentsRecordList(in);
					in.close();
					SystemHistoryRepaTuple xx;
					xx = recordListsHistoryRepa_4(8, *qq);
					uu = std::move(std::get<0>(xx));
					ur = std::move(std::get<1>(xx));
					ll.push_back(std::move(std::get<2>(xx)));
				}
				hr = vectorHistoryRepasConcat_u(ll);
			}
	//		EVAL(hr->size);

			vector<string> locations{ "door12", "door13", "door14", "door45", "door56", "room1", "room2", "room3", "room4", "room5", "room6", "unknown" };
			size_t ug = 0;
			while (ug < locations.size()-1)
				if (locations[ug] == room_initial)
					break;
				else
					ug++;
				
			auto& vvi = ur->mapVarSize();
			auto z = hr->size;
			auto rr = hr->arr;
			auto& mvv = hr->mapVarInt();
			auto p = mvv[vvi[Variable("location")]];		
			srand(room_seed);
			vector<size_t> counts {0};
			for (size_t j = 0; j < z; j++)
			{
				auto u = rr[p*z + j];
				if ((size_t)u == ug)
				{
					while ((size_t)u == ug)
						ug = (rand() % 6) + 5; 
					// EVAL(j);
					// EVAL(locations[(size_t)u]);					
					// EVAL(locations[ug]);
					counts.push_back(0);
				}
				counts.back()++;
			}
			counts.pop_back();
			EVAL(dataset2);
			EVAL(z);
			auto n = counts.size();
			EVAL(n);
			EVAL(counts);
			double mean = 0.0;
			for (auto a : counts)
				mean += a;
			mean /= n;
			EVAL(mean);
			double variance = 0.0;
			for (auto a : counts)
				variance += ((double)a - mean)*((double)a - mean);
			variance /= n;
			EVAL(sqrt(variance));
			EVAL(sqrt(variance/n));
			mean2 = mean;
			dev2 = sqrt(variance);
			err2 = sqrt(variance/n);
			
			EVAL(sqrt(err1*err1 + err2*err2));
			EVAL((mean2-mean1)/sqrt(err1*err1 + err2*err2));
		}
	}
	
	return 0;
}
