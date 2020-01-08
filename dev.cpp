#include "dev.h"

#include <stdlib.h>
#include <random>

using namespace Alignment;
using namespace TBOT01;
using namespace std;


void TBOT01::recordListsPersistent(RecordList& rr, std::ostream& out)
{
    for (auto& r : rr)
    {
	out.write(reinterpret_cast<char*>(&r.id), sizeof(std::size_t));
	out.write(reinterpret_cast<char*>(&r.ts), sizeof(double));
	for (std::size_t i = 0; i < 7; i++)
	    out.write(reinterpret_cast<char*>(&r.sensor_pose[i]), sizeof(double));
	for (std::size_t i = 0; i < 360; i++)
	    out.write(reinterpret_cast<char*>(&r.sensor_scan[i]), sizeof(double));
	out.write(reinterpret_cast<char*>(&r.action_linear), sizeof(double));
	out.write(reinterpret_cast<char*>(&r.action_angular), sizeof(double));
    }
}

std::unique_ptr<RecordList> TBOT01::persistentsRecordList(std::istream& in)
{
    auto rr = std::make_unique<RecordList>();
    while (true)
    {
	Record r;
	in.read(reinterpret_cast<char*>(&r.id), sizeof(std::size_t));
	if (in.eof())
	    break;
	in.read(reinterpret_cast<char*>(&r.ts), sizeof(double));
	for (std::size_t i = 0; i < 7; i++)
	    in.read(reinterpret_cast<char*>(&r.sensor_pose[i]), sizeof(double));
	for (std::size_t i = 0; i < 360; i++)
	    in.read(reinterpret_cast<char*>(&r.sensor_scan[i]), sizeof(double));
	in.read(reinterpret_cast<char*>(&r.action_linear), sizeof(double));
	in.read(reinterpret_cast<char*>(&r.action_angular), sizeof(double));
	rr->push_back(r);
    }
    return rr;
}

std::ostream& operator<<(std::ostream& out, const Record& r)
{
    out << "(" << r.id << "," << r.ts << ",(";
    for (std::size_t i = 0; i < 7; i++)
	out << (i ? "," : "") << r.sensor_pose[i];
    out << "),(";
    for (std::size_t i = 0; i < 360; i++)
	out << (i ? "," : "") << r.sensor_scan[i];
    out << ")," << r.action_linear << "," << r.action_angular << ")";
    return out;
}

std::ostream& operator<<(std::ostream& out, const RecordList& rr)
{
    for (auto& r : rr)
	out << r << std::endl;
    return out;
}
