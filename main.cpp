#include "dev.h"

using namespace Alignment;
using namespace TBOT01;
using namespace std;

typedef std::chrono::duration<double> sec; 
typedef std::chrono::high_resolution_clock clk;

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

    if (true)
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


    return 0;
}
