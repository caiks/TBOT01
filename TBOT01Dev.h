#ifndef TBOT01DEV_H
#define TBOT01DEV_H

#include "AlignmentUtil.h"
#include "Alignment.h"
#include "AlignmentApprox.h"
#include "AlignmentAeson.h"
#include "AlignmentRepa.h"
#include "AlignmentAesonRepa.h"
#include "AlignmentRandomRepa.h"
#include "AlignmentPracticableRepa.h"
#include "AlignmentPracticableIORepa.h"

#include <iomanip>
#include <set>
#include <unordered_set>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdio>
#include <thread>
#include <chrono>
#include <ctime>
#include <string>


namespace TBOT01
{
    struct Bitmap
    {
	Bitmap(int h = 1, int w = 1, unsigned char x = 0) {
	    height = h;
	    width = w;
	    image.resize(h*w*3,x);
	}
	int height;
	int width;
	std::vector<unsigned char> image;
    };

    Bitmap bminsert(const Bitmap&, int, int, const Bitmap&);
    Bitmap bmborder(int, const Bitmap&);
    Bitmap bmhstack(const std::vector<Bitmap>&);
    Bitmap bmvstack(const std::vector<Bitmap>&);

    void bmwrite(std::string, const Bitmap&);

    Bitmap hrbm(int,int,int,const Alignment::HistoryRepa&);


    typedef std::tuple<std::unique_ptr<Alignment::System>, std::unique_ptr<Alignment::SystemRepa>, std::unique_ptr<Alignment::HistoryRepa>> SystemHistoryRepaTuple;

    // trainBucketedIO :: Int -> IO (System, HistoryRepa)
    SystemHistoryRepaTuple trainBucketedIO(int);

    // trainBucketedAffineIO :: Int -> Int -> Double -> Int -> IO (System, HistoryRepa)
    SystemHistoryRepaTuple trainBucketedAffineIO(int, int, double, int);

    // trainBucketedIO :: Int -> IO (System, HistoryRepa)
    SystemHistoryRepaTuple trainBucketedRegionRandomIO(int,int,int);

    // testBucketedIO :: Int -> IO (System, HistoryRepa)
    SystemHistoryRepaTuple testBucketedIO(int);

}


#endif