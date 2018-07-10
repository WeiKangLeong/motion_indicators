/*
 * SvgPath.hpp
 *
 *  Created on: 11 Jul, 2014
 *      Author: liuwlz
 */

#ifndef SVGPATH_HPP_
#define SVGPATH_HPP_

#include <iostream>
#include <stdio.h>
#include <sstream>
#include <stdexcept>

#include <math.h>
#include <float.h>

#include <wordexp.h>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <nav_msgs/Path.h>

#define DEBUG 1

using namespace std;
using namespace boost;

namespace MissionPlan{

	struct PathPoint{
		PathPoint(){};
		PathPoint(double x, double y):x_(x),y_(y){};
		double x_, y_, yaw_;
	};

	class StationPath{
	public:
		StationPath();
		~StationPath(){point_set_.clear();};

		StationPath& operator=(const StationPath &stationPathIn);
		StationPath& operator+(const StationPath &stationPathIn);

		bool exist_;
		vector<PathPoint> point_set_;
		double distance_;
		double speed_limit_;
		double congestion_;
	};

	typedef pair<string, PathPoint> StationPair;

	class SvgPath{
	public:
		SvgPath();
		SvgPath(const char* fileNameIn,double resolution);
		virtual ~SvgPath();

		virtual void loadFile(const char* fileNameIn,double resolution);
		virtual vector<string> getStationList(vector<StationPair> &stationListOut);
		virtual StationPath getStationPath(string pathNameIn);
                virtual nav_msgs::Path getStart_End(string pathNameIn);
		virtual bool checkStationPath(string pathNameIn);
		virtual double getSpeedLimit(string pathNameIn){return 0.0;};

	private:

		double resolution_;

		string tildeExpand(const char *path);
	};
}



#endif /* SVGPATH_HPP_ */
