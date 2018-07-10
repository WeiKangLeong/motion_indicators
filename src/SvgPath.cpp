/*
 * SvgPath.cpp
 *
 *  Created on: 11 Jul, 2014
 *      Author: liuwlz
 */

#ifndef SVGPATH_CPP_
#define SVGPATH_CPP_

#include <motion_indicators/SvgPath.hpp>

#define NANOSVG_IMPLEMENTATION
#include "motion_indicators/nanosvg.h"
#include "motion_indicators/bezierArcLength.h"

#include <tf/transform_datatypes.h>

class BezierPath {
  string id_;
  vector<MissionPlan::PathPoint> poses_;
  double length_;
public:
  BezierPath(string id):id_(id), length_(0.0){};
  
  void appendPathWithControlPoints(float* p, double res, double height){
    //here the 8 points (4x + 4y) obtained from svg file is translated into equation space
    vector<double> x(4);
    vector<double> y(4);
    
    //convert from graph space to equation space (http://www.tinaja.com/glib/cubemath.pdf)
    for(int j=0; j<8; j+=2){
      x[j/2] = p[j];
      y[j/2] = p[j+1];
    }
    double A, B, C, D, E, F, G, H;
    A = x[3]-3*x[2]+3*x[1]-x[0];
    B = 3*x[2]-6*x[1]+3*x[0];
    C = 3*x[1]-3*x[0];
    D = x[0];
    E = y[3]-3*y[2]+3*y[1]-y[0];
    F = 3*y[2]-6*y[1]+3*y[0];
    G = 3*y[1]-3*y[0];
    H = y[0];
    double length = BezierArcLength::arclen(x,y,1e-6);
    //the path will be rasterized at the same resolution of the given map
    //note because t parameterization is non-linear, distance between 2 points is not constant
    double inc = 1.0/length;
    length_ += length;
//     cout<<"inc="<<inc<<" length="<<length<<endl;
    for(double t=0; t<=1.0; t+=inc){
      double x = (((A*t)+B)*t+C)*t+D;
      double y = (((E*t)+F)*t+G)*t+H;
      double x_der = (3*A*t+2*B)*t+C;
      double y_der = (3*E*t+2*F)*t+G;
      //-y_der to reversed the y-direction in image coordinate
      double yaw = atan2(-y_der,x_der);
//       cout<<x*res<<"\t"<<(height-y)*res<<"\t"<<x_der<<"\t"<<y_der<<"\t"<<yaw<<endl;
      MissionPlan::PathPoint p;
      p.x_ = x*res;
      p.y_ = (height-y)*res;
      p.yaw_ = yaw;
      poses_.push_back(p);
    }
    
  };
  
  vector<MissionPlan::PathPoint> getRasterizedPath(){
    return poses_;
  };
  
  double getPathLength(){
    return length_;
  }
  
};

namespace MissionPlan {

	NSVGimage *svg_doc_;
	StationPath::
	StationPath():
	exist_(false),
	distance_(DBL_MAX),
	speed_limit_(0.0),
	congestion_(DBL_MAX){

	}

	StationPath&
	StationPath::
	operator=(const StationPath &stationPathIn){
		if (this == &stationPathIn)
			return *this;
		exist_ = stationPathIn.exist_;
		distance_ = stationPathIn.distance_;
		congestion_ = stationPathIn.congestion_;
		speed_limit_ = stationPathIn.speed_limit_;
		point_set_ = stationPathIn.point_set_;
		return *this;
	}

	StationPath&
	StationPath::
	operator+(const StationPath &stationPathIn){
		exist_ = exist_ && stationPathIn.exist_;
		distance_ += stationPathIn.distance_;
		congestion_ += stationPathIn.congestion_;
		speed_limit_ = speed_limit_ < stationPathIn.speed_limit_ ? speed_limit_ : stationPathIn.speed_limit_;
		point_set_.insert(point_set_.end(),stationPathIn.point_set_.begin(), stationPathIn.point_set_.end());
		return *this;
	}

	SvgPath::
	SvgPath(){

	}

	SvgPath::
	SvgPath(const char* fileNameIn, double resolution):
	resolution_(resolution){
		svg_doc_ = NULL;
		loadFile(fileNameIn, resolution);
	}

	SvgPath::
	~SvgPath(){

	}

	string
	SvgPath::
	tildeExpand(const char *path){
		wordexp_t p;
		wordexp(path, &p, 0);
		char **w = p.we_wordv;
		assert(p.we_wordc==1);
		string res = w[0];
		wordfree(&p);
		return res;
	}

	void
	SvgPath::
	loadFile(const char* fileNameIn, double resolution){
	  
	  svg_doc_ = nsvgParseFromFile(tildeExpand(fileNameIn).c_str(), "px", 96.0);
	  if(svg_doc_ == NULL) cout<<"Failed to load svg "<<fileNameIn<<endl;
	  int w, h;
	  w = svg_doc_->width;
	  h = svg_doc_->height;
	  cout<<"Path file size of "<<w<<"x"<<h<<" loaded."<<endl;
	  
	}

	vector<string>
	SvgPath::
	getStationList(vector<StationPair> &stationListOut){
	  int num_station = 0;
	  NSVGshape* shape;
	  NSVGpath* path;
	  vector<string> major_station_name;

	  for (shape = svg_doc_->shapes; shape != NULL; shape = shape->next) {
	      for (path = shape->paths; path != NULL; path = path->next) {
		string path_id = path->id;
		
		//getting the major stations
		if(strncmp(path->id, "ST", 2) ==0)
		{
			major_station_name.push_back(path_id.substr(2));
		}
		//getting the station name by the name of the link,i.e., a link named A_B, will add two stations A,B
		size_t found_pos = path_id.find("_");
		if(found_pos == string::npos)
		{
			continue;
		}
		else
		{
		  string from_station = path_id.substr(0,found_pos);
		  string to_station = path_id.substr(found_pos+1);
		  //to test whether the station is already in the list
	          bool add_from_station = true;
   		  bool add_to_station = true;
		  for(size_t i = 0; i < stationListOut.size();i++)
		  {
			if(from_station.compare(stationListOut[i].first) == 0)
			{
				add_from_station = false;
			}
			if(to_station.compare(stationListOut[i].first)==0)
			{
				add_to_station = false;
			}
		  }
		  if(add_from_station)
		  {
			  BezierPath b_path(from_station);
			  for (int i = 0; i < path->npts-1; i += 3) {
			      float* p = &path->pts[i*2];
			      b_path.appendPathWithControlPoints(p, resolution_, svg_doc_->height);
			  }
			  stationListOut.push_back(make_pair(from_station, (b_path.getRasterizedPath()).front()));
			  num_station++;
		  }
		  if(add_to_station)
		  {
			  BezierPath b_path(to_station);
			  for (int i = 0; i < path->npts-1; i += 3) {
			      float* p = &path->pts[i*2];
			      b_path.appendPathWithControlPoints(p, resolution_, svg_doc_->height);
			  }
			  stationListOut.push_back(make_pair(to_station, (b_path.getRasterizedPath()).back()));
			  num_station++;
		  }


		}
	      }
	  }
	  cout <<"Number of stations: "<<num_station<<endl;
	  return major_station_name;
	}

	StationPath
	SvgPath::
	getStationPath(string pathNameIn){
	  StationPath station_path;
	  NSVGshape* shape;
	  NSVGpath* path;
	  for (shape = svg_doc_->shapes; shape != NULL; shape = shape->next) {
	      for (path = shape->paths; path != NULL; path = path->next) {
		string path_id = path->id;
		if(strcmp(path->id, pathNameIn.c_str()) ==0){
		  
		  BezierPath b_path(path_id);
		  for (int i = 0; i < path->npts-1; i += 3) {
		      float* p = &path->pts[i*2];
		      b_path.appendPathWithControlPoints(p, resolution_, svg_doc_->height);
		  }
		  station_path.point_set_ = b_path.getRasterizedPath();
		  station_path.distance_ = b_path.getPathLength();
		  station_path.congestion_ = 5.0;
		  station_path.speed_limit_ = 2.0;
		  station_path.exist_ = true;
		}
	      }
	  }
	  return station_path;
	}

        nav_msgs::Path
        SvgPath::
        getStart_End(string pathNameIn){
          StationPath station_path;
          NSVGshape* shape;
          NSVGpath* path;
          for (shape = svg_doc_->shapes; shape != NULL; shape = shape->next) {
              for (path = shape->paths; path != NULL; path = path->next) {
                string path_id = path->id;
                if(strcmp(path->id, pathNameIn.c_str()) ==0){

                  BezierPath b_path(path_id);
                  for (int i = 0; i < path->npts-1; i += 3) {
                      float* p = &path->pts[i*2];
                      b_path.appendPathWithControlPoints(p, resolution_, svg_doc_->height);
                  }
                  station_path.point_set_ = b_path.getRasterizedPath();
                  station_path.distance_ = b_path.getPathLength();
                  station_path.congestion_ = 5.0;
                  station_path.speed_limit_ = 2.0;
                  station_path.exist_ = true;
                }
              }
          }
          nav_msgs::Path goal_path;

          for (vector<PathPoint>::iterator iter = station_path.point_set_.begin(); iter!=station_path.point_set_.end(); iter++)
          {
              geometry_msgs::PoseStamped current_pose;
              current_pose.pose.position.x = iter->x_;
              current_pose.pose.position.y = iter->y_;
              tf::Quaternion orientation_qt;
              orientation_qt.setRPY(iter->yaw_, 0, 0);
              current_pose.pose.orientation.x = orientation_qt.x();
              current_pose.pose.orientation.y = orientation_qt.y();
              current_pose.pose.orientation.z = orientation_qt.z();
              current_pose.pose.orientation.w = orientation_qt.w();
              goal_path.poses.push_back(current_pose);
          }
          goal_path.header.frame_id = "map";
          goal_path.header.stamp = ros::Time::now();
          return goal_path;
          //return station_path;
        }

	bool
	SvgPath::
	checkStationPath(string pathNameIn){
	  NSVGshape* shape;
	  NSVGpath* path;
	  for (shape = svg_doc_->shapes; shape != NULL; shape = shape->next) {
	      for (path = shape->paths; path != NULL; path = path->next) {
		string path_id = path->id;
		if(strcmp(path->id, pathNameIn.c_str()) ==0){
		  return true;
		}
	      }
	  }
	    return false;
	}


} // namespace Mission_Plan

#endif /* SVGPATH_CPP_ */
