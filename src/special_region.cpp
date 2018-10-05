#include <stdlib.h>
#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <tinyxml.h>
#include <wordexp.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt8.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>

#include <boost/shared_ptr.hpp>

#include <motion_indicators/SvgPath.hpp>

#define SVG_FILE "~/SvgPath/path_def.svg"
#define RESOLUTION 0.1

//1 - right, 2 - left, 3 - dual, 0 - no
#define left_turn 1
#define right_turn 2
#define no_turn 0
#define dual_light 3

#ifdef TINYXML_API_PRE26
#define TINYXML_ELEMENT ELEMENT
#define TINYXML_TEXT TEXT
#endif

using namespace std;
using namespace MissionPlan;

ros::Publisher pub_indicators, pub_visualizer;

double resolution_;
boost::shared_ptr<TiXmlDocument> svg_doc_;

shared_ptr<SvgPath> svg_path_;

tf::TransformListener *tfl_;

std::vector<std::vector<double>* >* start_left_;
std::vector<std::vector<double>* >* start_right_;
std::vector<std::vector<double>* >* end_left_;
std::vector<std::vector<double>* >* end_right_;
std::vector<std::vector<double>* >* store_brake_;
std::vector<std::string>* station_name_;
pcl::PointCloud<pcl::PointXYZI>::Ptr store_indicators_(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr path_indicators_(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr right_indicators_(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr left_indicators_(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr dual_indicators_(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr no_indicators_(new pcl::PointCloud<pcl::PointXYZI>);

//pcl::PointCloud<pcl::PointXYZI>::Ptr start_end_pose_(new pcl::PointCloud<pcl::PointXYZI>);

std_msgs::UInt8 three_indicators_;

nav_msgs::Path path_received_;

std::vector<pcl::PointCloud<pcl::PointXYZI> >* route_station_;
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr >* start_end_pose_;
std::vector<string>* start_end_name_;

void blink(int state)
{

    //tf::Vector3 car_center = latest_odom_transform.getOrigin();

    visualization_msgs::Marker line_list;
    line_list.header.frame_id =  "/iMiev/base_link";
    line_list.ns = "warning_light";
    line_list.action = 3;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.id = 0;
    line_list.scale.x = 0.1;
    line_list.color.r = 0.98;
    line_list.color.g = 0.98;
    line_list.color.b = 0.823;
    line_list.color.a = 1.0;

    geometry_msgs::Point light_pt;

    pcl::PointCloud<pcl::PointXYZI>::Ptr around_car (new pcl::PointCloud<pcl::PointXYZI>);


    switch (state) {
        case left_turn:
            //pcl_ros::transformPointCloud(*left_indicators_, *around_car, latest_odom_transform);
            for (int k=0; k<left_indicators_->size(); k++)
            {
                light_pt.x = left_indicators_->points[k].x;
                light_pt.y = left_indicators_->points[k].y;
                line_list.points.push_back(light_pt);
            }break;
        case right_turn:
            //pcl_ros::transformPointCloud(*right_indicators_, *around_car, latest_odom_transform);
            for (int k=0; k<right_indicators_->size(); k++)
            {
                light_pt.x = right_indicators_->points[k].x;
                light_pt.y = right_indicators_->points[k].y;
                line_list.points.push_back(light_pt);
            }break;
    }

    pub_visualizer.publish(line_list);
    line_list.points.clear();
}

void store_path(const nav_msgs::Path new_path)
{
    path_received_ = new_path;
    path_indicators_->clear();
    three_indicators_.data = 0;
    //pub_indicators.publish(three_indicators_);

    pcl::PointXYZ input_start, input_end;
    input_start.x = path_received_.poses[0].pose.position.x;
    input_start.y = path_received_.poses[0].pose.position.y;
    input_end.x = path_received_.poses[path_received_.poses.size()-1].pose.position.x;
    input_end.y = path_received_.poses[path_received_.poses.size()-1].pose.position.y;

    for (int nosuke=0; nosuke<start_end_pose_->size(); nosuke++)
    {
        double st_x = start_end_pose_->at(nosuke)->points[0].x;
        double st_y = start_end_pose_->at(nosuke)->points[0].y;
        double ed_x = start_end_pose_->at(nosuke)->points[1].x;
        double ed_y = start_end_pose_->at(nosuke)->points[1].y;
        double start_range = sqrt((input_start.x-st_x)*(input_start.x-st_x)+(input_start.y-st_y)*(input_start.y-st_y));
        double end_range = sqrt((input_end.x-ed_x)*(input_end.x-ed_x)+(input_end.y-ed_y)*(input_end.y-ed_y));
        cout<<"from start: "<<start_range<<" from end: "<<end_range<<endl;
        cout<<"this path station: "<<route_station_->at(nosuke).size()<<endl;
        if (start_range<0.2 && end_range<0.2)
        {
            *path_indicators_ = route_station_->at(nosuke);
        }

    }

    //std::cout<<path_received_.poses.size()<<std::endl;
    //std::cout<<path_received_.poses[0].pose.position.x<<std::endl;
    /*for (int i=0; i<store_indicators_->size(); i++)
    {
        double point_x = store_indicators_->points[i].x;
        double point_y = store_indicators_->points[i].y;

        for (int j=0; j<path_received_.poses.size(); j++)
        {
            double path_x = path_received_.poses[j].pose.position.x;
            double path_y = path_received_.poses[j].pose.position.y;
            double on_the_path = sqrt((point_x-path_x)*(point_x-path_x)+(point_y-path_y)*(point_y-path_y));
            if (on_the_path<0.5)
            {
                path_indicators_->push_back(store_indicators_->points[i]);
                break;
            }
        }
    }*/
    cout<<"total special region: "<<path_indicators_->size()<<endl;
}

void check_region(const geometry_msgs::PoseWithCovarianceStamped amcl)
{
    double cur_x = amcl.pose.pose.position.x;
    double cur_y = amcl.pose.pose.position.y;

    for (int i=0; i<path_indicators_->size(); i++)
    {
        double ind_x = path_indicators_->points[i].x;
        double ind_y = path_indicators_->points[i].y;
        double current_dist = sqrt((cur_x-ind_x)*(cur_x-ind_x) + (cur_y-ind_y)*(cur_y-ind_y));
        //cout<<"index "<<i<<" : "<<current_dist<<" current indicator: "<<int(three_indicators_.data)<<endl;
        if (current_dist<2.0)
        {

            if (three_indicators_.data == path_indicators_->points[i].intensity)
            {
                three_indicators_.data=path_indicators_->points[i].intensity;
            }
            else
            {
                three_indicators_.data=path_indicators_->points[i].intensity;
                std::cout<<"publish indicator: "<<three_indicators_.data<<std::endl;
                pub_indicators.publish(three_indicators_);
            }
        }
    }
    blink(int(three_indicators_.data));
    //cout<<"current indicator: "<<int(three_indicators_.data)<<endl;
}

string tildeExpand(const string& stringIn){
		wordexp_t p;
		wordexp(stringIn.c_str(), &p, 0);
		char **w = p.we_wordv;
		assert(p.we_wordc==1);
		string stringOut = w[0];
		wordfree(&p);
		return stringOut;
	}

vector<int> getMapBound(){
		TiXmlElement* svgElement;
		svgElement = svg_doc_->FirstChildElement();

		double width,height;
		if( svgElement->QueryDoubleAttribute("width", &width)==TIXML_NO_ATTRIBUTE
				|| svgElement->QueryDoubleAttribute("height", &height)==TIXML_NO_ATTRIBUTE ){
			throw ("Height or width information not found");
		}
		vector<int> bound(2);
		bound[0] = width;
		bound[1] = height;
		return bound;
	}
	
void loadIndicatorStation()
{
    cout<<"Load Indicator Region!"<<endl;
    int stop_line_id = 0;
    TiXmlElement* svgElement;
    svgElement = svg_doc_->FirstChildElement();
    TiXmlNode* svg_node_;

    /** store path start and end point **/
    for ( svg_node_ = svgElement->FirstChild(); svg_node_ != 0; svg_node_ = svg_node_->NextSibling())
    {
        TiXmlElement* childElement= svg_node_->ToElement();
        if(strcmp(childElement->Value(),"path")==0)
        {
            const char* value = childElement->Attribute("id");
            assert( value!=NULL );
            if ( strncmp("ST",value,2)== 0 )
            {
                //char * temp_value = std::strcpy(temp_value, value);
                //std::string id_value = temp_value;
                std::string id_value = value;
                //char * id_value = std::strcpy(id_value, value);
                std::string station = id_value.substr(2);
                station_name_->push_back(station);
            }
        }
    }
    cout<<"station number: "<<station_name_->size()<<endl;
    for (int s=0; s<station_name_->size(); s++)
    {
        cout<<"station "<<s<<" : "<<station_name_->at(s)<<endl;
    }

    for ( svg_node_ = svgElement->FirstChild(); svg_node_ != 0; svg_node_ = svg_node_->NextSibling())
    {
        TiXmlElement* childElement= svg_node_->ToElement();
        if(strcmp(childElement->Value(),"path")==0)
        {
            const char* value = childElement->Attribute("id");
            assert( value!=NULL );
            for (int i=0; i<station_name_->size(); i++)
            {
                std::size_t pos = station_name_->at(i).length();
                if ( strncmp(station_name_->at(i).c_str(),value,pos)== 0 )
                {
                    //cout<<value<<endl;
                    const char * d_content = value;
                    std::string d_string = d_content;
                    char * d_char = new char [d_string.length()+1];
                    strcpy(d_char, d_string.c_str());
                    //char * d_element = std::strtok (d_content, " ");
                    char * d_element = std::strtok (d_char, "_");
                    char * first_element = d_element;
                    //cout<<first_element<<endl;
                    d_element = std::strtok(NULL,"_");
                    char * second_element = d_element;
                    //cout<<second_element<<endl;

                    for (int j=0; j<station_name_->size(); j++)
                    {
                        std::size_t pos_2 = station_name_->at(j).length();
                        if ( strncmp(station_name_->at(j).c_str(),second_element,pos_2)==0
                             && !(strncmp(station_name_->at(j).c_str(),first_element,pos)==0))
                        {
                            nav_msgs::Path route_in;
                            route_in = svg_path_->getStart_End(value);
                            //cout<<route_in.poses[0].pose.position.x<<" "<<route_in.poses[0].pose.position.y<<endl;
                            int path_node_size = route_in.poses.size();
                            //cout<<path_node_size<<": "<<route_in.poses[path_node_size-1].pose.position.x<<" "<<route_in.poses[path_node_size-1].pose.position.y<<endl;
                            pcl::PointCloud<pcl::PointXYZI>::Ptr node_pose(new pcl::PointCloud<pcl::PointXYZI>);
                            pcl::PointXYZI node;
                            node.x = route_in.poses[0].pose.position.x;
                            node.y = route_in.poses[0].pose.position.y;

                            node_pose->push_back(node);
                            node.x = route_in.poses[path_node_size-1].pose.position.x;
                            node.y = route_in.poses[path_node_size-1].pose.position.y;
                            node_pose->push_back(node);
                            start_end_pose_->push_back(node_pose);
                            start_end_name_->push_back(d_string);
                            node_pose->clear();
                        }
                    }

                    /*const char * d_content = (childElement->Attribute("d"));
                    std::string d_string = d_content;
                    char * d_char = new char [d_string.length()+1];
                    strcpy(d_char, d_string.c_str());
                    //char * d_element = std::strtok (d_content, " ");
                    char * d_element = std::strtok (d_char, " ");
                    if (strcmp(d_element,"D"))
                    {
                        pcl::PointCloud<pcl::PointXYZI>::Ptr node_pose;
                        node_pose->resize(2);
                        char * first_pose = std::strtok(NULL," ");
                        char * first_pose_xy = std::strtok(first_pose, ",");
                        node_pose->points[0].x = atof(first_pose_xy);
                        first_pose_xy = std::strtok(NULL,",");
                        node_pose->points[0].y = atof(first_pose_xy);
                        char * end_pose = first_pose;
                        while(first_pose!=0)
                        {
                            end_pose = first_pose;
                            first_pose = std::strtok(NULL," ");
                        }
                        char * end_pose_xy = std::strtok(first_pose, ",");
                        node_pose->points[1].x = atof(end_pose_xy);
                        end_pose_xy = std::strtok(NULL,",");
                        node_pose->points[1].y = atof(end_pose_xy);
                        start_end_pose_->push_back(node_pose);
                    }
                    else if (strcmp(d_element,"d"))
                    {
                        pcl::PointCloud<pcl::PointXYZI>::Ptr node_pose;
                        node_pose.resize(2);
                        char * first_pose = std::strtok(NULL," ");
                        char * first_pose_xy = std::strtok(first_pose, ",");
                        node_pose->points[0].x = atof(first_pose_xy);
                        first_pose_xy = std::strok(NULL,",");
                        node_pose->points[0].y = atof(first_pose_xy);
                        char * end_pose = first_pose;
                        while(first_pose!=0)
                        {
                            end_pose = first_pose;
                            first_pose = std::strtok(NULL," ");
                        }
                        char * end_pose_xy = std::strtok(first_pose, ",");
                        node_pose->points[0].x = atof(end_pose_xy);
                        end_pose_xy = std::strok(NULL,",");
                        node_pose->points[0].y = atof(end_pose_xy);
                    }*/
                }
            }
        }
    }
    cout<<"route number: "<<start_end_name_->size()<<endl;
    //route_station_->resize(start_end_name_->size());
    pcl::PointCloud<pcl::PointXYZI> empty_cloud;
    for (int k=0; k<start_end_pose_->size(); k++)
    {
        cout<<start_end_name_->at(k)<<" x: "<<start_end_pose_->at(k)->points[0].x<<" y: "<<start_end_pose_->at(k)->points[0].y<<endl;
        cout<<start_end_name_->at(k)<<" x: "<<start_end_pose_->at(k)->points[1].x<<" y: "<<start_end_pose_->at(k)->points[1].y<<endl;
        route_station_->push_back(empty_cloud);
        //route_station_->resize(start_end_name_->size());
    }



    TiXmlNode* svg_node_2;

    for ( svg_node_2 = svgElement->FirstChild(); svg_node_2 != 0; svg_node_2 = svg_node_2->NextSibling())
    {
        TiXmlElement* childElement= svg_node_2->ToElement();
        if(strcmp(childElement->Value(),"ellipse")==0 || strcmp(childElement->Value(),"circle")==0)
        {
            const char* value = childElement->Attribute("id");
            assert( value!=NULL );
            std::vector<double>* new_area = new std::vector<double>;
            pcl::PointXYZI indicator_point;
            new_area->resize(2);
            if( strncmp("RS",value,2)== 0 )
            {
                cout <<"Find indicator region with id: " << value << endl;
                std::string route_value = value;
                //char * id_value = std::strcpy(id_value, value);
                std::string route = route_value.substr(3);

                for (int momo=0; momo<start_end_name_->size(); momo++)
                {
                    if (route==start_end_name_->at(momo))
                    {
                        int position_x_pixel = atof(childElement->Attribute("cx"));
                        int position_y_pixel = atof(childElement->Attribute("cy"));
                        //get the transform attribute so that we can move the stop point
                        double offset_x = 0.0;
                        double offset_y = 0.0;
                        const char *  path_transform = (childElement->Attribute("transform"));
                        if(path_transform !=NULL)
                        {
                           string path_transform_str(path_transform);
                           cout<<"    *path tranform is "<<path_transform_str<<endl;

                           std::replace(path_transform_str.begin(),path_transform_str.end(),'(',' ');
                           std::replace(path_transform_str.begin(),path_transform_str.end(),')',' ');
                           std::replace(path_transform_str.begin(),path_transform_str.end(),',',' ');
                           std::stringstream ss(path_transform_str);

                           string tmp_str;
                           ss>>tmp_str>>offset_x>>offset_y;
                           cout<<"    *"<<tmp_str<<" is "<<offset_x<<","<<offset_y<<endl;

                        }

                        indicator_point.x = (position_x_pixel + offset_x)*resolution_;
                        indicator_point.y = (getMapBound()[1] - position_y_pixel - offset_y)*resolution_;
                        indicator_point.z = 0.0;
                        indicator_point.intensity = right_turn;
                        //right_indicators_->push_back(indicator_point);
                        route_station_->at(momo).push_back(indicator_point);
                        stop_line_id ++;
                        cout<<indicator_point.x<<" "<<indicator_point.y<<endl;
                        break;
                    }
                }

            }
            else if( strncmp("LS",value,2)== 0 )
            {
                cout <<"Find indicator region with id: " << value << endl;
                std::string route_value = value;
                //char * id_value = std::strcpy(id_value, value);
                std::string route = route_value.substr(3);

                for (int momo=0; momo<start_end_name_->size(); momo++)
                {
                    if (route==start_end_name_->at(momo))
                    {
                        int position_x_pixel = atof(childElement->Attribute("cx"));
                        int position_y_pixel = atof(childElement->Attribute("cy"));
                        //get the transform attribute so that we can move the stop point
                        double offset_x = 0.0;
                        double offset_y = 0.0;
                        const char *  path_transform = (childElement->Attribute("transform"));
                        if(path_transform !=NULL)
                        {
                           string path_transform_str(path_transform);
                           cout<<"    *path tranform is "<<path_transform_str<<endl;

                           std::replace(path_transform_str.begin(),path_transform_str.end(),'(',' ');
                           std::replace(path_transform_str.begin(),path_transform_str.end(),')',' ');
                           std::replace(path_transform_str.begin(),path_transform_str.end(),',',' ');
                           std::stringstream ss(path_transform_str);

                           string tmp_str;
                           ss>>tmp_str>>offset_x>>offset_y;
                           cout<<"    *"<<tmp_str<<" is "<<offset_x<<","<<offset_y<<endl;

                        }

                        indicator_point.x = (position_x_pixel + offset_x)*resolution_;
                        indicator_point.y = (getMapBound()[1] - position_y_pixel - offset_y)*resolution_;
                        indicator_point.z = 0.0;
                        indicator_point.intensity = left_turn;
                        //left_indicators_->push_back(indicator_point);
                        route_station_->at(momo).push_back(indicator_point);
                        stop_line_id ++;
                        cout<<indicator_point.x<<" "<<indicator_point.y<<endl;
                        break;
                    }
                }
            }
            else if( strncmp("RE",value,2)== 0 || strncmp("LE",value,2)== 0)
            {
                cout <<"Find indicator region with id: " << value << endl;
                std::string route_value = value;
                //char * id_value = std::strcpy(id_value, value);
                std::string route = route_value.substr(3);

                for (int momo=0; momo<start_end_name_->size(); momo++)
                {
                    if (route==start_end_name_->at(momo))
                    {
                        int position_x_pixel = atof(childElement->Attribute("cx"));
                        int position_y_pixel = atof(childElement->Attribute("cy"));
                        //get the transform attribute so that we can move the stop point
                        double offset_x = 0.0;
                        double offset_y = 0.0;
                        const char *  path_transform = (childElement->Attribute("transform"));
                        if(path_transform !=NULL)
                        {
                           string path_transform_str(path_transform);
                           cout<<"    *path tranform is "<<path_transform_str<<endl;

                           std::replace(path_transform_str.begin(),path_transform_str.end(),'(',' ');
                           std::replace(path_transform_str.begin(),path_transform_str.end(),')',' ');
                           std::replace(path_transform_str.begin(),path_transform_str.end(),',',' ');
                           std::stringstream ss(path_transform_str);

                           string tmp_str;
                           ss>>tmp_str>>offset_x>>offset_y;
                           cout<<"    *"<<tmp_str<<" is "<<offset_x<<","<<offset_y<<endl;

                        }

                        indicator_point.x = (position_x_pixel + offset_x)*resolution_;
                        indicator_point.y = (getMapBound()[1] - position_y_pixel - offset_y)*resolution_;
                        indicator_point.z = 0.0;
                        indicator_point.intensity = no_turn;
                        //no_indicators_->push_back(indicator_point);
                        route_station_->at(momo).push_back(indicator_point);
                        stop_line_id ++;
                        cout<<indicator_point.x<<" "<<indicator_point.y<<endl;
                        break;
                    }
                }
            }
        }
    }

    cout <<"Number of stop lines: "<<stop_line_id<<endl;

}
	
void loadSvgFile()
{
		cout << "Loading SvgFile with name: "<<string(SVG_FILE).c_str()<<endl;
	    TiXmlElement* svgElement;
	    if (svg_doc_->LoadFile(tildeExpand(string(SVG_FILE)).c_str())){
	        svgElement = svg_doc_->FirstChildElement();
	        const char * s = svgElement->Value();
	        if( strcmp(s,"svg")==0 ){
	        	cout<<"Svg file loaded"<<endl;
	        }
	        else
	            throw (string("Is SVG file loaded? The value found is ")+s);
	    }
	    else{
	        throw (string("Failed to load file ")+string(SVG_FILE));
	    }
            start_left_->clear();
            start_right_->clear();
	    store_brake_->clear();
            store_indicators_->clear();
            right_indicators_->clear();
            left_indicators_->clear();
            no_indicators_->clear();
            loadIndicatorStation();
	}

int main(int argc, char** argv){
        ros::init(argc, argv,"indicator_region");
	ros::NodeHandle nh;
	
	ros::Subscriber sub_pose = nh.subscribe("input", 1, check_region);
        ros::Subscriber sub_route = nh.subscribe("/iMiev/route_plan", 1, store_path);
        pub_indicators = nh.advertise<std_msgs::UInt8>("/indicators_light", 1);
        pub_visualizer = nh.advertise<visualization_msgs::Marker>("/indicators_visualizer", 1);

        tfl_ = new tf::TransformListener();

        start_left_ = new std::vector<std::vector<double>* >;
        start_right_ = new std::vector<std::vector<double>* >;
        end_left_ = new std::vector<std::vector<double>* >;
        end_right_ = new std::vector<std::vector<double>* >;
        store_brake_ = new std::vector<std::vector<double>* >;
        station_name_ = new std::vector<std::string>;
        route_station_ = new std::vector<pcl::PointCloud<pcl::PointXYZI> >;
        start_end_pose_ = new std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr >;
        start_end_name_ = new std::vector<string>;

        three_indicators_.data = 0;

        pcl::PointXYZI pt_line;
        pt_line.x = 0.5;
        pt_line.y = -2.0;
        right_indicators_->push_back(pt_line);
        pt_line.x = -0.5;
        pt_line.y = -2.0;
        right_indicators_->push_back(pt_line);
        pt_line.x = 1.5;
        pt_line.y = -3.0;
        right_indicators_->push_back(pt_line);
        pt_line.x = -1.5;
        pt_line.y = -3.0;
        right_indicators_->push_back(pt_line);
        pt_line.x = 2.5;
        pt_line.y = -4.0;
        right_indicators_->push_back(pt_line);
        pt_line.x = -2.5;
        pt_line.y = -4.0;
        right_indicators_->push_back(pt_line);

        pt_line.x = 0.5;
        pt_line.y = 2.0;
        left_indicators_->push_back(pt_line);
        pt_line.x = -0.5;
        pt_line.y = 2.0;
        left_indicators_->push_back(pt_line);
        pt_line.x = 1.5;
        pt_line.y = 3.0;
        left_indicators_->push_back(pt_line);
        pt_line.x = -1.5;
        pt_line.y = 3.0;
        left_indicators_->push_back(pt_line);
        pt_line.x = 2.5;
        pt_line.y = 4.0;
        left_indicators_->push_back(pt_line);
        pt_line.x = -2.5;
        pt_line.y = 4.0;
        left_indicators_->push_back(pt_line);
	
	svg_doc_ = boost::shared_ptr<TiXmlDocument>(new TiXmlDocument());

	resolution_ = RESOLUTION;
        svg_path_ = shared_ptr<SvgPath>(new SvgPath(string(SVG_FILE).c_str(),resolution_));
	loadSvgFile();
	
	ros::spin();
}
