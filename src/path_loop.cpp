#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pnc_msgs/move_status.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>

ros::Publisher pub, pub_next_goal;

bool path_received_;
double dist_next_goal_;
std::vector<int>* store_loop_;
int count;

void move_cb (const pnc_msgs::move_status to_goal)
{
    if (path_received_)
    {
        double dist_goal = to_goal.dist_to_goal.data;
        if (dist_goal<dist_next_goal_)
        {
            std_msgs::Int32 new_loop;
            new_loop.data = store_loop_->at(count);
            pub_next_goal.publish(new_loop);
            path_received_=false;
            count++;
        }
    }

}

void path_receive_cb (const nav_msgs::Path path)
{
    path_received_ = true;
    std::cout<<path_received_<<std::endl;
}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...
  output = *input;

  // Publish the data.
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "path_loop");
  ros::NodeHandle nh;

  path_received_ = false;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe <pnc_mgs::move_status>("/iMiev/move_status", 1, move_cb);
  ros::Subscriber sub_path = nh.subscribe <nav_msgs::Path> ("/iMiev/route_plan", 1, receive_path_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  pub_next_goal = nh.advertise<std_msgs::Int32> ("station_sequence", 1);

  store_loop_ = new std::vector<int>;
  store_loop_->push_back(1);
  store_loop_->push_back(12);
  store_loop_->push_back(23);
  store_loop_->push_back(34);

  count=0;

  // Spin
  ros::spin ();
}

