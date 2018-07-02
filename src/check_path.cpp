#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/Path.h>

ros::Publisher pub;

nav_msgs::Path stored_route;

std::vector<std::vector<int>* >* path_;

void store_path (const nav_msgs::Path path_in)
{
    stored_route = path_in;

    int path_fraction = stored_route.poses.size();

    double path_gradient;

    for (int i=1; i<path_fraction-1; i++)
    {
        double node_x0 = stored_route.poses[i-1].pose.position.x;
        double node_y0 = stored_route.poses[i-1].pose.position.y;
        double node_x1 = stored_route.poses[i].pose.position.x;
        double node_y1 = stored_route.poses[i].pose.position.y;
        double node_x2 = stored_route.poses[i+1].pose.position.x;
        double node_y2 = stored_route.poses[i+1].pose.position.y;
        double del_x1 = sqrt((node_x1-node_x0)*(node_x1-node_x0));
        double del_y1 = sqrt((node_y1-node_y0)*(node_y1-node_y0));
        double del_x2 = sqrt((node_x2-node_x1)*(node_x2-node_x1));
        double del_y2 = sqrt((node_y2-node_y1)*(node_y2-node_y1));
        double grad_1 = atan(del_y1/del_x1)*180/PI;
        double grad_2 = atan(del_y2/del_x2)*180/PI;
        std::cout<<"grad 1: "<<grad_1<<" "<<"grad 2: "<<grad_2<<std::endl;
        if (std::abs(grad_1-grad_2)<5.0)
        {

        }

    }
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
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
  ros::Subscriber sub_path = nh.subscribe ("route_path", 1, store_path);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  path_ = new std::vector<std::vector<int>* >;

  // Spin
  ros::spin ();
}

