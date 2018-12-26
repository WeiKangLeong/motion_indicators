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
double dist_next_goal_ = 8.0;
std::vector<int>* store_loop_;
int count;
int path_number_, sequence_, starting_order_;

void send_msg ()
{
    std::cout<<"going to publish"<<std::endl;
    std_msgs::Int32 first_data;
    first_data.data = store_loop_->at(count);

//    for (int i=0; i<100; i++)
//    {
      pub_next_goal.publish(first_data);
      //std::cout<<"publishing "<<first_data<<std::endl;
//    }

      count++;
    std::cout<<"published"<<std::endl;
    //pub_next_goal.publish()
}

void move_cb (const pnc_msgs::move_status to_goal)
{
    if (path_received_) //&& count<=store_loop_->size())
    {
        double dist_goal = double(to_goal.dist_to_goal);
//        std::cout<<dist_goal<<std::endl;
//        std_msgs::Int32 first_data;
//        first_data.data = 12;
//        pub_next_goal.publish(12);
        if (dist_goal<dist_next_goal_)
        {
		std::cout<<"count: "<<count<<" path_number: "<<path_number_<<std::endl;
            std_msgs::Int32 new_loop;
            
            std::cout<<"waiting for 3.0 seconds"<<std::endl;
            ros::Duration(3.0).sleep();

            if (count<path_number_)
            {
		new_loop.data = store_loop_->at(count);
                pub_next_goal.publish(new_loop);
                std::cout<<"publish new path"<<std::endl;
                path_received_=false;
                count++;
            }
            else if (count==path_number_)
            {
                std::cout<<"restart or Enter to exit? ( r / e )"<<std::endl;
                std::string choice;
                std::cin>>choice;
                if (choice=="r")
                {
                    count=0;
			path_received_=false;
                    send_msg();
                }
                else
                {
                    return;
                }
            }

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
  ros::NodeHandle priv_nh ("~");

  priv_nh.getParam("path_sequence", sequence_);
  priv_nh.getParam("path_number", path_number_);
  priv_nh.getParam("starting_station", starting_order_);

  path_received_ = false;


  //ros::Rate t(1000);


  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe <pnc_msgs::move_status>("move_status", 1, move_cb);
  ros::Subscriber sub_path = nh.subscribe <nav_msgs::Path> ("route_plan", 1, path_receive_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  pub_next_goal = nh.advertise<std_msgs::Int32> ("station_sequence", 1000);

    ros::Duration(5.0).sleep();


      //send_msg();

  store_loop_ = new std::vector<int>;

  int store_path = sequence_;

  std::vector<int> store_station_reverse;
  std::vector<int> store_station;

  while (store_path>0)
  {
      int temp_store_path = store_path/10;

      int station_number = store_path - temp_store_path*10;
      store_station_reverse.push_back(station_number);
      store_path = temp_store_path;
      //std::cout<<station_number<<std::endl;
  }

  //std::cout<<store_station_reverse.size()<<std::endl;

  for (int k=1; k<store_station_reverse.size()+1; k++)
  {
      int placement = store_station_reverse.size()-k;
      //std::cout<<"store_station_reverse.size()-k: "<<placement<<std::endl;
      int new_place = store_station_reverse.at(placement);
      //std::cout<<new_place<<std::endl;
      store_station.push_back(new_place);
      //std::cout<<store_station.at(k-1)<<std::endl;
  }


  if (path_number_==0 || sequence_==0)
  {
      return 0;
  }
  else
  {
      store_loop_->push_back(starting_order_*10+store_station.at(0));
      for (int i=0; i<store_station.size()-1; i++)
      {
          store_loop_->push_back(store_station.at(i)*10 + store_station.at(i+1));

      }

  }

  for (int l=0; l<store_loop_->size(); l++)
  {
      std::cout<<store_loop_->at(l)<<std::endl;
  }

 /*
  store_loop_->push_back(1);
  store_loop_->push_back(12);
  store_loop_->push_back(23);
  store_loop_->push_back(34);
  store_loop_->push_back(43);
  store_loop_->push_back(35);
  store_loop_->push_back(56);
  store_loop_->push_back(67);
  
*/
  /*store_loop_->push_back(4);
  store_loop_->push_back(49);
  store_loop_->push_back(95);
  store_loop_->push_back(56);
  store_loop_->push_back(65);
  store_loop_->push_back(57);
  store_loop_->push_back(78);
  store_loop_->push_back(83);*/


  count=0;

  send_msg();

  // Spin
  ros::spin ();
}

