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
double dist_next_goal_ = 6.0;
std::vector<int>* store_loop_;
int count;
int sequence_, path_number_;

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
        std::cout<<dist_goal<<std::endl;
//        std_msgs::Int32 first_data;
//        first_data.data = 12;
//        pub_next_goal.publish(12);
        if (dist_goal<dist_next_goal_)
        {
            std_msgs::Int32 new_loop;
            new_loop.data = store_loop_->at(count);
            std::cout<<"waiting for 5.0 seconds"<<std::endl;
            ros::Duration(5.0).sleep();

            if (count<path_number_)
            {
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

  priv_nh.getParam("sequence", sequence_);
  priv_nh.getParam("path_number", path_number_);

  path_received_ = false;


  //ros::Rate t(1000);


  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe <pnc_msgs::move_status>("move_status", 1, move_cb);
  ros::Subscriber sub_path = nh.subscribe <nav_msgs::Path> ("route_plan", 1, path_receive_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  pub_next_goal = nh.advertise<std_msgs::Int32> ("station_sequence", 1000);

    //ros::Duration(5.0).sleep();


      //send_msg();
    int divider=0;
    divider = pow(10, path_number_);
    std::cout<<divider<<std::endl;

  store_loop_ = new std::vector<int>;

  if (path_number_==0 || sequence_==0)
  {
      return 0;
  }
  else if (sequence_/divider == 0)
  {

      for (int i=0; i<path_number_-1; i++)
      {
          int remainder = 0;
          int remainder_2 = 0;
          int path = 0;
          int send_goal = path_number_-i;
          divider = pow(10, send_goal);
          remainder = sequence_/divider;
          remainder_2 = sequence_-remainder*divider;
          send_goal= send_goal-1;
          divider = pow(10, send_goal);
          remainder_2 = remainder_2/divider;
          path = remainder*10 + remainder_2;
          store_loop_->push_back(path);

          sequence_ = sequence_ - remainder*(pow(10, send_goal+1));

          std::cout<<path<<std::endl;
          std::cout<<sequence_<<std::endl;
      }

      std::cout<<sequence_/divider<<std::endl;
  }
  else
  {
      std::cout<<sequence_/divider<<std::endl;
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

