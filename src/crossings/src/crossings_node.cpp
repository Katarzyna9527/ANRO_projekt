#include "ros/ros.h"
#include "crossings/add_crossing.h"
#include "crossings/cross_msg.h"

bool add(crossings::add_crossing::Request  &req,
         crossings::add_crossing::Response &res)
{

	// place some code to get data from MAP about new CROSSING

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "crossings_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_crossing", add);
  ROS_INFO("Ready to add new crossings.");
 
  ros::spin();

 return 0;
}
