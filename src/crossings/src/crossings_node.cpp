#include "ros/ros.h"
#include "crossings/add_crossing.h"
#include "crossings/cross_msg.h"

bool add(crossings::add_crossing::Request  &req,
         crossings::add_crossing::Response &res)
{

	// place some code to get data from MAP about new CROSSING
	ROS_INFO("Get some data over \'add_crossing\' service :\n");
	for(int i = 0 ; i < req.crossingsData.size() ; i++)
	{
		ROS_INFO("ID:%d NeighSize:%ld LengthSize:%ld", 
			req.crossingsData[i].ID,
			req.crossingsData[i].neighbours.size(),
			req.crossingsData[i].lengths.size());
	}
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "crossings_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_crossing_service", add);
  ROS_INFO("Ready to add new crossings.");
 
  ros::spin();

 return 0;
}
