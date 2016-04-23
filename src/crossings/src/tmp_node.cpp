#include "ros/ros.h"
#include "crossings/add_crossing.h"
#include "crossings/cross_msg.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "add_crossing_client");
	ros::NodeHandle n;

	ros::ServiceClient client = n.serviceClient<crossings::add_crossing>("add_crossing_service");
	crossings::add_crossing srv;
	
	crossings::cross_msg msg;
	msg.ID = 10;
	msg.neighbours.push_back(1122);
	msg.lengths.push_back(10);
        
	srv.request.crossingsData.push_back(msg);
	if(client.call(srv))
	{
		ROS_INFO("Poszło!");
	} 
	else
	{	
		ROS_INFO("Nie poszło!");
		return -1;
	}

	return 0;
}
