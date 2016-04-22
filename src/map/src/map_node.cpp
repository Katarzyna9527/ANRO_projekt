#include "ros/ros.h"
#include <vector>
#include <cstdint>
#include <iostream>
#include <cstddef>

#include "map/map_config.h"
#include "map/cross_msg.h"

bool getConfig(map::map_config::Request  &req,
         map::map_config::Response &res) {
/*
	auto nodes = new std::vector<map::node>();

	for (auto it = map->nodes.cbegin(); it != map->nodes.cend(); ++it) {
		nodes->push(*new map::node());
		**(nodes->end()).neighbours = **it.getNeighbours();
		**(nodes->end()).lengths = **it.getLengths();
	}

	res.crossings = NULL/nodes;
/
int id
int[] neighbours
int[] lengths
uint8 node_id */
  //ROS_INFO("request: %ld", req.req);
  ROS_INFO("stub");
  return true;
}

class Crossing {

};

class Map
{
	//static Map* singleton;
	//Map() {}
	
	//std::vector<Crossing*>  crossings;

//public:
	//Map* getInstance() {}

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_config");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("get_map_conifg", getConfig);
  ROS_INFO("Map ready");
  ros::spin();

  return 0;
}
