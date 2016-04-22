#include "ros/ros.h"
#include <vector>
#include <uint8>


#include "map/msg/node.msg"
#include "map/srv/map_config.msg"

bool getConfig(map::GetMapConfig::Request  &req,
         map::GetMapConfig::Response &res)
{
	auto nodes = new std::vector<map::node>();

	for (auto it = map->nodes.cbegin(); it != map->nodes.cend(); ++it) {
		nodes->push(*new map::node());
		**(nodes->end()).neighbours = **it.getNeighbours();
		**(nodes->end()).lengths = **it.getLengths();
	}

	res.nodes = *nodes;
int id
int[] neighbours
int[] lengths
uint8 node_id
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

class Crossing;

class Map
{
	static Map* singleton = nullptr;
	Map();
	
	std::vector<Crossing*>  crossings;

public:
	Map* getInstance();

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("get_map_conifg", getConfig);
  ROS_INFO("Map ready");
  ros::spin();

  return 0;
}
