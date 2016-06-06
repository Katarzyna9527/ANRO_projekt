#include "ros/ros.h"
#include <vector>
#include <cstdint>
#include <string>
#include <iostream>
#include <fstream>
#include <cstddef>
#include <ctime>
#include <cstdlib>

#include "anro_msgs/map_config.h"
#include "anro_msgs/car_init.h"
#include "anro_msgs/map_cross_msg.h"

#include "anro_msgs/cross_init.h"

struct Crossing {
	int16_t ID;
	std::vector<int16_t>* neighbours = new std::vector<int16_t>();
	std::vector<int16_t>* lengths = new std::vector<int16_t>();
};

class Map
{
	Map() {}
	Map(const Map&) {}

	int16_t lastCarID = 0;
	int16_t lastCrossingID = 0;
	//shared_ptr
	std::vector<Crossing*> crossings;


public:
	bool configureFromFile(std::string fname)
	{
		//delete &crossings;
		crossings = *new std::vector<Crossing*>();

		std::ifstream conffile;
		//try {
			conffile.open(fname, std::ios::in);
		//}
		if (!conffile.is_open()) return false;

		int16_t rbuf;
		conffile >> rbuf;
		

		while (rbuf != 0) {
			Crossing* crossbuf = new Crossing();
			crossbuf->ID = rbuf;

			crossbuf->neighbours = new std::vector<int16_t>();
			crossbuf->lengths = new std::vector<int16_t>();
			
			for (int i = 0; i < 4; ++i) {
				conffile >> rbuf;
				crossbuf->neighbours->push_back(rbuf);
			}
			
			for (int i = 0; i < 4; ++i) {
				conffile >> rbuf;
				crossbuf->lengths->push_back(rbuf);
			}
			
			crossings.push_back(crossbuf);

			conffile >> rbuf;
		}

		conffile.close();

		return true;
	}

	static Map& getInstance() {
		static Map instance;
		return instance;
	}

	bool getCrossings(anro_msgs::map_config::Response &res) {
		for (auto it = crossings.cbegin(); it != crossings.cend(); ++it) {
			anro_msgs::map_cross_msg msg;
			msg.ID = (**it).ID;
			std::vector<int16_t>* neighbours = (**it).neighbours;
			for (auto it = neighbours->cbegin(); it!= neighbours->cend(); ++it) {
				msg.neighbours.push_back(*it);
			}

			std::vector<int16_t>* lengths = (**it).lengths;
			for (auto it = lengths->cbegin(); it!= lengths->cend(); ++it) {
				msg.lengths.push_back(*it);
			}

			res.crossings.push_back(msg);
		}

		return true;
	}

	bool initCrossing(anro_msgs::cross_init::Response &res) {

		if (lastCrossingID >= crossings.size()) {
			res.crossing.ID = 0; 	
			return false;
		} // nie istniejesz

		Map::getInstance().lastCrossingID += 1;
		res.crossing.ID = lastCrossingID;
		res.crossing.neighbours = *(crossings[lastCrossingID-1]->neighbours);
		res.crossing.lengths = *(crossings[lastCrossingID-1]->lengths);

		return true;
	}
	bool initCar(anro_msgs::car_init::Response &res) {
		int crossA=std::rand()%crossings.size();
		std::vector<int16_t> avCross;
		int j=0;
		for(int i=0; i<4;++i) {
			if(crossings[crossA]->neighbours->at(i)!=0){
				avCross.push_back(i);
			}
		}

		int crossB = crossings[crossA]->neighbours->at( avCross[std::rand()%avCross.size()] );
		res.carID = ++lastCarID;
		res.prevCrossing = crossings[crossA]->ID;
		res.nextCrossing = crossB;
		res.pathLenght = 5;
	}

	int16_t getNumberOfCrossings() {
		return (int16_t)crossings.size();
	}

	static bool configService(anro_msgs::map_config::Request  &req,
         				  anro_msgs::map_config::Response &res) {

		Map& map = Map::getInstance();

		map.getCrossings(res);
		res.number_of_crossings = map.getNumberOfCrossings();
	
		ROS_INFO("Map server received config request %d", req.req);
		return true;
	}

	static bool crossingInitService(anro_msgs::cross_init::Request  &req,
	                         anro_msgs::cross_init::Response &res) {	
	
		if (!Map::getInstance().initCrossing(res)) 
			ROS_INFO("Map server received bad request");
		else
			ROS_INFO("Map server received new init request");
		
		return true;
	}

	static bool carInitService(anro_msgs::car_init::Request  &req,
         				  anro_msgs::car_init::Response &res) {

		Map& map = Map::getInstance();

		map.initCar(res);
	
		ROS_INFO("Map server received car init request %d", req.req);
		return true;
	}
};

int main(int argc, char **argv)
{
    std::srand( std::time( NULL ) );

	ros::init(argc, argv, "map_node");
	ros::NodeHandle n;
	
	std::string confname ;
	n.getParam("map_confname", confname);

	if (!Map::getInstance().configureFromFile(confname)) { ROS_INFO("Unable to read conf file"); return 1; }

	ros::ServiceServer configService    = n.advertiseService("get_map_config", Map::configService);
	ros::ServiceServer carFactory = n.advertiseService("init_car" , Map::carInitService);
	ros::ServiceServer crossFactory  = n.advertiseService("init_crossing" , Map::crossingInitService);
	ROS_INFO("Map ready");
	ros::spin();

	return 0;
}
