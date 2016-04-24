using namespace std;

#include "ros/ros.h"
#include "lights/State.h"
#include "lights/LightState.h"
#include "lights/map_config.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <iostream>
#include <vector>
#include <ctime>

#include <string>

class DirectionAllower{
	private:
		bool directionMatrix [4][4];
	public:
		bool shift();
		bool shiftRight(int i);
		bool shiftLeft (int i);
		vector <bool> takeDirections(int direction);
		DirectionAllower();
		~DirectionAllower();
		void show();
};

DirectionAllower::DirectionAllower(){
	bool tab [4][4] = { {1,0,0,0},
						{1,0,0,0},
						{0,0,1,0},
						{0,0,1,0},};
	for(int i = 0; i < 4; i++)
		for(int j = 0; j < 4; j++)
			directionMatrix[i][j] = tab[i][j];

}

DirectionAllower::~DirectionAllower(){

}

bool DirectionAllower::shiftRight(int i){
	int temp;
	temp = directionMatrix[i][3];
	for(int j = 3; j > 0; j--)
		directionMatrix[i][j] = directionMatrix[i][j-1];
	directionMatrix[i][0] = temp;
	return true;
}

bool DirectionAllower::shiftLeft(int i){
	int temp;
	temp = directionMatrix[i][0];
	for(int j = 0; j < 3; j++)
		directionMatrix[i][j] = directionMatrix[i][j+1];
	directionMatrix[i][3] = temp;
	return true;
}

bool DirectionAllower::shift(){
	this->shiftLeft(0);
	this->shiftRight(1);
	this->shiftLeft(2);
	this->shiftRight(3);
	return true;
}

vector <bool> DirectionAllower::takeDirections(int direction){
	vector <bool> column;
	for(int i = 0; i < 4; i++)
		column.push_back(directionMatrix[i][direction]);
	return column;
}

void DirectionAllower::show(){
	for(int i = 0; i < 4; i++){
		for(int j = 0; j < 4; j++)
			cout<<directionMatrix[i][j]<<' ';
		cout<<'\n';
	}
	cout<<'\n';
}

lights::map_config getMapConfiguration(){						//TYM POBIERAMY KONFIGURACJĘ MAPY

    lights::map_config mapConfiguration;
    ros::NodeHandle mapNode;

    ros::ServiceClient lightsClient = mapNode.serviceClient<lights::map_config>("get_map_config");
    mapConfiguration.request.req = 1;
    while(1){
	if(lightsClient.call(mapConfiguration)){
		break;
	}
	else{
		ROS_INFO("Failed to call service get_map_config");
	}
    }

    return mapConfiguration;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lights");

    lights::map_config map = getMapConfiguration();					//TUTAJ POBIERAMY KONFIGURACJĘ MAPY

    int nNodes=map.response.number_of_crossings;

    DirectionAllower *allower [nNodes];
    lights::LightState lightStates [nNodes];

	for(int x = 0; x < nNodes; x++){
            allower[x] = new DirectionAllower();
            for(int y = x%4; y > 0; y--)
                allower[x]->shift();
        }
    lights::State states [4];

    ros::NodeHandle lightsNodes[nNodes];
    ros::Publisher lightsPubs[nNodes];

    for(int i =0;i<nNodes;++i){
	stringstream ss;
	ss<<map.response.crossings[i].ID;
	string topicName="lights_";
	string number=ss.str();
	topicName=topicName+number; 
        lightsPubs[i] = lightsNodes[i].advertise<lights::LightState>(topicName, 1000);
    }

    while(ros::ok()){
			for(int x = 0; x < nNodes; x++){

				allower[x]->shift();

				for(int i  = 0; i < 4; i++){

					vector <bool> directions = allower[x]->takeDirections(i);
					states[i].A = directions[0];
					if(map.response.crossings[x].neighbours[0]<=0)states[i].A=false;
					states[i].B = directions[1];
					if(map.response.crossings[x].neighbours[1]<=0)states[i].B=false;
					states[i].C = directions[2];
					if(map.response.crossings[x].neighbours[2]<=0)states[i].C=false;
					states[i].D = directions[3];
					if(map.response.crossings[x].neighbours[3]<=0)states[i].D=false;
				}
				lightStates[x].n = states[0];
				lightStates[x].e = states[1];
				lightStates[x].s = states[2];
				lightStates[x].w = states[3];
				lightsPubs[x].publish(lightStates[x]);
			}
			ROS_INFO("Publishing finished");
			time_t czas;
			time(&czas);
			int czas1 = czas;
			while(time(&czas)-czas1<5);
    }
	return 0;
}
