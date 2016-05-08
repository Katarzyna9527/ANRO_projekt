using namespace std;

#include "ros/ros.h"
#include "lights/State.h"
#include "lights/LightState.h"
#include "map/map_config.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "vizualization/auto_viz.h"


#include <stdio.h>
#include <iostream>
#include <vector>
#include <ctime>

#include <string>

struct Coordinates{
	float x;
	float y;
};

struct Car{
	int16_t ID;
	int16_t startCrossID;
	int16_t endCrossID;		
	int32_t distance;
	int32_t x;
	int32_t y;
};

struct Lights{
	bool direction[4];
};

class CarCollection{
private:
	CarCollection() {}
	CarCollection(const CarCollection&) {}

public:


	std::vector<Car> *lista = new std::vector<Car>();
	visualization_msgs::MarkerArray *markerArray = new visualization_msgs::MarkerArray;
	Coordinates * crossings;
	//std::vector<Car>::iterator it;
	Car* get(int16_t checkID){
		for(auto it = lista->begin(); it != lista->end(); ++it){
			if((*it).ID == checkID) return &(*it);
		}
		return NULL;
	}
	
	static CarCollection& getInstance() {
		static CarCollection instance;
		return instance;
	}
	
	void add(int16_t ID, int16_t startCrossID, int16_t endCrossID, int32_t distance){
		Car *tmp = new Car;
		tmp->ID = ID;
		tmp->startCrossID = startCrossID-1;
		tmp->endCrossID = endCrossID-1;		
		tmp->distance = distance;
		int32_t xcheck = crossings[tmp->endCrossID].x - crossings[tmp->startCrossID].x;
		if(xcheck != 0){
			if(xcheck > 0) tmp->x = crossings[tmp->startCrossID].x + distance;
			else tmp->x = crossings[tmp->startCrossID].x - distance;
			tmp->y = crossings[tmp->startCrossID].y;
		}
		else{
			if(crossings[tmp->endCrossID].y - crossings[tmp->startCrossID].y > 0)
				tmp->y = crossings[tmp->startCrossID].y + distance;
			else	tmp->y = crossings[tmp->startCrossID].y - distance;
			tmp->x = crossings[tmp->startCrossID].x;
		}
		
		
		lista->push_back(*tmp);
	}
	
	void updateCar(int16_t ID, int16_t startCrossID, int16_t endCrossID, int32_t distance){
		Car *currentCar = CarCollection::getInstance().get(ID);
		if(currentCar->ID == ID){
			ROS_INFO("update_car");
			currentCar->startCrossID = startCrossID-1;
			currentCar->endCrossID = endCrossID-1;		
			currentCar->distance = distance;
			int32_t xcheck = crossings[currentCar->endCrossID].x - crossings[currentCar->startCrossID].x;
			if(xcheck != 0){
				if(xcheck > 0) currentCar->x = crossings[currentCar->startCrossID].x + distance;
				else currentCar->x = crossings[currentCar->startCrossID].x - distance;
				currentCar->y = crossings[currentCar->startCrossID].y;
			}
			else{
				if(crossings[currentCar->endCrossID].y - crossings[currentCar->startCrossID].y > 0)
					currentCar->y = crossings[currentCar->startCrossID].y + distance;
				else	currentCar->y = crossings[currentCar->startCrossID].y - distance;
				currentCar->x = crossings[currentCar->startCrossID].x;
			}
		}
	}
	
	static void addCar(const vizualization::auto_viz& msg)
	{
		Car *currentCar = CarCollection::getInstance().get(msg.autoID);
		if(CarCollection::getInstance().get(msg.autoID) != NULL){
			ROS_INFO("update");
			CarCollection::getInstance().updateCar(msg.autoID, msg.startCrossID, msg.endCrossID, msg.distance);
			ROS_INFO("%d, %d , %d",msg.distance, currentCar->x, currentCar->y);
		}
		else{
			ROS_INFO("dodaje");
			CarCollection::getInstance().add(msg.autoID, msg.startCrossID, msg.endCrossID, msg.distance);
			/*visualization_msgs::Marker *marker = new visualization_msgs::Marker;
			marker->header.frame_id = "/base_link";
			marker->header.stamp = ros::Time::now();

			marker->ns = "basic_shapes";
			marker->id = msg.autoID;

			marker->type = visualization_msgs::Marker::CUBE;

			marker->action = visualization_msgs::Marker::ADD;
			
			int16_t direction;
			int16_t dist;*/
			/*
			if(crossings[msg.startCrossID-(int16_t)1].x - crossings[msg.endCrossID-(int16_t)1].x ==0){
				//dist= crossings[msg.startCrossID-(int16_t)1].y - crossings[msg.endCrossID-(int16_t)1].y;
				//marker->pose.position.y = crossings[msg.endCrossID-(int16_t)1].y + dist + msg.distance;
				marker->pose.position.y = 0;
			}
			else {
				//dist= crossings[msg.startCrossID-1].x - crossings[msg.endCrossID-(int16_t)1].x;
				//marker->pose.position.x = crossings[msg.endCrossID-(int16_t)1].x + dist + msg.distance;
				marker->pose.position.x = 0;
			}*/
			/*marker->pose.position.y = msg.autoID;
			marker->pose.position.x = 3;
			marker->pose.position.z = 0;
			
			marker->pose.orientation.x = 0.0;
			marker->pose.orientation.y = 0.0;
			marker->pose.orientation.z = 0.0;
			marker->pose.orientation.w = 1.0;

			marker->scale.x = 1.0;
			marker->scale.y = 1.0;
			marker->scale.z = 1.0;

			marker->color.r = 0.0f;
			marker->color.g = 1.0f;
			marker->color.b = 0.0f;
			marker->color.a = 1.0;

			marker->lifetime = ros::Duration();
			CarCollection::getInstance().markerArray->markers.push_back(*marker);*/
		}
		//ROS_INFO("%f", CarCollection::getInstance().markerArray->markers[0].pose.position.y);
	  	ROS_INFO("recieved data %d", msg.autoID);
	}
};

int main(int argc, char **argv) 
{
	int16_t roadSize = 2;
	float lightOffset = 2.5;
	float lightDistance = 5;
	
	ros::init(argc, argv, "vizualization_node");
	ros::NodeHandle n;

	ros::ServiceClient client = n.serviceClient<::map::map_config>("get_map_config");
	::map::map_config srv;

	uint8_t ReqID = 1;
	srv.request.req = (uint8_t) ReqID;
	Coordinates * crossings;
	Coordinates lights[100][4];
	int connections[100][4];
	
	visualization_msgs::Marker* lightBulb[100][4][4];

    	if (client.call(srv)) {
    		int16_t multiplier = 5;
		crossings = new Coordinates [srv.response.crossings.size()];
		crossings[0].x = 0;
		crossings[0].y = 0;
		
		//Ustalenie wspolrzednych wszystkich skrzyzowan
		for (int16_t crossingID = 0; crossingID < srv.response.crossings.size(); crossingID++){
			//if(crossingID!=0 && crossings[crossingID].x == 0 && crossings[crossingID].y == 0) continue;
			ROS_INFO("Skrzyzowanie: %d", ((int16_t)crossingID+1));
			
			//NORTH
			int16_t neighbour=srv.response.crossings[crossingID].neighbours[0];
			int16_t length = srv.response.crossings[crossingID].lengths[0];
			if(neighbour != 0){
				crossings[neighbour-1].y = crossings[crossingID].y + length * multiplier;
				crossings[neighbour-1].x = crossings[crossingID].x;
				ROS_INFO("north: %d: %d %d", (int16_t)neighbour, (int16_t)crossings[neighbour-1].x, (int16_t)crossings[neighbour-1].y);
				lights[crossingID][0].x = crossings[crossingID].x - lightOffset;
				lights[crossingID][0].y = crossings[crossingID].y + lightDistance;
				connections[crossingID][0] = neighbour-1;
				//ROS_INFO("north: %d: %d %d", (int16_t)crossingID, (int16_t)lights[crossingID][0.x, (int16_t)crossings[neighbour-1].y);
			}
			else{
				lights[crossingID][0].x = 0;
				lights[crossingID][0].y = 0;
				connections[crossingID][0] = -1;
			}
			
			//EAST
			neighbour=srv.response.crossings[crossingID].neighbours[1];
			length = srv.response.crossings[crossingID].lengths[1];
			if(neighbour != 0){
				crossings[neighbour-1].x = crossings[crossingID].x + length * multiplier;
				crossings[neighbour-1].y = crossings[crossingID].y;
				ROS_INFO("east: %d: %d %d", (int16_t)neighbour, (int16_t)crossings[neighbour-1].x, (int16_t)crossings[neighbour-1].y);
				lights[crossingID][1].x = crossings[crossingID].x + lightDistance;
				lights[crossingID][1].y = crossings[crossingID].y + lightOffset;
				connections[crossingID][1] = neighbour-1;
			}
			else{
				lights[crossingID][1].x = 0;
				lights[crossingID][1].y = 0;
				connections[crossingID][1] = -1;
			}
			
			
			//SOUTH
			neighbour=srv.response.crossings[crossingID].neighbours[2];
			length = srv.response.crossings[crossingID].lengths[2];
			if(neighbour != 0){
				crossings[neighbour-1].y = crossings[crossingID].y - length * multiplier;
				crossings[neighbour-1].x = crossings[crossingID].x;
				ROS_INFO("south: %d: %d %d", (int16_t)neighbour, (int16_t)crossings[neighbour-1].x, (int16_t)crossings[neighbour-1].y);
				lights[crossingID][2].x = crossings[crossingID].x + lightOffset;
				lights[crossingID][2].y = crossings[crossingID].y - lightDistance;
				connections[crossingID][2] = neighbour-1;
			}
			else{
				lights[crossingID][2].x = 0;
				lights[crossingID][2].y = 0;
				connections[crossingID][2] = -1;
			}
			
			//WEST
			neighbour=srv.response.crossings[crossingID].neighbours[3];
			length = srv.response.crossings[crossingID].lengths[3];
			if(neighbour != 0){
				crossings[neighbour-1].x = crossings[crossingID].x - length * multiplier;
				crossings[neighbour-1].y = crossings[crossingID].y;
				ROS_INFO("west: %d: %d %d", (int16_t)neighbour, (int16_t)crossings[neighbour-1].x, (int16_t)crossings[neighbour-1].y);
				lights[crossingID][3].x = crossings[crossingID].x - lightDistance;
				lights[crossingID][3].y = crossings[crossingID].y - lightOffset;
				connections[crossingID][3] = neighbour-1;
			}
			else{
				lights[crossingID][3].x = 0;
				lights[crossingID][3].y = 0;
				connections[crossingID][3] = -1;
			}
			
		}
		for (int16_t crossingID = 0; crossingID < srv.response.crossings.size(); crossingID++)
		{
			ROS_INFO("%d: %d %d", (int16_t)crossingID+1, (int16_t)crossings[crossingID].x, (int16_t)crossings[crossingID].y);
		}
		
		for (int16_t crossingID = 0; crossingID < srv.response.crossings.size(); crossingID++){
			for (int16_t i = 0; i < 4; i++){
			
				ROS_INFO("%d: %d %d", (int16_t)crossingID+1, (int16_t)lights[crossingID][i].x, (int16_t)lights[crossingID][i].y);
			}
		}


	} else {
		ROS_ERROR("Failed to call map for config");
		return 1;
	}
	
	//Topic do którego wysylane sa samochody
	
	CarCollection::getInstance().crossings = crossings;
	
	ros::Subscriber sub = n.subscribe("viz_auto", 1000, CarCollection::addCar);
	
	ros::Rate loop_rate(1);
	//WIZUALIZACJA
	
	visualization_msgs::MarkerArray markerArrayC;
	ros::Publisher crossing_pub = n.advertise<visualization_msgs::MarkerArray>("crossing_marker_array", 100);
	
	visualization_msgs::MarkerArray lightsMarkerArray;
	ros::Publisher lights_pub = n.advertise<visualization_msgs::MarkerArray>("lights_marker_array", 100);
	
	visualization_msgs::MarkerArray roadMarkerArray;
	ros::Publisher road_pub = n.advertise<visualization_msgs::MarkerArray>("road_marker_array", 100);
	
	//Wizualizacja skrzyżowań
	for (int16_t crossingID = 0; crossingID < srv.response.crossings.size(); crossingID++){
		//Skrzyzowanie
	    visualization_msgs::Marker *marker = new visualization_msgs::Marker;
	    
	    marker->header.frame_id = "/base_link";
	    marker->header.stamp = ros::Time::now();

	    marker->ns = "basic_shapes";
	    marker->id = crossingID;

	    marker->type = visualization_msgs::Marker::CUBE;

	    marker->action = visualization_msgs::Marker::ADD;

	    marker->pose.position.x = crossings[crossingID].x;
	    marker->pose.position.y = crossings[crossingID].y;
	    //marker->pose.position.x = abs(crossings[crossingID].y);
	    //marker->pose.position.y = abs(crossings[crossingID].x);
	    marker->pose.position.z = 0;
	    marker->pose.orientation.x = 0.0;
	    marker->pose.orientation.y = 0.0;
	    marker->pose.orientation.z = 0.0;
	    marker->pose.orientation.w = 1.0;

	    marker->scale.x = 2 * roadSize;
	    marker->scale.y = 2 * roadSize;
	    marker->scale.z = 1.0;

	    marker->color.r = 0.0f;
	    marker->color.g = 0.0f;
	    marker->color.b = 1.0f;
	    marker->color.a = 1.0;

	    marker->lifetime = ros::Duration();
	    markerArrayC.markers.push_back(*marker);
	    
	    //swiatla
	    for (int16_t i = 0; i < 4; i++){
	    	visualization_msgs::Marker *markerS = new visualization_msgs::Marker;
	    	if(lights[crossingID][i].x !=0 && lights[crossingID][i].y !=0)
	    	{
	    		int16_t height = 5;
	    
			markerS->header.frame_id = "/base_link";
			markerS->header.stamp = ros::Time::now();

			markerS->ns = "basic_shapes";
			markerS->id = crossingID*10+i;

			markerS->type = visualization_msgs::Marker::CUBE;

			markerS->action = visualization_msgs::Marker::ADD;

			markerS->pose.position.x = lights[crossingID][i].x;
			markerS->pose.position.y = lights[crossingID][i].y;
			//markerS->pose.position.x = abs(lights[crossingID][i].y);
			//markerS->pose.position.y = abs(lights[crossingID][i].x);
			markerS->pose.position.z = height/2;
			markerS->pose.orientation.x = 0.0;
			markerS->pose.orientation.y = 0.0;
			markerS->pose.orientation.z = 0.0;
			markerS->pose.orientation.w = 1.0;

			markerS->scale.x = 0.5;
			markerS->scale.y = 0.5;
			markerS->scale.z = height;

			markerS->color.r = 0.0f;
			markerS->color.g = 1.0f;
			markerS->color.b = 1.0f;
			markerS->color.a = 1.0;

			markerS->lifetime = ros::Duration();
			lightsMarkerArray.markers.push_back(*markerS);
	    	}
	    }
	    
	    //drogi
	    for (int16_t i = 0; i < 4; i++){
	    	visualization_msgs::Marker *markerS = new visualization_msgs::Marker;
	    	if(connections[crossingID][i] >= 0)
	    	{
	    		Coordinates center;
	    		center.x = crossings[crossingID].x - 0.5 * (crossings[crossingID].x - crossings[connections[crossingID][i]].x);
	    		center.y = crossings[crossingID].y - 0.5 * (crossings[crossingID].y - crossings[connections[crossingID][i]].y);
	    		
	    		if(i == 0 ||i==2){
	    		
	    		}	
	    		
			markerS->header.frame_id = "/base_link";
			markerS->header.stamp = ros::Time::now();

			markerS->ns = "basic_shapes";
			markerS->id = crossingID*100+i;

			markerS->type = visualization_msgs::Marker::CUBE;

			markerS->action = visualization_msgs::Marker::ADD;

			markerS->pose.position.x = center.x;
			markerS->pose.position.y = center.y;
			//markerS->pose.position.x = abs(lights[crossingID][i].y);
			//markerS->pose.position.y = abs(lights[crossingID][i].x);
			markerS->pose.position.z = 0;
			markerS->pose.orientation.x = 0.0;
			markerS->pose.orientation.y = 0.0;
			markerS->pose.orientation.z = 0.0;
			markerS->pose.orientation.w = 1.0;

			if(i == 0 || i==2){
	    			markerS->scale.x = 2*roadSize;
				markerS->scale.y = abs(crossings[crossingID].y - crossings[connections[crossingID][i]].y) - 2*roadSize;
	    		}
	    		else{
	    			markerS->scale.x = abs(crossings[crossingID].x - crossings[connections[crossingID][i]].x) - 2*roadSize;
				markerS->scale.y = 2*roadSize;
	    		}
			markerS->scale.z = 0.5;

			markerS->color.r = 1.0f;
			markerS->color.g = 1.0f;
			markerS->color.b = 0.0f;
			markerS->color.a = 1.0;

			markerS->lifetime = ros::Duration();
			roadMarkerArray.markers.push_back(*markerS);
	    	}
	    }
		
	}
	
	
	
	
	ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);
	//ROS_INFO("%f", CarCollection::getInstance().markerArray.markers[0].pose.position.x);
	while(ros::ok()){
		crossing_pub.publish(markerArrayC);
		lights_pub.publish(lightsMarkerArray);
		road_pub.publish(roadMarkerArray);
		//ros::Subscriber sub = n.subscribe("viz_auto", 1000, CarCollection::addCar);
		
		//ros::Publisher car_pub = n.advertise<visualization_msgs::Marker>("car_marker", 100);
		
		//********************************
		//tworzenie marker array
		//********************************
		visualization_msgs::MarkerArray *markerArray = new visualization_msgs::MarkerArray;
		int16_t ilosc = 0;
		visualization_msgs::Marker *markersToDelete[100];
		int16_t i = 0;
		for(auto it = CarCollection::getInstance().lista->cbegin(); it != CarCollection::getInstance().lista->cend(); ++it)
		{
			visualization_msgs::Marker *marker = new visualization_msgs::Marker;
			marker->header.frame_id = "/base_link";
			marker->header.stamp = ros::Time::now();

			marker->ns = "basic_shapes";
			marker->id = (*it).ID;

			marker->type = visualization_msgs::Marker::CUBE;

			marker->action = visualization_msgs::Marker::ADD;
			
			int16_t direction;
			int16_t dist;

			marker->pose.position.y = (*it).y;
			marker->pose.position.x = (*it).x;
			//marker->pose.position.y = abs((*it).x);
			//marker->pose.position.x = abs((*it).y);
			marker->pose.position.z = 0;
			
			marker->pose.orientation.x = 0.0;
			marker->pose.orientation.y = 0.0;
			marker->pose.orientation.z = 0.0;
			marker->pose.orientation.w = 1.0;

			marker->scale.x = 1.0;
			marker->scale.y = 1.0;
			marker->scale.z = 1.0;

			marker->color.r = 1.0f;
			marker->color.g = 0.0f;
			marker->color.b = 0.0f;
			marker->color.a = 1.0;

			marker->lifetime = ros::Duration();
			markerArray->markers.push_back(*marker);
			markersToDelete[i] = marker;
			i++;
			ilosc++;
		}
		
		//Publikowanie
		if(!CarCollection::getInstance().lista->empty()){
			marker_pub.publish(*markerArray);
		}
		
		for(i = 0; i<ilosc; i++){
			delete markersToDelete[i];
		} 
		
		delete markerArray;
		
		ros::spinOnce();
		//loop_rate.sleep();
		
	}

    return 0;
}
