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
#include <ctype.h>

#include <string>
#include <sstream>


int16_t lightBulbState[100][4][4];

int16_t multiplier = 5;

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

void updateLightState(const lights::LightState::ConstPtr &msg, const std::string &topic){
	int16_t lastDigit = topic.size()-1;
	int16_t crossingID = 0;
	int16_t power = 1;
	for (int i = lastDigit; i>=0; i--)
	{
		char tmp;
		tmp = topic[i];
		if(isdigit(tmp)){
			int16_t itmp = tmp - '0';
			crossingID = crossingID + itmp*power;
			power = power*10;
		}
	}
	crossingID = crossingID-1;
	//ROS_INFO("%d", crossingID+1);
	//NORTH CROSSING
	if(msg->n.A) lightBulbState[crossingID][0][0] = 1;
	else lightBulbState[crossingID][0][0] = 0;
	
	if(msg->n.B) lightBulbState[crossingID][0][1] = 1;
	else lightBulbState[crossingID][0][1] = 0;
	
	if(msg->n.C) lightBulbState[crossingID][0][2] = 1;
	else lightBulbState[crossingID][0][2] = 0;
	
	if(msg->n.D) lightBulbState[crossingID][0][3] = 1;
	else lightBulbState[crossingID][0][3] = 0;
	
	//EAST CROSSING
	if(msg->e.A) lightBulbState[crossingID][1][0] = 1;
	else lightBulbState[crossingID][1][0] = 0;
	
	if(msg->e.B) lightBulbState[crossingID][1][1] = 1;
	else lightBulbState[crossingID][1][1] = 0;
	
	if(msg->e.C) lightBulbState[crossingID][1][2] = 1;
	else lightBulbState[crossingID][1][2] = 0;
	
	if(msg->e.D) lightBulbState[crossingID][1][3] = 1;
	else lightBulbState[crossingID][1][3] = 0;
	
	//SOUTH CROSSING
	if(msg->s.A) lightBulbState[crossingID][2][0] = 1;
	else lightBulbState[crossingID][2][0] = 0;
	
	if(msg->s.B) lightBulbState[crossingID][2][1] = 1;
	else lightBulbState[crossingID][2][1] = 0;
	
	if(msg->s.C) lightBulbState[crossingID][2][2] = 1;
	else lightBulbState[crossingID][2][2] = 0;
	
	if(msg->s.D) lightBulbState[crossingID][2][3] = 1;
	else lightBulbState[crossingID][2][3] = 0;
	
	//WEST CROSSING
	if(msg->w.A) lightBulbState[crossingID][3][0] = 1;
	else lightBulbState[crossingID][3][0] = 0;
	
	if(msg->w.B) lightBulbState[crossingID][3][1] = 1;
	else lightBulbState[crossingID][3][1] = 0;
	
	if(msg->w.C) lightBulbState[crossingID][3][2] = 1;
	else lightBulbState[crossingID][3][2] = 0;
	
	if(msg->w.D) lightBulbState[crossingID][3][3] = 1;
	else lightBulbState[crossingID][3][3] = 0;

	
	
}

class CarCollection{
private:
	CarCollection() {}
	CarCollection(const CarCollection&) {}

public:


	std::vector<Car> *lista = new std::vector<Car>();
	visualization_msgs::MarkerArray *markerArray = new visualization_msgs::MarkerArray;
	Coordinates * crossings;

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
			if(xcheck > 0) tmp->x = (crossings[tmp->startCrossID].x + distance*multiplier);
			else tmp->x = (crossings[tmp->startCrossID].x - distance *multiplier);
			tmp->y = (crossings[tmp->startCrossID].y) ;
		}
		else{
			if(crossings[tmp->endCrossID].y - crossings[tmp->startCrossID].y > 0)
				tmp->y = (crossings[tmp->startCrossID].y + distance*multiplier);
			else	tmp->y = (crossings[tmp->startCrossID].y - distance*multiplier);
			tmp->x = (crossings[tmp->startCrossID].x);
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
				if(xcheck > 0) currentCar->x = (crossings[currentCar->startCrossID].x + distance*multiplier);
				else currentCar->x = (crossings[currentCar->startCrossID].x - distance*multiplier);
				currentCar->y = (crossings[currentCar->startCrossID].y);
			}
			else{
				if(crossings[currentCar->endCrossID].y - crossings[currentCar->startCrossID].y > 0)
					currentCar->y = (crossings[currentCar->startCrossID].y + distance*multiplier);
				else	currentCar->y = (crossings[currentCar->startCrossID].y - distance*multiplier);
				currentCar->x = (crossings[currentCar->startCrossID].x);
			}
		}
	}
	
	static void addCar(const vizualization::auto_viz& msg)
	{
		Car *currentCar = CarCollection::getInstance().get(msg.autoID);
		if(CarCollection::getInstance().get(msg.autoID) != NULL){
			ROS_INFO("update");
			CarCollection::getInstance().updateCar(msg.autoID, msg.startCrossID, msg.endCrossID, msg.distance);
			ROS_INFO("x,y: %d , %d", currentCar->x, currentCar->y);
		}
		else{
			ROS_INFO("dodaje");
			CarCollection::getInstance().add(msg.autoID, msg.startCrossID, msg.endCrossID, msg.distance);
			currentCar = CarCollection::getInstance().get(msg.autoID);
			ROS_INFO("x,y: %d , %d", currentCar->x, currentCar->y);
		}

	}
};
int16_t numberOfCrossings;

int main(int argc, char **argv) 
{
	int16_t roadSize = 2;
	float lightOffset = 2.5;
	float lightDistance = 5;
	int16_t height = 5;
	
	ros::init(argc, argv, "vizualization_node");
	ros::NodeHandle n;

	ros::ServiceClient client = n.serviceClient<::map::map_config>("get_map_config");
	::map::map_config srv;

	uint8_t ReqID = 1;
	srv.request.req = (uint8_t) ReqID;
	Coordinates * crossings;
	Coordinates lights[100][4];
	int16_t connections[100][4];
	
	visualization_msgs::Marker* lightBulb[100][4][4];
	int16_t lightBulbData[100][4][4];
	

	if (client.call(srv)) {
    		numberOfCrossings = srv.response.crossings.size();
    		
    		for (int16_t crossingID = 0; crossingID < 100; crossingID++)
		{
			for (int16_t i = 0; i < 4; i++){
				for(int16_t j =0;j<4;j++)
				{
					lightBulbData[crossingID][i][j]=0;
				}
			}
		}
    		
    		
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
				if(connections[crossingID][i] >= 0){
				
					for(int16_t j =0;j<4;j++)
					{
						if(connections[crossingID][j]>=0)
						{
							lightBulbData[crossingID][i][j]=1;
						}
						
					}
				}
			}
		}


	} else {
		ROS_ERROR("Failed to call map for config");
		return 1;
	}
	
	//Topic do którego wysylane sa samochody
	
	CarCollection::getInstance().crossings = crossings;
	
	ros::Subscriber sub = n.subscribe("auto_viz", 1000, CarCollection::addCar);
	
	ros::Subscriber *lightSubscribers = new ros::Subscriber[numberOfCrossings];

	
	//Tworzenie subscriberArray - naSwiatla
	for(int i =0; i<numberOfCrossings; i++){
		stringstream ss;
		ss<<(i+1);
		string topicName="lights_";
		string number=ss.str();
		topicName=topicName+number; 
		lightSubscribers[i] = n.subscribe<lights::LightState>(topicName, 1, boost::bind(updateLightState, _1, topicName));
	}
	
	ros::Rate loop_rate(1);
	//WIZUALIZACJA
	
	visualization_msgs::MarkerArray markerArrayC;
	ros::Publisher crossing_pub = n.advertise<visualization_msgs::MarkerArray>("crossing_marker_array", 100);
	
	visualization_msgs::MarkerArray lightsMarkerArray;
	ros::Publisher lights_pub = n.advertise<visualization_msgs::MarkerArray>("lights_marker_array", 100);
	
	visualization_msgs::MarkerArray roadMarkerArray;
	ros::Publisher road_pub = n.advertise<visualization_msgs::MarkerArray>("road_marker_array", 100);
	

	
	//Wizualizacja statyczna
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
	    		//int16_t height = 5;
	    
			markerS->header.frame_id = "/base_link";
			markerS->header.stamp = ros::Time::now();

			markerS->ns = "basic_shapes";
			markerS->id = crossingID*10+i;

			markerS->type = visualization_msgs::Marker::CUBE;

			markerS->action = visualization_msgs::Marker::ADD;

			markerS->pose.position.x = lights[crossingID][i].x;
			markerS->pose.position.y = lights[crossingID][i].y;

			markerS->pose.position.z = height/2;
			markerS->pose.orientation.x = 0.0;
			markerS->pose.orientation.y = 0.0;
			markerS->pose.orientation.z = 0.0;
			markerS->pose.orientation.w = 1.0;

			markerS->scale.x = 0.3;
			markerS->scale.y = 0.3;
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
		    		
				markerS->header.frame_id = "/base_link";
				markerS->header.stamp = ros::Time::now();

				markerS->ns = "basic_shapes";
				markerS->id = crossingID*100+i;

				markerS->type = visualization_msgs::Marker::CUBE;

				markerS->action = visualization_msgs::Marker::ADD;

				markerS->pose.position.x = center.x;
				markerS->pose.position.y = center.y;

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

	while(ros::ok()){
		crossing_pub.publish(markerArrayC);
		lights_pub.publish(lightsMarkerArray);
		road_pub.publish(roadMarkerArray);
		
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
		
		
		//****************************************
		//tworzenie markerarrayZarowekSwietlnych
		//****************************************
		visualization_msgs::MarkerArray *lightBulbMarkerArray = new visualization_msgs::MarkerArray;
		ros::Publisher light_bulb_pub = n.advertise<visualization_msgs::MarkerArray>("light_bulb_marker_array", 100);
		
		for (int16_t crossingID = 0; crossingID < numberOfCrossings; crossingID++)
		{
			for (int16_t i = 0; i < 4; i++)
			{
				for(int16_t j = 0; j<4; j++)
			    	{
			    		if(lightBulbData[crossingID][i][j]==1)
			    		{
			    			
						visualization_msgs::Marker *markerS = new visualization_msgs::Marker;
						markerS->header.frame_id = "/base_link";
						markerS->header.stamp = ros::Time::now();

						markerS->ns = "basic_shapes";
						markerS->id = crossingID*1000 + 100*i + 10*j;

						markerS->type = visualization_msgs::Marker::ARROW;

						markerS->action = visualization_msgs::Marker::ADD;
							    			
						markerS->pose.position.x = lights[crossingID][i].x;
						markerS->pose.position.y = lights[crossingID][i].y;
						markerS->pose.position.z = height;

						markerS->scale.x = 1;
						markerS->scale.y = 1;
						markerS->scale.z = 1;

						
						markerS->color.b = 0.0f;
						markerS->color.a = 1.0;
						
						if(lightBulbState[crossingID][i][j] == 0){
						
							markerS->color.r = 1.0f;
							markerS->color.g = 0.0f;
						}
						
						if(lightBulbState[crossingID][i][j] == 1){
							markerS->color.r = 0.0f;
							markerS->color.g = 1.0f;
						}
						

						markerS->pose.orientation.x = 0.0;
						markerS->pose.orientation.y = 0.0;
						markerS->pose.orientation.z = 0.0;
						markerS->pose.orientation.w = 1.0;

						markerS->lifetime = ros::Duration();

						//NORTH Light
						if(i==0){
							if(j==0){
								//dol
								markerS->pose.orientation.y = 1.0;
							}
							if(j==1){
								//wschod
								markerS->pose.orientation.z = 0.0;
							}
							if(j==2){
								//gora
								markerS->pose.orientation.y = -1.0;
							}
							if(j==3){
								//zachod
								markerS->pose.orientation.z = 1000.0;
							}
						}

						//EAST Light
						if(i==1){
							if(j==0){
								markerS->pose.orientation.z = 1.0;
							}
							if(j==1){
								markerS->pose.orientation.y = 1.0;
							}
							if(j==2){
								markerS->pose.orientation.z = -1.0;
							}
							if(j==3){
								markerS->pose.orientation.y = -1.0;
							}
						}
						//SOUTH Light
						if(i==2){
							if(j==0){
								markerS->pose.orientation.y = -1.0;
							}
							if(j==1){
								markerS->pose.orientation.z = 0.0;
							}
							if(j==2){
								markerS->pose.orientation.y = 1.0;
							}
							if(j==3){
								markerS->pose.orientation.z = -1000.0;
							}
						}

						//SOUTH Light
						if(i==3){
							if(j==0){
								markerS->pose.orientation.z = 1.0;
							}
							if(j==1){
								markerS->pose.orientation.y = -1.0;
							}
							if(j==2){
								markerS->pose.orientation.z = -1.0;
							}
							if(j==3){
								markerS->pose.orientation.y = 1.0;
							}
						}

						lightBulb[crossingID][i][j] = markerS;
						lightBulbMarkerArray->markers.push_back(*markerS);
			    		}
			    	}
			}	
		}
		
		light_bulb_pub.publish(*lightBulbMarkerArray);
		
		for (int16_t crossingID = 0; crossingID < numberOfCrossings; crossingID++)
		{
			for (int16_t i = 0; i < 4; i++)
			{
				for(int16_t j = 0; j<4; j++)
			    	{
			    		if(lightBulbData[crossingID][i][j]==1) delete lightBulb[crossingID][i][j];
			    	}
			}
		}
		delete lightBulbMarkerArray;

		ros::spinOnce();
		
	}

    return 0;
}
