using namespace std;

#include "ros/ros.h"
#include "anro_msgs/State.h"
#include "anro_msgs/LightState.h"
#include "anro_msgs/map_config.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "anro_msgs/vizualization_auto_viz.h"


#include <stdio.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <ctype.h>

#include <string>
#include <sstream>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


int16_t lightBulbState[100][4][4];

int16_t multiplier = 5;

float roadSize = 2;
float lightOffset = 0; //2.5
float lightDistance = 5;
int16_t height = 5;
float lightBulbOffset = 0.3;
float lightBulbLenght = 2;
int16_t lightMode = 3; //1 dla widocznych z samochodu, 2 dla strzalek widocznych z góry, 3 dla kulturalnych

struct Coordinates{
	float x;
	float y;
};

struct Car{
	int16_t ID;
	int16_t startCrossID;
	int16_t endCrossID;		
	float distance;
	float x;
	float y;
	int32_t direction;
};

struct Lights{
	bool direction[4];
};


class CarCollection{
private:
	CarCollection() {}
	CarCollection(const CarCollection&) {}

public:

	int16_t numberOfCars= 0;
	std::vector<Car> *lista = new std::vector<Car>();
	visualization_msgs::MarkerArray *carMarkerArray = new visualization_msgs::MarkerArray;
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
	
	void addToMarkerArray(Car* it){
		float xOffset = 0;
		float yOffset = 0;
		visualization_msgs::Marker *marker = new visualization_msgs::Marker;
		marker->header.frame_id = "/base_link";
		marker->header.stamp = ros::Time::now();

		marker->ns = "car_markers";
		marker->id = (*it).ID;

		marker->type = visualization_msgs::Marker::CUBE;

		marker->action = visualization_msgs::Marker::ADD;
		
		ROS_INFO("%d", (*it).direction);
		if((*it).direction == 0) xOffset = roadSize/2;
		else if((*it).direction == 1) yOffset = -roadSize/2;
		else if((*it).direction == 2) xOffset = -roadSize/2;
		else if((*it).direction == 3) yOffset = roadSize/2;
		ROS_INFO("xoffset, yoffset: %f %f", xOffset, yOffset);
		
		marker->pose.position.y = (*it).y + yOffset;
		marker->pose.position.x = (*it).x + xOffset;

		marker->pose.position.z = 0.5;
	
		marker->scale.x = 1.0;
		marker->scale.y = 1.0;
		marker->scale.z = 1.0;
		tf::Quaternion quat;
		if((*it).direction == 0) marker->pose.orientation = tf::createQuaternionMsgFromYaw((3.14/2)*2);
		if((*it).direction == 1) marker->pose.orientation = tf::createQuaternionMsgFromYaw((3.14/2)*1);
		if((*it).direction == 2) marker->pose.orientation = tf::createQuaternionMsgFromYaw((3.14/2)*4);
		if((*it).direction == 3) marker->pose.orientation = tf::createQuaternionMsgFromYaw((3.14/2)*3);

		marker-> mesh_use_embedded_materials=true;

		marker->type  = visualization_msgs::Marker::MESH_RESOURCE;
		marker->mesh_resource = "package://vizualization/meshes/fake_car_1/fake_car_1.dae";

		marker->lifetime = ros::Duration();
		carMarkerArray->markers.push_back(*marker);
	}
	
	void updateMarkerArray(Car* it){
		for(int16_t i= 0; i<numberOfCars; i++){
			if(carMarkerArray->markers[i].id == it->ID){
				float xOffset = 0;
				float yOffset = 0;
				
				ROS_INFO("%d", (*it).direction);
				if((*it).direction == 0) xOffset = roadSize/2;
				else if((*it).direction == 1) yOffset = -roadSize/2;
				else if((*it).direction == 2) xOffset = -roadSize/2;
				else if((*it).direction == 3) yOffset = roadSize/2;
				ROS_INFO("xoffset, yoffset: %f %f", xOffset, yOffset);
				
				carMarkerArray->markers[i].pose.position.y = it->y + yOffset;
				carMarkerArray->markers[i].pose.position.x = it->x + xOffset;
				
				carMarkerArray->markers[i].scale.x = 1.0;
				carMarkerArray->markers[i].scale.y = 1.0;
				carMarkerArray->markers[i].scale.z = 1.0;

				if((*it).direction == 0) carMarkerArray->markers[i].pose.orientation = tf::createQuaternionMsgFromYaw((3.14/2)*2);
				if((*it).direction == 1) carMarkerArray->markers[i].pose.orientation = tf::createQuaternionMsgFromYaw((3.14/2)*1);
				if((*it).direction == 2) carMarkerArray->markers[i].pose.orientation = tf::createQuaternionMsgFromYaw((3.14/2)*4);
				if((*it).direction == 3) carMarkerArray->markers[i].pose.orientation = tf::createQuaternionMsgFromYaw((3.14/2)*3);

			}
		} 
	}
	
	void add(int16_t ID, int16_t startCrossID, int16_t endCrossID, float distance){
		Car *tmp = new Car;
		tmp->ID = ID;
		tmp->startCrossID = startCrossID-1;
		tmp->endCrossID = endCrossID-1;		
		tmp->distance = distance;
		int32_t xcheck = crossings[tmp->endCrossID].x - crossings[tmp->startCrossID].x;
		if(xcheck != 0){
			if(xcheck > 0) {
				tmp->x = (crossings[tmp->startCrossID].x + distance*multiplier);
				tmp->direction = 1;
			}
			else {
				tmp->x = (crossings[tmp->startCrossID].x - distance *multiplier);
				tmp->direction = 3;
			}
			tmp->y = (crossings[tmp->startCrossID].y) ;
		}
		else{
			if(crossings[tmp->endCrossID].y - crossings[tmp->startCrossID].y > 0){
				tmp->y = (crossings[tmp->startCrossID].y + distance*multiplier);
				tmp->direction = 0;
			}
			else{
				tmp->y = (crossings[tmp->startCrossID].y - distance*multiplier);
				tmp->direction = 2;
			}
			tmp->x = (crossings[tmp->startCrossID].x);
		}
		
		numberOfCars++;
		lista->push_back(*tmp);
		addToMarkerArray(tmp);
	}
	
	void updateCar(int16_t ID, int16_t startCrossID, int16_t endCrossID, float distance){
		Car *currentCar = CarCollection::getInstance().get(ID);
		if(currentCar->ID == ID){
			ROS_INFO("update_car");
			currentCar->startCrossID = startCrossID-1;
			currentCar->endCrossID = endCrossID-1;		
			currentCar->distance = distance;
			int32_t xcheck = crossings[currentCar->endCrossID].x - crossings[currentCar->startCrossID].x;
			if(xcheck != 0){
				if(xcheck > 0) {
					currentCar->x = (crossings[currentCar->startCrossID].x + distance*multiplier);
					currentCar->direction = 1;
				}
				else {
					currentCar->x = (crossings[currentCar->startCrossID].x - distance *multiplier);
					currentCar->direction = 3;
				}
				currentCar->y = (crossings[currentCar->startCrossID].y);
			}
			else{
				if(crossings[currentCar->endCrossID].y - crossings[currentCar->startCrossID].y > 0){
					currentCar->y = (crossings[currentCar->startCrossID].y + distance*multiplier);
					currentCar->direction = 0;
				}
				else{
					currentCar->y = (crossings[currentCar->startCrossID].y - distance*multiplier);
					currentCar->direction = 2;
				}
				currentCar->x = (crossings[currentCar->startCrossID].x);
			}
		}
		updateMarkerArray(currentCar);
	}
	
	static void addCar(const anro_msgs::vizualization_auto_viz& msg)
	{
		Car *currentCar = CarCollection::getInstance().get(msg.autoID);
		if(CarCollection::getInstance().get(msg.autoID) != NULL){
			ROS_INFO("update");
			CarCollection::getInstance().updateCar(msg.autoID, msg.startCrossID, msg.endCrossID, msg.distance);
			ROS_INFO("x,y: %f , %f", currentCar->x, currentCar->y);
		}
		else{
			ROS_INFO("dodaje");
			CarCollection::getInstance().add(msg.autoID, msg.startCrossID, msg.endCrossID, msg.distance);
			currentCar = CarCollection::getInstance().get(msg.autoID);
			ROS_INFO("x,y: %f , %f", currentCar->x, currentCar->y);
		}

	}
};

class LightBulbCollection{
private:
	LightBulbCollection() {}
	LightBulbCollection(const LightBulbCollection&) {}

public:
	
	int16_t numberOfLightBulbs = 0;
	
	visualization_msgs::MarkerArray *LightBulbMarkerArray = new visualization_msgs::MarkerArray;

	static LightBulbCollection& getInstance() {
		static LightBulbCollection instance;
		return instance;
	}
	
	visualization_msgs::Marker* getMarker(int16_t crossingID, int16_t i, int16_t j){
		for(int16_t k = 0; k<numberOfLightBulbs; k++){
			if(LightBulbMarkerArray->markers[k].id == crossingID*100 + 10*i + 1*j){
					return &(LightBulbMarkerArray->markers[k]);
				}
		}
		return NULL;
	}
	
	void activateMarker(int16_t crossingID, int16_t i, int16_t j){
		visualization_msgs::Marker* currentBulb = getMarker(crossingID, i, j);
		if(currentBulb == NULL) return;
		if(currentBulb->id == crossingID*100 + 10*i + 1*j){
			//ROS_INFO("Prawdziwy activate: %d", currentBulb->id);
			currentBulb->action = visualization_msgs::Marker::ADD;
			/*currentBulb->color.r = 0.0f;
			currentBulb->color.g = 1.0f;*/
		}
	}
	
	void deactivateMarker(int16_t crossingID, int16_t i, int16_t j){
		visualization_msgs::Marker* currentBulb = getMarker(crossingID, i, j);
		if(currentBulb == NULL) return;
		if(currentBulb->id == crossingID*100 + 10*i + 1*j){
			//ROS_INFO("Prawdziwy deactivate: %d", currentBulb->id);
			currentBulb->action = visualization_msgs::Marker::DELETE;
			/*currentBulb->color.r = 1.0f;
			currentBulb->color.g = 0.0f;*/
		}
	}
	
	void updateLightState(const anro_msgs::LightState::ConstPtr &msg, int16_t crossingID){
		//ROS_INFO("%d", crossingID+1);
		//ROS_INFO("%d", numberOfLightBulbs);
		//NORTH CROSSING
		if(msg->n.A) activateMarker(crossingID,0,0);
		else deactivateMarker(crossingID,0,0);
	
		if(msg->n.B) activateMarker(crossingID,0,1);
		else deactivateMarker(crossingID,0,1);
	
		if(msg->n.C) activateMarker(crossingID,0,2);
		else deactivateMarker(crossingID,0,2);
	
		if(msg->n.D) activateMarker(crossingID,0,3);
		else deactivateMarker(crossingID,0,3);
	
		//EAST CROSSING
		if(msg->e.A) activateMarker(crossingID,1,0);
		else deactivateMarker(crossingID,1,0);
	
		if(msg->e.B) activateMarker(crossingID,1,1);
		else deactivateMarker(crossingID,1,1);
	
		if(msg->e.C) activateMarker(crossingID,1,2);
		else deactivateMarker(crossingID,1,2);
	
		if(msg->e.D) activateMarker(crossingID,1,3);
		else deactivateMarker(crossingID,1,3);
	
		//SOUTH CROSSING
		if(msg->s.A) activateMarker(crossingID,2,0);
		else deactivateMarker(crossingID,2,0);
	
		if(msg->s.B) activateMarker(crossingID,2,1);
		else deactivateMarker(crossingID,2,1);
	
		if(msg->s.C) activateMarker(crossingID,2,2);
		else deactivateMarker(crossingID,2,2);
	
		if(msg->s.D) activateMarker(crossingID,2,3);
		else deactivateMarker(crossingID,2,3);
	
		//WEST CROSSING
		if(msg->w.A) activateMarker(crossingID,3,0);
		else deactivateMarker(crossingID,3,0);
	
		if(msg->w.B) activateMarker(crossingID,3,1);
		else deactivateMarker(crossingID,3,1);
	
		if(msg->w.C) activateMarker(crossingID,3,2);
		else deactivateMarker(crossingID,3,2);
	
		if(msg->w.D) activateMarker(crossingID,3,3);
		else deactivateMarker(crossingID,3,3);
	}
	
};


void updateLightState(const anro_msgs::LightState::ConstPtr &msg, const std::string &topic){
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
	LightBulbCollection::getInstance().updateLightState(msg, crossingID);
	//ROS_INFO("%d", crossingID+1);
}




int16_t numberOfCrossings;

int main(int argc, char **argv) 
{

	ros::init(argc, argv, "vizualization_node");
	ros::NodeHandle n;

	ros::ServiceClient client = n.serviceClient<::anro_msgs::map_config>("get_map_config");
	::anro_msgs::map_config srv;

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
		float lightMode2Offset = 0.5;	
		
		//Stworzenie lightBulbs
		for (int16_t crossingID = 0; crossingID < srv.response.crossings.size(); crossingID++){
			for (int16_t i = 0; i < 4; i++){
				if(connections[crossingID][i] >= 0){
					int16_t numberOfConnections = 0;
					
					//Sprawdzenie ile jest połączeń
					for(int16_t k =0;k<4;k++){
						if(connections[crossingID][k]>=0) numberOfConnections++;
					}
					
					for(int16_t j =0;j<4;j++)
					{
						
						if(connections[crossingID][j]>=0 && i!=j && numberOfConnections > 2)
						{
							visualization_msgs::Marker *markerS = new visualization_msgs::Marker;
							markerS->header.frame_id = "/base_link";
							markerS->header.stamp = ros::Time::now();

							markerS->ns = "light_bulb_shapes";
							markerS->id = crossingID*100 + 10*i + 1*j;
							if(lightMode == 1 || lightMode == 2 || lightMode == 3)
								markerS->type = visualization_msgs::Marker::ARROW;

							markerS->action = visualization_msgs::Marker::ADD;
							
							geometry_msgs::Point startPoint;
							geometry_msgs::Point endPoint;
							geometry_msgs::Point middlePoint;
							middlePoint.z = 0;
							
							if(lightMode == 1 || lightMode == 3){
								startPoint.x = lights[crossingID][i].x;
								startPoint.y = lights[crossingID][i].y;
								startPoint.z = height;
							
								endPoint.x = lights[crossingID][i].x;
								endPoint.y = lights[crossingID][i].y;
								endPoint.z = height;
							}
							if(lightMode == 2){
								startPoint.z = 0.0;
								endPoint.z = 0.0;
							}
							markerS->color.b = 0.0f;
							markerS->color.a = 1.0;
							markerS->color.r = 0.0f;
							markerS->color.g = 1.0f;
							
							markerS->lifetime = ros::Duration();

							//NORTH Light
							if(i==0){
								if(lightMode == 2){
									startPoint.x = crossings[crossingID].x - 0.5*roadSize;
									startPoint.y = crossings[crossingID].y + (1+lightMode2Offset)*roadSize;
								}
								if(j==0){
									if(lightMode == 1){
										endPoint.z += -(lightBulbLenght + lightBulbOffset);
										startPoint.z += - (lightBulbOffset);
									}
									if(lightMode == 2){
										endPoint.x = crossings[crossingID].x - roadSize;
										endPoint.y = crossings[crossingID].y + (1.5+lightMode2Offset)*roadSize;
									}
									if(lightMode == 3){
										endPoint.y += (lightBulbLenght + lightBulbOffset);
										startPoint.y += (lightBulbOffset);
									}
									
								}
								if(j==1){
									if(lightMode == 1){
										endPoint.x += lightBulbLenght + lightBulbOffset;
										startPoint.x += lightBulbOffset;
									}
									if(lightMode == 2){
										endPoint.x = crossings[crossingID].x + (1+lightMode2Offset)*roadSize;
										endPoint.y = crossings[crossingID].y -0.5*roadSize;
									}
									if(lightMode == 3){
										endPoint.x += lightBulbLenght + lightBulbOffset;
										startPoint.x += lightBulbOffset;
									}
								}
								if(j==2){
									if(lightMode == 1){
										endPoint.z += lightBulbLenght + lightBulbOffset;
										startPoint.z += lightBulbOffset;
									}
									if(lightMode == 2){
										endPoint.x = crossings[crossingID].x - 0.5*roadSize;
										endPoint.y = crossings[crossingID].y - (1+lightMode2Offset)*roadSize;
									}
									if(lightMode == 3){
										endPoint.y += -(lightBulbLenght + lightBulbOffset);
										startPoint.y += -(lightBulbOffset);
									}
								}
								if(j==3){
									if(lightMode == 1){
										endPoint.x += -(lightBulbLenght + lightBulbOffset);
										startPoint.x += -(lightBulbOffset);
									}
									if(lightMode == 2){
										endPoint.x = crossings[crossingID].x - (1+lightMode2Offset)*roadSize;
										endPoint.y = crossings[crossingID].y + 0.5*roadSize;
									}
									if(lightMode == 3){
										endPoint.x += -(lightBulbLenght + lightBulbOffset);
										startPoint.x += -(lightBulbOffset);
									}
									
								}
							}

							//EAST Light
							if(i==1){
								if(lightMode == 2){
									startPoint.x = crossings[crossingID].x + (1+lightMode2Offset)*roadSize;
									startPoint.y = crossings[crossingID].y + 0.5*roadSize;
								}
								if(j==0){
									if(lightMode == 1){
										endPoint.y += (lightBulbLenght + lightBulbOffset);
										startPoint.y += lightBulbOffset;
									}
									if(lightMode == 2){
										endPoint.x = crossings[crossingID].x + 0.5*roadSize;
										endPoint.y = crossings[crossingID].y + (1+lightMode2Offset)*roadSize;
									}
									if(lightMode == 3){
										endPoint.y += (lightBulbLenght + lightBulbOffset);
										startPoint.y += lightBulbOffset;
									}
								}
								if(j==1){
									if(lightMode == 1){
										endPoint.z += -(lightBulbLenght + lightBulbOffset);
										startPoint.z += -(lightBulbOffset);
									}
									if(lightMode == 2){
										endPoint.x = crossings[crossingID].x + (1.5+lightMode2Offset)*roadSize;
										endPoint.y = crossings[crossingID].y + 0.5*roadSize;
									}
									if(lightMode == 3){
										endPoint.x += (lightBulbLenght + lightBulbOffset);
										startPoint.x += (lightBulbOffset);
									}
								}
								if(j==2){
									if(lightMode == 1){
										endPoint.y += -(lightBulbLenght+lightBulbOffset);
										startPoint.y += -(lightBulbOffset);
									}
									if(lightMode == 2){
										endPoint.x = crossings[crossingID].x - 0.5*roadSize;
										endPoint.y = crossings[crossingID].y - (1+lightMode2Offset)*roadSize;
									}
									if(lightMode == 3){
										endPoint.y += -(lightBulbLenght+lightBulbOffset);
										startPoint.y += -(lightBulbOffset);
									}
								}
								if(j==3){
									if(lightMode == 1){
										endPoint.z += lightBulbLenght + lightBulbOffset;
										startPoint.z += lightBulbOffset;
									}
									if(lightMode == 2){
										endPoint.x = crossings[crossingID].x - (1+lightMode2Offset)*roadSize;
										endPoint.y = crossings[crossingID].y + 0.5*roadSize;
									}
									if(lightMode == 3){
										endPoint.x += -(lightBulbLenght + lightBulbOffset);
										startPoint.x += -(lightBulbOffset);
									}
								}
							}
							//SOUTH Light
							if(i==2){
								if(lightMode == 2){
									startPoint.x = crossings[crossingID].x + 0.5*roadSize;
									startPoint.y = crossings[crossingID].y - (1+lightMode2Offset)*roadSize;
								}
								if(j==0){
									if(lightMode == 1){
										endPoint.z += (lightBulbLenght + lightBulbOffset);
										startPoint.z += lightBulbOffset;
									}
									if(lightMode == 2){
										endPoint.x = crossings[crossingID].x + 0.5*roadSize;
										endPoint.y = crossings[crossingID].y + (1+lightMode2Offset)*roadSize;
									}
									if(lightMode == 3){
										endPoint.y += (lightBulbLenght + lightBulbOffset);
										startPoint.y += (lightBulbOffset);
									}
								}
								if(j==1){
									if(lightMode == 1){
										endPoint.x += (lightBulbLenght + lightBulbOffset);
										startPoint.x += lightBulbOffset;
									}
									if(lightMode == 2){
										endPoint.x = crossings[crossingID].x + (1+lightMode2Offset)*roadSize;
										endPoint.y = crossings[crossingID].y - 0.5*roadSize;
									}
									if(lightMode == 3){
										endPoint.x += (lightBulbLenght + lightBulbOffset);
										startPoint.x += lightBulbOffset;
									}
								}
								if(j==2){
									if(lightMode == 1){
										endPoint.z += -(lightBulbLenght + lightBulbOffset);
										startPoint.z += -lightBulbOffset;
									}
									if(lightMode == 2){
										endPoint.x = crossings[crossingID].x + 0.5*roadSize;
										endPoint.y = crossings[crossingID].y - (1.5+lightMode2Offset)*roadSize;
									}
									if(lightMode == 3){
										endPoint.y += -(lightBulbLenght + lightBulbOffset);
										startPoint.y += -lightBulbOffset;
									}
								}
								if(j==3){
									if(lightMode == 1){
										endPoint.x += -(lightBulbLenght + lightBulbOffset);
										startPoint.x += -lightBulbOffset;
									}
									if(lightMode == 2){
										endPoint.x = crossings[crossingID].x - (1+lightMode2Offset)*roadSize;
										endPoint.y = crossings[crossingID].y + 0.5*roadSize;
									}
									if(lightMode == 3){
										endPoint.x += -(lightBulbLenght + lightBulbOffset);
										startPoint.x += -lightBulbOffset;
									}
								}
							}

							//WEST Light
							if(i==3){
								if(lightMode == 2){
									startPoint.x = crossings[crossingID].x - (1+lightMode2Offset)*roadSize;
									startPoint.y = crossings[crossingID].y - 0.5*roadSize;
								}
								if(j==0){
									if(lightMode == 1){
										endPoint.y += (lightBulbLenght + lightBulbOffset);
										startPoint.y += lightBulbOffset;
									}
									if(lightMode == 2){
										endPoint.x = crossings[crossingID].x + 0.5*roadSize;
										endPoint.y = crossings[crossingID].y + (1+lightMode2Offset)*roadSize;
									}
									if(lightMode == 3){
										endPoint.y += (lightBulbLenght + lightBulbOffset);
										startPoint.y += lightBulbOffset;
									}
								}
								if(j==1){
									if(lightMode == 1){
										endPoint.z += (lightBulbLenght + lightBulbOffset);
										startPoint.z += lightBulbOffset;
									}
									if(lightMode == 2){
										endPoint.x = crossings[crossingID].x + (1+lightMode2Offset)*roadSize;
										endPoint.y = crossings[crossingID].y - 0.5*roadSize;
									}
									if(lightMode == 3){
										endPoint.x += (lightBulbLenght + lightBulbOffset);
										startPoint.x += lightBulbOffset;
									}
								}
								if(j==2){
									if(lightMode == 1){
										endPoint.y += -(lightBulbLenght + lightBulbOffset);
										startPoint.y += -lightBulbOffset;
									}
									if(lightMode == 2){
										endPoint.x = crossings[crossingID].x - 0.5*roadSize;
										endPoint.y = crossings[crossingID].y - (1+lightMode2Offset)*roadSize;
									}
									if(lightMode == 3){
										endPoint.y += -(lightBulbLenght + lightBulbOffset);
										startPoint.y += -lightBulbOffset;
									}
								}
								if(j==3){
									if(lightMode == 1){
										endPoint.z += -(lightBulbLenght + lightBulbOffset);
										startPoint.z += -lightBulbOffset;
									}
									if(lightMode == 2){
										endPoint.x = crossings[crossingID].x - (1.5+lightMode2Offset)*roadSize;
										endPoint.y = crossings[crossingID].y - 0.5*roadSize;
									}
									if(lightMode == 3){
										endPoint.x += -(lightBulbLenght + lightBulbOffset);
										startPoint.x += -lightBulbOffset;
									}
								}
							}
							if(lightMode == 1 || lightMode == 2 || lightMode == 3){
								markerS->points.push_back(startPoint);
								markerS->points.push_back(endPoint);
								markerS->scale.x = 0.5;
								markerS->scale.y = 1;
							}
							/*if(lightMode == 3){
								markerS->points.push_back(startPoint);
								markerS->points.push_back(middlePoint);
								markerS->points.push_back(endPoint);
								markerS->scale.x = 2.0;
							}*/
							
							
							LightBulbCollection::getInstance().LightBulbMarkerArray->markers.push_back(*markerS);
							LightBulbCollection::getInstance().numberOfLightBulbs++;
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
		lightSubscribers[i] = n.subscribe<anro_msgs::LightState>(topicName, 1, boost::bind(updateLightState, _1, topicName));
	}

	ros::Rate loop_rate(1);
	
	ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);
	ros::Publisher light_bulb_pub = n.advertise<visualization_msgs::MarkerArray>("light_bulb_marker_array", 100);

	while(ros::ok()){

		//Publikowanie
		if(!CarCollection::getInstance().lista->empty()){
			//marker_pub.publish(*markerArray);
			marker_pub.publish(*(CarCollection::getInstance().carMarkerArray));
		}
		
		light_bulb_pub.publish(*(LightBulbCollection::getInstance().LightBulbMarkerArray));

		ros::spinOnce();
	}

    return 0;
}
