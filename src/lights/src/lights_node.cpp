using namespace std;

#include "ros/ros.h"
#include "lights/State.h"
#include "lights/LightState.h"
#include "map/map_config.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <iostream>
#include <vector>
#include <ctime>

#include <string>

class DirectionAllower{									//klasa odpowiadająca za zmianę stanu świateł we wszystkich kierunkach na skrzyżowaniu
	private:
		bool directionMatrix [4][4];					//macierze odpowiadająca przejazdowi z każdego kierunku do innego kierunku (albo do siebie samego)
	public:
		bool shift();									//przesuwanie wierszy macierzy -> potrzebne do zmiany stanu świateł
		bool shiftRight(int i);							//przesuwanie w prawo wiersza 'i'
		bool shiftLeft (int i);							//jw. tylko w lewo
		vector <bool> takeDirections(int direction);	//pobranie wektora dozwolonych kierunków przejazdu, dla określonego kierunku (wektor kolumnowy)
		DirectionAllower();								//konstruktor
		~DirectionAllower();
		void show();									//klasa pomocnicza, pokazująca, czy wszystko działa OK
};
																														//SKĄD
DirectionAllower::DirectionAllower(){					//konstruktor												 D	 N E S W
	bool tab [4][4] = { {1,0,0,0},						//tworzymy macierz zgodnie z takim oto założeniem: 		   	 O N X X X X
						{1,0,0,0},						//															 K E X X X X
						{0,0,1,0},						//															 Ą S X X X X
						{0,0,1,0},};					//                     										 D W X X X X
	for(int i = 0; i < 4; i++)							//przypisujemy tę macierz do macierzy w klasie
		for(int j = 0; j < 4; j++)
			directionMatrix[i][j] = tab[i][j];

}

DirectionAllower::~DirectionAllower(){					//destruktor

}

bool DirectionAllower::shiftRight(int i){				//przesuwanie w prawo wiersza 'i' macierzy
	int temp;
	temp = directionMatrix[i][3];
	for(int j = 3; j > 0; j--)
		directionMatrix[i][j] = directionMatrix[i][j-1];
	directionMatrix[i][0] = temp;
	return true;
}

bool DirectionAllower::shiftLeft(int i){				//przesuwanie w lewo wiersza 'i' macierzy
	int temp;
	temp = directionMatrix[i][0];
	for(int j = 0; j < 3; j++)
		directionMatrix[i][j] = directionMatrix[i][j+1];
	directionMatrix[i][3] = temp;
	return true;
}

bool DirectionAllower::shift(){							//przesuwanie, odpowiadające zmianie stanu naszych świateł
	this->shiftLeft(0);
	this->shiftRight(1);
	this->shiftLeft(2);
	this->shiftRight(3);
	return true;
}

vector <bool> DirectionAllower::takeDirections(int direction){			//pobranie kierunków do jazdy, dla odpowiedniego wlotu skrzyżowania
	vector <bool> column;
	for(int i = 0; i < 4; i++)
		column.push_back(directionMatrix[i][direction]);				//do wektora wstawiamy każdy element z kolumny o numerze "direction" (N=0,E=1,S=2,W=3)
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

::map::map_config getMapConfiguration(){								//POBIERANIE INFORMACJI Z MAPY

    ::map::map_config mapConfiguration;
    ros::NodeHandle mapNode;											

    ros::ServiceClient lightsClient = mapNode.serviceClient<::map::map_config>("get_map_config");
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

int main(int argc, char **argv)											//główny program
{
    ros::init(argc, argv, "lights");

    ::map::map_config map = getMapConfiguration();

    int nNodes=map.response.number_of_crossings;						//number of nodes

    DirectionAllower *allower [nNodes];									//tablica przełączników stanu
    lights::LightState lightStates [nNodes];							//struktury do wysyłania informacji skrzyżowaniu

	for(int x = 0; x < nNodes; x++){									//pętla, w której tworzymy nNodes sygnalizatorów, a ponadto przesuwamy je w zależności
            allower[x] = new DirectionAllower();
            for(int y = x%4; y > 0; y--)
                allower[x]->shift();
        }
    lights::State states [4];											//wszystkie sygnalizatory aktualizujemy, przechodząc do kolejnego stanu

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
	
	ros::Rate loop_rate(0.25);
	
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
			
			ros::spinOnce();
			loop_rate.sleep();
    }
	return 0;
}
