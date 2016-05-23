using namespace std;

#include "ros/ros.h"
#include "anro_msgs/State.h"
#include "anro_msgs/LightState.h"
#include "anro_msgs/map_config.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <iostream>
#include <vector>
#include <ctime>

#include <string>


::anro_msgs::map_config map_data;		//structure with map configure

//function to create LightState structures from bool-information
anro_msgs::LightState make_lightState(bool a1, bool b1, bool c1, bool d1, 			//from north to other directions (n,e,s,w)
				   bool a2, bool b2, bool c2, bool d2, 			//from east to other directions (n,e,s,w)
				   bool a3, bool b3, bool c3, bool d3, 			//from south to other directions (n,e,s,w)
				   bool a4, bool b4, bool c4, bool d4){			//from west to other directions (n,e,s,w)

    anro_msgs::State lightsFor1;
    lightsFor1.A = a1;
    lightsFor1.B = b1;
    lightsFor1.C = c1;
    lightsFor1.D = d1;

    anro_msgs::State lightsFor2;
    lightsFor2.A = a2;
    lightsFor2.B = b2;
    lightsFor2.C = c2;
    lightsFor2.D = d2;

    anro_msgs::State lightsFor3;
    lightsFor3.A = a3;
    lightsFor3.B = b3;
    lightsFor3.C = c3;
    lightsFor3.D = d3;

    anro_msgs::State lightsFor4;
    lightsFor4.A = a4;
    lightsFor4.B = b4;
    lightsFor4.C = c4;
    lightsFor4.D = d4;

    anro_msgs::LightState lightsForAll;
    lightsForAll.n = lightsFor1;
    lightsForAll.e = lightsFor2;
    lightsForAll.s = lightsFor3;
    lightsForAll.w = lightsFor4;

    return lightsForAll;
}

//virtual class
class Allower{			//lights - node

protected:
    int which_state;				//current state
    int max_state;				//how many states are able
    int id;					//id of crossing

public:
    virtual void change_state()=0;
    virtual anro_msgs::LightState create_message()=0;

};

//crossing with 4 roads
class Allower4: public Allower{

private:
    anro_msgs::LightState allower_states[4];	//all states that may occure

public:

    Allower4(int index=0,int starting_state=1){
        max_state = 4;
        which_state=starting_state%max_state;
        id=index;
        allower_states[0]= make_lightState(0,0,1,1,  0,0,0,0,  1,1,0,0,  0,0,0,0);
        allower_states[1]= make_lightState(0,0,0,0,  1,0,0,1,  0,0,0,0,  0,1,1,0);
        allower_states[2]= make_lightState(0,1,0,0,  0,0,0,0,  0,0,0,1,  0,0,0,0);
        allower_states[3]= make_lightState(0,0,0,0,  0,0,1,0,  0,0,0,0,  1,0,0,0);
    }

     anro_msgs::LightState create_message(){
        return allower_states[which_state];
     }

     void change_state(){
        which_state++;
        which_state = which_state%max_state;
    }
};

//crossing with 3 roads
class Allower3: public Allower{

private:
    anro_msgs::LightState allower_states[3];
    int index_of_empty_road;

public:

    Allower3(int index=0, int starting_state=0){
        max_state = 3;
        which_state=starting_state % max_state;
        id=index;
        check_which_road_empty();
        switch(index_of_empty_road){

    case 0:
        {
            allower_states[0]= make_lightState(0,0,0,0,  0,0,0,1,  0,0,0,0,  0,1,1,0);
            allower_states[1]= make_lightState(0,0,0,0,  0,0,0,0,  0,1,0,1,  0,0,0,0);
            allower_states[2]= make_lightState(0,0,0,0,  0,0,1,1,  0,0,0,0,  0,0,0,0);
            break;
         }
    case 1:
        {
            allower_states[0]= make_lightState(0,0,1,1,  0,0,0,0,  1,0,0,0,  0,0,0,0);
            allower_states[1]= make_lightState(0,0,0,0,  0,0,0,0,  0,0,0,0,  1,0,1,0);
            allower_states[2]= make_lightState(0,0,0,0,  0,0,0,0,  1,0,0,0,  0,0,0,0);
            break;
         }
    case 2:
        {
            allower_states[0]= make_lightState(0,0,0,0,  1,0,0,1,  0,0,0,0,  0,1,0,0);
            allower_states[1]= make_lightState(0,1,0,1,  0,0,0,0,  0,0,0,0,  0,0,0,0);
            allower_states[2]= make_lightState(0,0,0,0,  0,0,0,0,  0,0,0,0,  1,1,0,0);
            break;
        }
    case 3:
        {
            allower_states[0]= make_lightState(0,0,1,0,  0,0,0,0,  1,1,0,0,  0,0,0,0);
            allower_states[1]= make_lightState(0,0,0,0,  1,0,1,0,  0,0,0,0,  0,0,0,0);
            allower_states[2]= make_lightState(0,1,1,0,  0,0,0,0,  0,0,0,0,  0,0,0,0);
            break;
        }
        }

    }

    void check_which_road_empty(){
        for(int i =0; i< 4; ++i){
            if(map_data.response.crossings[id-1].neighbours[i]<=0){
                index_of_empty_road=i;
                break;
            }
        }
    }

    anro_msgs::LightState create_message(){
	return allower_states[which_state];
    }

     void change_state(){
        which_state++;
        which_state = which_state%max_state;
    }
};

//crossing with 2 roads
class Allower2 : public Allower{

private:
    anro_msgs::LightState state;
    int index_of_empty_road1;
    int index_of_empty_road2;

public:

    Allower2(int index=0, int starting_state=0){
        id=index;
        check_which_road_empty();	//checking which roads arent empty
		
        if((index_of_empty_road1 + index_of_empty_road2) % 2 == 0)
		cout<<"HERE I HAVE CROSSING THAT IS NOT A CROSSING :)"<<endl;

        else if(index_of_empty_road1 + index_of_empty_road2 == 1)
		state=make_lightState(0,1,0,0,  1,0,0,0,  0,0,0,0,  0,0,0,0);

        else if(index_of_empty_road1 + index_of_empty_road2 == 5)
		state=make_lightState(0,0,0,0,  0,0,0,0,  0,0,0,1,  0,0,1,0);

        else if(index_of_empty_road1 + index_of_empty_road2 == 3){

            if(index_of_empty_road1 == 0 || index_of_empty_road2 ==0)
                state=make_lightState(0,0,0,1,  0,0,0,0,  0,0,0,0,  1,0,0,0);

            else
                state=make_lightState(0,0,0,0,  0,0,1,0,  0,1,0,0,  0,0,0,0);
        }
    }

    void check_which_road_empty(){
        for(int i =0; i< 4; ++i){
            if(map_data.response.crossings[id-1].neighbours[i]>0){
                index_of_empty_road1=i;
                i++;
                for(int j=i; j<4; ++j){
                    if(map_data.response.crossings[id-1].neighbours[j]>0){
                        index_of_empty_road2=j;
                        break;
                    }
                }
                break;
            }
        }
    }

     anro_msgs::LightState create_message(){
        return state;
     }

     void change_state(){
        //nothing to do
    }
};

class LightsOnCrossing{

    int id;
    Allower* allower;
    ros::NodeHandle lightsNode;
    ros::Publisher lightsPub;

public:
    LightsOnCrossing(int number_of_crossing=0){
        id = number_of_crossing;

         int nRodes=0;		//how many roads
        for(int i =0; i<4; ++i){
            if(map_data.response.crossings[id-1].neighbours[i]>0)nRodes++; 
        }
  

        if(nRodes==4)
            allower = new Allower4(number_of_crossing, number_of_crossing);

        else if(nRodes==3)
            allower = new Allower3(number_of_crossing, number_of_crossing);
        else if(nRodes==2)
            allower = new Allower2(number_of_crossing, number_of_crossing);


        stringstream ss;
        ss<<id;
        
        string topicName="lights_";
        string number=ss.str();
        topicName=topicName+number;
            lightsPub = lightsNode.advertise<anro_msgs::LightState>(topicName, 1000);
       // cout<<"\n"<<"Lights for crossing " + number + " created";
    }

    void next_state(){
        allower->change_state();
    }

    void send_msg(){

        lightsPub.publish(allower->create_message());
    }
};

::anro_msgs::map_config getMapConfiguration(){								//POBIERANIE INFORMACJI Z MAPY

    ::anro_msgs::map_config mapConfiguration;
    ros::NodeHandle mapNode;

    ros::ServiceClient lightsClient = mapNode.serviceClient<::anro_msgs::map_config>("get_map_config");
    mapConfiguration.request.req = 1;
    ROS_INFO("Waiting for map...");
    while(1){
        if(lightsClient.call(mapConfiguration))
            break;
    }
    ROS_INFO("Map received!");
    return mapConfiguration;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lights");
    ros::NodeHandle dataNode;
    map_data = getMapConfiguration();
        cout<<"/n"<<"Map configuration received"<<endl;

    int nNodes=map_data.response.number_of_crossings;

    LightsOnCrossing *lights_on_crossings[nNodes];

    for(int i =0;i<nNodes; ++i){
        lights_on_crossings[i] = new LightsOnCrossing(map_data.response.crossings[i].ID);
    }

	cout<<"\n"<<"Lights done";
	
	double time;
	dataNode.param("rate", time, 0.25);
	ros::Rate loop_rate(time);

    while(ros::ok()){
            for(int i =0; i < nNodes; ++i){

                lights_on_crossings[i]->send_msg();
                lights_on_crossings[i]->next_state();
            }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
