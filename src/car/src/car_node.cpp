//samochod
#include <sys/types.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cstdlib>
#include "anro_msgs/car_init.h"//finish the address
#include "anro_msgs/crossings_autocross_msg.h"
#include "anro_msgs/vizualization_auto_viz.h"
#include <ctime>

enum states{
	crossing,
	crossed,
	askingForDir,
	sendingDir,
	drivingTowCrossing,
	waitingForDir,
	waiting,
	tailgate
};

class Car{

private:
	bool isCrossed;		//informuje o tym, czy samochód opuścił skrzyżowanie
	int32_t speed;
	int16_t newLenght;
	int16_t currentLenght;
	int16_t carID;		//samochód  zgłaszający albo do którego kierowana jest odpowiedź
	int16_t direction;		//przy pierwszym zapytaniu = -1
	int32_t lenght;		//odległość do naspnego skrzyżowania
	int16_t nextCrossID;		//
	int16_t previousCrossID;	//
	int16_t previousAutoID;
	int16_t futureGoal;
	anro_msgs::crossings_autocross_msg msgA;//crossings
	anro_msgs::vizualization_auto_viz msgV;
	states state;
	ros::Publisher carViz;
	std::vector<int16_t> avaibleDirections;	//dosprtępne skręty

public:

	//ros::NodeHandle viz;
	ros::NodeHandle n;
	//bool gotMessage;

	void choseADir(){
		std::vector<int16_t> aD;
		int j=0;
		for(int i=0; i<4;++i)
			if(avaibleDirections[i]==1){
				aD.push_back(i);
			}
		int i=std::rand()%aD.size();
		direction=aD[i];
	}

	void carCallback(const anro_msgs::crossings_autocross_msg::ConstPtr& msg){//crossings::
		if(!msg->isMsgFromAuto) {
			//gotMessage=true;
			ROS_INFO("I got msg from crossing with ID:%d", msg->nextCrossID);
			readTheMessage(msg);
		}
	}

	Car(){

		ros::ServiceClient client = n.serviceClient<anro_msgs::car_init>("init_car");//map
		//map::
		anro_msgs::car_init srv;
		uint8_t ReqID = 0;
		srv.request.req = (uint8_t) ReqID;
		if (client.call(srv)){
			carID=srv.response.carID;
			currentLenght = srv.response.pathLenght;
			previousCrossID =1; //srv.response.prevCrossing;
			nextCrossID = srv.response.nextCrossing;
			isCrossed=false;
			speed = 1;
			direction = -1;
			lenght=0;
			state=askingForDir;
			//gotMessage=false;
			carViz = n.advertise<anro_msgs::vizualization_auto_viz>("auto_viz", 1000);
			fillMessageViz();
			carViz.publish(msgV);
			ROS_INFO("iniciated, asking for directions");
		}
		else{
			ROS_ERROR("Failed to call service car_init");
		}
	}

	anro_msgs::crossings_autocross_msg fillTheMessage(){
		msgA.isMsgFromAuto = true;
		msgA.autoID = carID;
		msgA.direction = direction;
		msgA.isCrossed = isCrossed;
		msgA.length = lenght;
		msgA.previousCrossID=previousCrossID;
		msgA.nextCrossID = nextCrossID;
		return msgA;
	}

	void fillMessageViz(){
		msgV.autoID=carID;
		msgV.startCrossID=previousCrossID;
		msgV.endCrossID=nextCrossID;
		msgV.distance=lenght;
	}

	void readTheMessage(const anro_msgs::crossings_autocross_msg::ConstPtr& msg){//crossings::
		if(!msg->isMsgFromAuto){
			if(state == waiting){
				newLenght = msg->length;
				futureGoal = msg->nextCrossID;
				speed=1;
				state = crossing;
				ROS_INFO("crossing");
			}
			else if(state == waitingForDir){
				previousAutoID = msg->previousAutoID;
				avaibleDirections = msg->availableDirections;
				if(previousAutoID==0){ROS_INFO("reading");
				choseADir();
				state = drivingTowCrossing;
				ROS_INFO("driving towards crossing");
				}
				else{
					state = tailgate;
					ROS_INFO("tailgating");
				}
			}
		}
	}

	void moveFrd(){ROS_INFO("%d", lenght);
	lenght = lenght + speed;
	if(lenght>currentLenght-5 && state==drivingTowCrossing){
		speed = 0;
		state = sendingDir;
		ROS_INFO("sending the direction I wanna go to the crossing");
	}
	else if(lenght == currentLenght){
		lenght=0;
		currentLenght=newLenght;
		previousCrossID=nextCrossID;
		nextCrossID=futureGoal;
	}
	else if(lenght == 1 && state == crossing){
		isCrossed=true;
		direction=-1;
		state = crossed;
		ROS_INFO("crossed");
	}
	fillMessageViz();
	carViz.publish(msgV);
	}

	void changeState(states newState){
		state = newState;
		if(newState == askingForDir){
			isCrossed=false;
		}
	}

	int16_t giveNextCrossing(){
		return nextCrossID;
	}

	states giveState(){
		return state;
	}

	void setSpeed(int newSpeed){
		speed = newSpeed;
	}
	int16_t giveLenght(){
		return lenght;
	}
	void changeIsCrossed(){
		isCrossed=false;
	}

};

int main(int argc, char **argv)
{
	ROS_INFO("iniciation in progress");
	states state;
	int16_t instance = 0;
	//crossings::
	anro_msgs::crossings_autocross_msg msgB;
	time_t tm=std::time(NULL);
	time_t tmsg=std::time(NULL);
	std::srand( std::time( NULL ) );
	ros::init(argc, argv, "car_node", ros::init_options::AnonymousName);

	Car car;
	std::stringstream ss;
	ss << "crossing_" << car.giveNextCrossing();
	ROS_INFO("%s",ss.str().c_str());
	ros::Subscriber carSub = car.n.subscribe(ss.str().c_str(), 1000, &Car::carCallback, &car);
	ros::Publisher carPub = car.n.advertise<anro_msgs::crossings_autocross_msg>(ss.str().c_str(), 1000);//crossing
	ros::Rate loop_rate(10);

	while(ros::ok()){
		if(std::time(NULL)>tm+0.1){
			car.moveFrd();
			tm=std::time(NULL);
		}
		state = car.giveState();
		if(((state == askingForDir || state == sendingDir)&&car.giveLenght()>3)|| state == crossed){
			msgB = car.fillTheMessage();
			carPub.publish(msgB);
			if(state == askingForDir){
				car.changeState(waitingForDir);
				ROS_INFO("waiting for possible directions");
			}
			else if(state == crossed){
				msgB = car.fillTheMessage();
				ss.str("");
				ss.clear();
				ss << "crossing_" << car.giveNextCrossing();
				carPub = car.n.advertise<anro_msgs::crossings_autocross_msg>(ss.str().c_str(), 1000);//crossing
				carSub = car.n.subscribe(ss.str().c_str(), 1000, &Car::carCallback, &car);
				car.changeState(askingForDir);
				car.changeIsCrossed();
				ROS_INFO("asking for possible directions");
			}
			else if(state == sendingDir){
				car.changeState(waiting);
				ROS_INFO("waiting for a permission to enter the crossing");
			}
		}
		ros::spinOnce();
	}
}
