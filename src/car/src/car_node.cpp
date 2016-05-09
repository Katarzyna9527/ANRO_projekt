//samochod
#include <sys/types.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cstdlib>
#include "car/car_init.h"//finish the address
#include "car/autocross_msg.h"
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
   int16_t speed;
   int16_t newLenght;
   int16_t carID;		//samochód  zgłaszający albo do którego kierowana jest odpowiedź
   int16_t direction;		//przy pierwszym zapytaniu = -1
   bool isCrossed;		//informuje o tym, czy samochód opuścił skrzyżowanie
   std::vector<int16_t> avaibleDirections;	//dosprtępne skręty
   int16_t lenght;		//odległość do naspnego skrzyżowania
   int16_t nextCrossID;		//
   int16_t previousCrossID;	//
   int16_t previousAutoID;
   car::autocross_msg msgA;//crossings
   states state;   

public:

   ros::NodeHandle n;
   bool gotMessage;
   void choseADir(){
     while(true){
      int a = std::rand()%4;
      if(avaibleDirections[a]>0)
           direction = a;
	   break;
      }
   }

   void carCallback(const car::autocross_msg::ConstPtr& msg){//crossings::
      if(!msg->isMsgFromAuto) {
          gotMessage=true;
	  ROS_INFO("I got msg from crossing with ID:%d", msg->nextCrossID);
	  readTheMessage(msg);
      }
    }

   Car(){
		
      ros::ServiceClient client = n.serviceClient<car::car_init>("init_car");//map
      //map::
      car::car_init srv;
      uint8_t ReqID = 0;
      srv.request.req = (uint8_t) ReqID;
      if (client.call(srv)){
         carID=srv.response.carID;
         lenght = srv.response.pathLenght+5;
         previousCrossID =srv.response.prevCrossing;
         nextCrossID = srv.response.nextCrossing;
         isCrossed=false;
         speed = 1;
         direction = -1;
         state=askingForDir;
         gotMessage=false;
         ROS_INFO("iniciated, asking for directions");
      }
      else{
         ROS_ERROR("Failed to call service car_init");
      }
   }

   //crossings::
   car::autocross_msg fillTheMessage(){
       msgA.isMsgFromAuto = true;
       msgA.autoID = carID;        
       msgA.direction = direction;     
       msgA.isCrossed = isCrossed;      
       msgA.length = lenght;       
       msgA.nextCrossID = nextCrossID;
       return msgA;
   }

   void readTheMessage(const car::autocross_msg::ConstPtr& msg){//crossings::
        if(!msg->isMsgFromAuto){    
         if(state == waiting){
           newLenght = msg->length+10;       
           nextCrossID = msg->nextCrossID;
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
   
   void moveFrd(){
       lenght = lenght - speed;
       if(lenght<=10 && state == drivingTowCrossing){
         speed = 0;
         state = sendingDir;
         ROS_INFO("sending the direction I wanna go to the crossing");
       }
       else if(lenght == 0){
         lenght=newLenght;
       }
       else if(lenght ==5){
         isCrossed=true;
         state = crossed;
         ROS_INFO("crossed");
       }
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

};

int main(int argc, char **argv)
{
  ROS_INFO("iniciation in progress");
  states state;
  int16_t instance = 0;
  //crossings::
  car::autocross_msg msgB;
  time_t tm=std::time(NULL);
  time_t tmsg=std::time(NULL);
  std::srand( std::time( NULL ) );
  ros::init(argc, argv, "car_node", ros::init_options::AnonymousName);
  
  ros::NodeHandle n;

  Car car;
  std::stringstream ss;
  ss << "crossing_" << car.giveNextCrossing();
  
  ros::Publisher carPub = n.advertise<car::autocross_msg>("crossing_1", 1000);//crossingss.str().c_str()
  ros::Subscriber carSub = n.subscribe(ss.str().c_str(), 1000, &Car::carCallback, &car);
  ros::Rate loop_rate(10);
  while(ros::ok()){
    if(std::time(NULL)>tm+1){
       car.moveFrd();
       tm=std::time(NULL);ROS_INFO("2");
    }
    state = car.giveState();
    if(state == askingForDir || state == sendingDir || state == crossed){
       msgB = car.fillTheMessage();
       carPub.publish(msgB);
       car.gotMessage=false;
       if(state == askingForDir){carPub.publish(msgB);
         car.changeState(waitingForDir);
         ROS_INFO("waiting for possible directions");
       }
       else if(state == crossed){
         msgB = car.fillTheMessage();
         carPub.publish(msgB);
         ss.str("");
         ss.clear();
         ss << "crossing_" << car.giveNextCrossing();
         carPub = n.advertise<car::autocross_msg>(ss.str().c_str(), 1000);//crossing
         carSub = n.subscribe(ss.str().c_str(), 1000, &Car::carCallback, &car);
         car.changeState(askingForDir);
         ROS_INFO("asking for possible directions");
       }
       else if(state == sendingDir){
         car.changeState(waiting);
         ROS_INFO("waiting for a permission to enter the crossing");
       }
    }
    else if((state == waiting || state == waitingForDir)&& !car.gotMessage){
          //carPub.publish(msgB);ROS_INFO("5");
          tmsg=std::time(NULL);
    }
    ros::spinOnce();
  }
}
