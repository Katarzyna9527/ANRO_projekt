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
   int16_t avaibleDirections[4];	//dosprtępne skręty
   int16_t lenght;		//odległość do naspnego skrzyżowania
   int16_t nextCrossID;		//
   int16_t previousCrossID;	//
   int16_t previousAutoID;
   car::autocross_msg msgA;//crossings
   states state;   

public:

   void choseADir(){
     while(true){
      int a = std::rand()%4;
      if(avaibleDirections[a]>0)
           direction = a;
	   break;
      }
   }
   

	void carCallback(const car::autocross_msg& msg)//crossings::
	{
	    if(msg.isMsgFromAuto != true)
	    {
		ROS_INFO("I get msg from crossing with ID:%d", msg.nextCrossID);
		readTheMessage(msg);
	    }
	}

   Car(){
		ros::NodeHandle n;
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
         state=askingForDir;
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

   void readTheMessage(const car::autocross_msg& msg){//crossings::
       if(msg.isMsgFromAuto != true){    
         if(state == waiting){
           newLenght = msg.length+10;       
           nextCrossID = msg.nextCrossID;
	   speed=1;
           state = crossing;
         }
         else if(state == waitingForDir){
           previousAutoID = msg.previousAutoID;
           for(int j = 0; j<4;j++)
             avaibleDirections[j] = msg.availableDirections[j];
           if(previousAutoID==0){
             choseADir();
             state = drivingTowCrossing;
           }
           else{
             state = tailgate;
           }
         }
       }
   }
   
   void moveFrd(){
       lenght = lenght - speed;
       if(lenght<=10 && state == drivingTowCrossing){
         speed = 0;
         state = sendingDir;
       }
       else if(lenght == 0){
         lenght=newLenght;
       }
       else if(lenght ==5){
         isCrossed=true;
         state = crossed;
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
  states state;
  int16_t instance = 0;
  //crossings::
  car::autocross_msg msgB;
  time_t tm=std::time(NULL);
  std::srand( std::time( NULL ) );
  ros::init(argc, argv, "car_node", ros::init_options::AnonymousName);
  
  ros::NodeHandle n;

  Car car;

  std::stringstream ss;
  ss << "crossing_" << car.giveNextCrossing();
  ros::Publisher carPub = n.advertise<car::autocross_msg>(ss.str().c_str(), 1000);//crossing
  ros::Subscriber carSub = n.subscribe(ss.str().c_str(), 1000, &Car::carCallback, &car);
  ros::Rate loop_rate(10);

  while(ros::ok()){
    if(std::time(NULL)>tm+1){
       car.moveFrd();
       tm=std::time(NULL);
    }
    state = car.giveState();
    if(state == askingForDir || state == sendingDir || state == crossed){
       msgB = car.fillTheMessage();
       carPub.publish(msgB);
       if(state == askingForDir){
         car.changeState(waitingForDir);
       }
       else if(state == crossed){
         msgB = car.fillTheMessage();
         carPub.publish(msgB);
         ss.str("");
         ss.clear();
         ss << "crossing_" << car.giveNextCrossing();
         ros::Publisher carPub = n.advertise<car::autocross_msg>(ss.str().c_str(), 1000);//crossing
         ros::Subscriber carSub = n.subscribe(ss.str().c_str(), 1000, &Car::carCallback, &car);
         car.changeState(askingForDir);
       }
       else{
         car.changeState(waiting);
       }
    }
    ros::spinOnce();
  }
}
