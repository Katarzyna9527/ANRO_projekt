//samochod
#include <sys/types.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cstdlib>
#include "anro_msgs/car_init.h"
#include "anro_msgs/crossings_autocross_msg.h"
#include "anro_msgs/vizualization_auto_viz.h"
#include "car/car_to_car.h"
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
    float newLenght;
    float currentLenght;
    int16_t carID;		//samochód  zgłaszający albo do którego kierowana jest odpowiedź
    int16_t direction;		//przy pierwszym zapytaniu = -1
    float lenght;		//odległość do naspnego skrzyżowania
    int16_t nextCrossID;		//
    int16_t previousCrossID;	//
    int16_t previousCrossIDForViz;
    int16_t previousAutoID;
    int16_t futureCrossing;
    anro_msgs::crossings_autocross_msg msgA;//crossings
    anro_msgs::vizualization_auto_viz msgV;
    car::car_to_car carCarMessage;
    states state;
    ros::Publisher carViz;
    std::vector<int16_t> avaibleDirections;	//dosprtępne skręty

public:

    //ros::NodeHandle viz;
    ros::NodeHandle n;
    ros::NodeHandle carCarNode;
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

    void carCallback(const anro_msgs::crossings_autocross_msg::ConstPtr& msg){//anro_msgs::
        if(!msg->isMsgFromAuto) {
            //gotMessage=true;

            readTheMessage(msg);
        }
    }

    Car(){

        ros::ServiceClient client = n.serviceClient<anro_msgs::car_init>("init_car");//map
        //initiateService();
        //map::
        anro_msgs::car_init srv;
        uint8_t ReqID = 0;
        srv.request.req = (uint8_t) ReqID;
        if (client.call(srv)){
            carID=srv.response.carID;
            currentLenght = (float)srv.response.pathLenght;
            previousCrossID =srv.response.prevCrossing;//1;
            previousCrossIDForViz=previousCrossID;
            nextCrossID = srv.response.nextCrossing;
            isCrossed=false;
            speed = 1;
            direction = -1;
            lenght=0.0;
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
        msgA.length = (int)lenght;
        msgA.previousCrossID=previousCrossID;
        msgA.nextCrossID = nextCrossID;ROS_INFO("i chose :%d", direction);
        return msgA;
    }

    void fillMessageViz(){
        msgV.autoID=carID;
        msgV.startCrossID=previousCrossIDForViz;
        msgV.endCrossID=nextCrossID;
        msgV.distance=lenght;
    }

    void readTheMessage(const anro_msgs::crossings_autocross_msg::ConstPtr& msg){//anro_msgs::
        if(!msg->isMsgFromAuto && state!=tailgate){ ROS_INFO("I got msg from crossing with nextID:%d", msg->nextCrossID);
            if(state == waiting){
                newLenght = (float)msg->length;
                futureCrossing = msg->nextCrossID;
                speed=1;
                state = crossing;
                ROS_INFO("crossing");
            }
            else if(state == waitingForDir){
                previousAutoID = msg->previousAutoID;
                avaibleDirections = msg->availableDirections;
                if(previousAutoID==0||previousAutoID==carID){ROS_INFO("reading");
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

    void contactPrevCar()
    {
        std::stringstream ssss;
        ssss.clear();
        ssss.str("");
        ssss << "car_to_car_" << previousAutoID;
        ros::ServiceClient carCarClient = carCarNode.serviceClient<car::car_to_car>(ssss.str().c_str());
        carCarMessage.request.previousCrossing = previousCrossID;
        carCarMessage.request.nextCrossing = nextCrossID;
        //ROS_INFO("contacting car with ID ", previousAutoID);

        if (carCarClient.call(carCarMessage))
        {
            ROS_INFO("Previous car successfully called!");
            //ROS_INFO("My distance %d, its distance %d", lenght, carCarMessage.response.distance);

            if(carCarMessage.response.distance<5)
            {
                choseADir();
                state=drivingTowCrossing;
                ROS_INFO("driving towards crossing");
                speed=1;
                return;
            }
            if (carCarMessage.response.distance- lenght <= 4.0)
                speed=0;
            else
                speed=1;
        }
            else
                ROS_INFO("Fail to contact previous car, it's not responding!");

    }

    bool carService(car::car_to_car::Request &req, car::car_to_car::Response &res)
    {
        res.distance=lenght;
        ROS_INFO("sending info to car");
        return true;
    }

    /*void initiateService()
    {
        std::stringstream sss;
        sss.clear();
        sss.str("");
        sss << "car_to_car_" << carID;
        ros::ServiceServer carCarService = carCarNode.advertiseService(sss.str().c_str(), Car::carService);
        std::cout << "\n";
        std::cout << (sss.str().c_str());
    }*/


    void moveFrd(){ROS_INFO("%f", lenght);
                   if(state==tailgate)
                       contactPrevCar();
                   lenght = lenght + 0.04*speed;
                              if(lenght>currentLenght-2 && state==drivingTowCrossing){
                                  speed = 0;
                                  state = sendingDir;
                                  ROS_INFO("sending the direction I wanna go to the crossing");
                              }
                              else if(lenght == currentLenght){
                                  lenght=0.0;
                                  currentLenght=newLenght;
                                  previousCrossIDForViz=nextCrossID;
                                  nextCrossID=futureCrossing;
                              }
                              else if(lenght == 1.0 && state == crossing){
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
            previousCrossID=previousCrossIDForViz;
        }
    }

    int16_t giveNextCrossing(){
        return nextCrossID;
    }
    int16_t giveID(){
        return carID;
    }

    states giveState(){
        return state;
    }

    void setSpeed(int newSpeed){
        speed = newSpeed;
    }
    float giveLenght(){
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
    //int16_t instance = 0;
    //anro_msgs::
    anro_msgs::crossings_autocross_msg msgB;
    time_t tm=std::time(NULL);
    //time_t tmsg=std::time(NULL);
    std::srand( std::time( NULL ) );
    ros::init(argc, argv, "car_node", ros::init_options::AnonymousName);

    Car car;
    std::stringstream ss;
    ss << "crossing_" << car.giveNextCrossing();
    ROS_INFO("%s",ss.str().c_str());
    ros::Subscriber carSub = car.n.subscribe(ss.str().c_str(), 1000, &Car::carCallback, &car);
    ros::Publisher carPub = car.n.advertise<anro_msgs::crossings_autocross_msg>(ss.str().c_str(), 1000);//crossing
    ros::Rate loop_rate(25);

    std::stringstream sss;
    sss.clear();
    sss.str("");
    sss << "car_to_car_" << car.giveID();
    ros::ServiceServer carCarService = car.carCarNode.advertiseService(sss.str().c_str(), &Car::carService, &car);
    std::cout << "\n";
    std::cout << (sss.str().c_str());

    while(ros::ok()){
       // if(std::time(NULL)>tm+0.004){
            car.moveFrd();
          //  tm=std::time(NULL);
       // }
        state = car.giveState();
        if(((state == askingForDir || state == sendingDir)&&car.giveLenght()>3)|| state == crossed){
            msgB = car.fillTheMessage();
            carPub.publish(msgB);
            if(state == askingForDir){
                car.changeState(waitingForDir);
                ROS_INFO("waiting for possible directions");
            }
            else if(state == crossed){
                //msgB = car.fillTheMessage();
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
        loop_rate.sleep();
    }
}
