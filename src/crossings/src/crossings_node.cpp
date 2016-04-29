#include "crossings_lib.h"
#include "map/cross_init.h"
#include "map/cross_msg.h"
#include "crossings/autocross_msg.h"
#include "lights/LightState.h"
#include "lights/State.h"

map::cross_msg  crossCfg; // this crossing Configuration
ros::Publisher crossPub;
ros::Subscriber crossSub;
ros::Subscriber lightsSub;
std::vector<int16_t> myAvailDirs;

std::vector<crossings::autocross_msg> waitingQueues[4];
int carsThatCrossed[4] = {0, 0, 0, 0};
bool isCrossOccupied = false;

bool checkIfCanDrive(crossings::autocross_msg &response, int currCarPos);
void pushAutoIntoCrossing(crossings::autocross_msg &response, int currCarPos);

int calculateCarPosition(int previousCrossID)
{
    int currCarPos;
    for(currCarPos = 0; currCarPos < 4; currCarPos++)
        if(crossCfg.neighbours[currCarPos] == previousCrossID)
            break;

    return (currCarPos == 4 ? -1 : currCarPos);
}

bool isLightsPublishing = false;
lights::LightState currLightsCfg ; // current lights config
void lightsCallback(const lights::LightState::ConstPtr& msg)
{
    isLightsPublishing = true;
    ROS_INFO("Get data from Lights.");
    ROS_INFO("\tn=[%d, %d, %d, %d]", msg->n.A, msg->e.A, msg->s.A, msg->w.A);
    ROS_INFO("\te=[%d, %d, %d, %d]", msg->n.B, msg->e.B, msg->s.B, msg->w.B);
    ROS_INFO("\ts=[%d, %d, %d, %d]", msg->n.C, msg->e.C, msg->s.C, msg->w.C);
    ROS_INFO("\tw=[%d, %d, %d, %d]", msg->n.D, msg->e.D, msg->s.D, msg->w.D);
    currLightsCfg = lights::LightState(*msg);

    crossings::autocross_msg waitingCar;
    // sprawdzamy, czy czasem nie trzeba puścić następnej osoby z każdego kierunku
    for(int i = 0 ; i < 4; i++)
    {
        if(waitingQueues[i].size() > 0)
        {
            waitingCar = waitingQueues[i].at(0);
            if(checkIfCanDrive(waitingCar, i))
                pushAutoIntoCrossing(waitingCar, i);

        }
    }
}

// Write available directions to response and return position of this car
bool checkIfCanDrive(crossings::autocross_msg &response, int currCarPos)
{
    if(isCrossOccupied)
        return false;
    ROS_INFO("CrossInfo: Cross is not occupied");

    // musi byc pierwszy w kolejce, by mogl przejechac
    if(waitingQueues[currCarPos].at(0).autoID != response.autoID)
        return false;
    ROS_INFO("CrossInfo: Auto with ID=%d is first at the queue on position=%d", response.autoID, currCarPos);

    lights::State dirStates;
    switch(currCarPos)
    {
        case 0:
            dirStates = currLightsCfg.n;
            break;
        case 1:
            dirStates = currLightsCfg.e;
            break;
        case 2:
            dirStates = currLightsCfg.s;
            break;
        case 3:
            dirStates = currLightsCfg.w;
            break;
        default:
            return false;
    }
   
    switch(response.direction)
    {
        case 0:
            if(!dirStates.A)
                return false;
            break;
        case 1:
            if(!dirStates.B)
                return false;
            break;
        case 2:
            if(!dirStates.C)
                return false;
            break;
        case 3:
            if(!dirStates.D)
                return false;
            break;
        default:
            return false;
    }

    ROS_INFO("CrossInfo: Auto with ID=%d, could drive from position=%d", response.autoID, currCarPos);
    return true;
}

void pushAutoIntoCrossing(crossings::autocross_msg &response, int currCarPos)
{
    // usuwamy go z kolejki
    waitingQueues[currCarPos].erase(waitingQueues[currCarPos].begin());
                
    // może jechać, wyślij potwierdzenie
    isCrossOccupied = true;
    response.length = crossCfg.lengths[response.direction];
    response.nextCrossID = crossCfg.neighbours[response.direction];
    response.previousAutoID = carsThatCrossed[response.direction];

    crossPub.publish(response);
    ROS_INFO("CrossInfo: I pushed auto with ID=%d into crossing!", response.autoID); 
}

void fromCarCallback(const crossings::autocross_msg::ConstPtr& msg)
{
    if(msg->isMsgFromAuto == true)
    {
        ROS_INFO("I got msg from auto with ID:%d", msg->autoID);

        if(msg->isCrossed)
        {
            ROS_INFO("\tCar has left the crossing.");
            isCrossOccupied = false;
            carsThatCrossed[msg->direction] = msg->autoID;
        }
        else if(msg->direction == -1) // send him all avail directions
        {
            ROS_INFO("\tI got msg with direction=-1");
            crossings::autocross_msg response = *msg;
            response.isMsgFromAuto = false;
            
            int currCarPos = calculateCarPosition(response.previousCrossID);
            if(currCarPos == -1)
            {
                ROS_INFO("\tBad previous cross ID=%d ! No response...", response.previousCrossID);
                return;
            }

            ROS_INFO("This car current position: %d", currCarPos);

            response.availableDirections = myAvailDirs;
    
            if(waitingQueues[currCarPos].size() == 0)
                response.previousAutoID = 0;
            else
                response.previousAutoID = waitingQueues[currCarPos].at(0).autoID;

            waitingQueues[currCarPos].push_back(response);
            crossPub.publish(response);       

            ROS_INFO("\tI send to auto msg with available directions.");
        }
        else if(msg->direction >= 0 && msg->direction <= 3)
        {
            ROS_INFO("\tI got msg with direction=%d, A car would like to drive!", msg->direction);
            crossings::autocross_msg response = *msg;
            response.isMsgFromAuto = false;

            int currCarPos = calculateCarPosition(response.previousCrossID);
            if(currCarPos == -1)
            {
                ROS_INFO("\tBad previous cross ID=%d ! No response...", response.previousCrossID);
                return;
            }
               
            ROS_INFO("\tThis car current position : %d", currCarPos);

            if(checkIfCanDrive(response, currCarPos))
                pushAutoIntoCrossing(response, currCarPos);
            else
            {
                ROS_INFO("\tCar can NOT drive! It has to wait.");
                for(int i = 0 ; i < waitingQueues[currCarPos].size(); i++)
                    if(waitingQueues[currCarPos].at(i).autoID == response.autoID)
                        waitingQueues[currCarPos].at(i).direction = msg->direction;
            }
        }
    }
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "crossing_node", ros::init_options::AnonymousName );

    if(!Crossing::getInstance().initCrossing())
    {
        ROS_INFO("crossing_node: I have to exit...");
        return 1;
    }

    ros::spin();
    
    return 0;
}
