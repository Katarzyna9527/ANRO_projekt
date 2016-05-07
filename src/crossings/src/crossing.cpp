#include "crossings_lib.h"

const std::string Crossing::DEFAULT_CROSSING_PREFIX = "crossing_";
const std::string Crossing::DEFAULT_LIGHTS_PREFIX = "lights_";

int Crossing::calculateCarPosition(int16_t previousCrossID)
{
    int i ;
    for(i = 0; i < crossSize; i++)
        if(crossCfg.neighbours[i] == previousCrossID)
            return i;

    return -1;
}

void Crossing::autoAskAboutDirections(Auto& whichAuto)
{
    whichAuto.setAvailDirs(myAvailDirs);
    whichAuto.setDirection(-1);

    // we must put this car into queue 
    int index = whichAuto.getCurrPos();
    if(wait4DriveQueues[index].size() == 0)
        whichAuto.setPreviousAutoID(0);
    else
        whichAuto.setPreviousAutoID(wait4DriveQueues[index].back()->getAutoID());

    wait4DriveQueues[index].push_back(&whichAuto);

    // write response
    crossPub.publish(whichAuto.getMsgToPublish());
}

void Crossing::autoWouldLikeToDrive(Auto& whichAuto, int16_t direction)
{
    whichAuto.setDirection(direction);
    // so, we've to check if auto can be pushed into the crossing
    if(checkIfCanDrive(whichAuto))
    {
        pushIntoTheCrossing(whichAuto);
    }
}

void Crossing::autoHasCrossedMe(Auto& whichAuto)
{
    isCrossOccupied = false;
    carsThatCrossed[whichAuto.getDirection()] = whichAuto.getAutoID();

    int pos = whichAuto.getCurrPos();

    AutosReg::getInstance().deleteAutoFromRegister(whichAuto);
    ROS_INFO("autohascrossedme: deleted from regiser");

    if(wait4DriveQueues[pos].size() == 0)
        return ; // the queue is empty

    ROS_INFO("autocrossedme: Queue is not empty");

    Auto& secondAuto = *wait4DriveQueues[pos].front();
    if(secondAuto.getDirection() != -1)
    {
        ROS_INFO("autoHasCrossedME: checkinh");
        if(checkIfCanDrive(secondAuto))
        {
            ROS_INFO("autoHasCrossedMe: pushing next car");
            pushIntoTheCrossing(secondAuto);
            ROS_INFO("autoHasCrossed: Next car can drive! ID=%d", secondAuto.getAutoID());
        }
    }
    ROS_INFO("autohascrossedme: all done");
}

void Crossing::pushIntoTheCrossing(Auto &whichAuto)
{
    isCrossOccupied = true;

    int index = whichAuto.getCurrPos();
    // clear this element from queue
    wait4DriveQueues[index].pop_front();
    
    // prepare answer
    int dir = whichAuto.getDirection();
    whichAuto.setPreviousAutoID(carsThatCrossed[dir]);
    whichAuto.setNextCrossData(crossCfg.lengths[dir], crossCfg.neighbours[dir]);

    crossPub.publish(whichAuto.getMsgToPublish());
    ROS_INFO("pushIntoCrossing: A Car ID=%d successfully pushed into the crossing!", whichAuto.getAutoID());
}

bool Crossing::checkIfCanDrive(Auto& whichAuto)
{
    if(isCrossOccupied)
        return false;
    ROS_INFO("checkIfCanDrive: Crossing is not occupied.");
 
    if(whichAuto.getDirection() == -1)
        return false;
    ROS_INFO("checkIfCanDrive: This Auto would like to drive ID=%d", whichAuto.getAutoID());

    int16_t pos = whichAuto.getCurrPos(),
            dir = whichAuto.getDirection();

    // if it is not in the front of the queue
    if(wait4DriveQueues[pos].front() != &whichAuto)
        return false;
    ROS_INFO("checkIfCanDrive: This car is on the top of the queue!");

    lights::State dirStates;
    switch(pos)
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

    switch(dir)
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

    ROS_INFO("checkIfCanDrive: This Car can drive!");
    return true;
}

void Crossing::crossSubCallback(const crossings::autocross_msg::ConstPtr& crossMsg)
{
    ROS_INFO("crossSubCallback: I've got a callback");
    if(crossMsg->isMsgFromAuto)
    {
        Auto* thisAuto = AutosReg::getInstance().getAutoFromMsg(crossMsg);
        if(thisAuto == 0) {
            ROS_INFO("NullPointer!!!");
            return;
        }
        
        ROS_INFO("\tI receive message from auto with ID:%d", thisAuto->getAutoID());

        if(crossMsg->isCrossed)
        {
            ROS_INFO("\t\tCar has left the crossing.");
            autoHasCrossedMe(*thisAuto);
        }
        else if(crossMsg->direction == -1)
        {
            ROS_INFO("\t\tI got msg with direction=-1. A car wants to know about crossing");
            autoAskAboutDirections(*thisAuto);
        }
        else if(crossMsg->direction >= 0 && crossMsg->direction <= crossSize-1)
        {
            ROS_INFO("\t\tI got msg with direction=%d. A car would to drive", crossMsg->direction);
            autoWouldLikeToDrive(*thisAuto, crossMsg->direction);
        }
        else
        {
            ROS_INFO("\t\tUnrecognized message from this car. No response...");
        }
    }
}

void Crossing::lightsSubCallback(const lights::LightState::ConstPtr& lightsMsg)
{
    isLightsPublishing = true;
    ROS_INFO("lightsCallback: Get data from Lights.");
    ROS_INFO("\tn=[%d, %d, %d, %d]", lightsMsg->n.A, lightsMsg->e.A, lightsMsg->s.A, lightsMsg->w.A);
    ROS_INFO("\te=[%d, %d, %d, %d]", lightsMsg->n.B, lightsMsg->e.B, lightsMsg->s.B, lightsMsg->w.B);
    ROS_INFO("\ts=[%d, %d, %d, %d]", lightsMsg->n.C, lightsMsg->e.C, lightsMsg->s.C, lightsMsg->w.C);
    ROS_INFO("\tw=[%d, %d, %d, %d]", lightsMsg->n.D, lightsMsg->e.D, lightsMsg->s.D, lightsMsg->w.D);

    currLightsCfg = *lightsMsg;

    Auto* waitingAuto;
    for(auto wQueue = wait4DriveQueues.begin() ; wQueue != wait4DriveQueues.end(); wQueue++)
    {
        if(wQueue->size() > 0) // if there is anyone in queue
        {
            waitingAuto = wQueue->front();
            if(checkIfCanDrive(*waitingAuto))
            {
                ROS_INFO("lightsCallback: Next auto can drive! ID=%d", waitingAuto->getAutoID());
                pushIntoTheCrossing(*waitingAuto);
            }
        }
    }
}

void Crossing::timeoutCallback(const ros::TimerEvent&)
{
    if(!isLightsPublishing)
    {
        ROS_INFO("timeoutCallback: Lights topis is not working!");
    }
    else
        isLightsPublishing = false;
}

void Crossing::printInfoAboutCrossing()
{
    ROS_INFO("Crossing: I received following data from Map:");
    ROS_INFO("\tMy ID is %d. My topic name is: %s", crossCfg.ID, myTopicName.c_str());
    ROS_INFO("\tNeighbours: n=%d, e=%d, s=%d, w=%d",
        crossCfg.neighbours[0],crossCfg.neighbours[1],crossCfg.neighbours[2],crossCfg.neighbours[3]);
    ROS_INFO("\tLengths:    n=%d, e=%d, s=%d, w=%d",
        crossCfg.lengths[0], crossCfg.lengths[1], crossCfg.lengths[2], crossCfg.lengths[3]);
}

void Crossing::initCommunication()
{
    ros::NodeHandle n;
    crossPub = n.advertise<crossings::autocross_msg>(myTopicName, CROSS_PUB_BUFF_SZ);
    crossSub = n.subscribe(myTopicName, CROSS_SUB_BUFF_SZ, &Crossing::crossSubCallback, this);
    lightsSub = n.subscribe(lightsTopicName, LIGHTS_SUB_BUFF_SZ, &Crossing::lightsSubCallback, this); 
    
    timer = n.createTimer(ros::Duration(LIGHTS_CYCLE), &Crossing::timeoutCallback, this);
}

void Crossing::initCrossData()
{
    crossSize = crossCfg.neighbours.size();

    // check available directions
    for(auto it = crossCfg.neighbours.begin() ; it != crossCfg.neighbours.end(); it++)
        myAvailDirs.push_back(*it == 0 ? 0 : 1);

    // init arrays with size that our Crossing is
    wait4DriveQueues.resize(crossSize);
    carsThatCrossed.resize(crossSize);

    isCrossOccupied = false;
    isLightsPublishing = false; // at the init, we don't know, that lights are working


    // init topic names
    std::stringstream ss;
    ss << DEFAULT_CROSSING_PREFIX << crossCfg.ID;
    myTopicName = ss.str();
    ss.str("");
    ss << DEFAULT_LIGHTS_PREFIX << crossCfg.ID;
    lightsTopicName = ss.str();
}

bool Crossing::getCfgFromMap()
{
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<map::cross_init>("init_crossing");
    map::cross_init srv ;

    if(!client.call(srv))
        return false;

    crossCfg = srv.response.crossing;
    return true;
}

bool Crossing::initCrossing()
{
    ros::NodeHandle n;

    ROS_INFO("sIema");

    Crossing& crossing = Crossing::getInstance();

    if(crossing.isInitiated)
        return false;

    if(!crossing.getCfgFromMap())
    {
        ROS_INFO("initCrossing: can't get data from Map!");
        return false;
    }

    if(crossing.crossCfg.ID == 0)
    {
        ROS_INFO("initCrossing: Map Server didn't accept my request! :(");
        return false;
    }

    printInfoAboutCrossing();
    initCrossData();
    initCommunication();


    isInitiated = true;
    return true;
}

Crossing& Crossing::getInstance()
{
    static Crossing crossing;
    return crossing;
}