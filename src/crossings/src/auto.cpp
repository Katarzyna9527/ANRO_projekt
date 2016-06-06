#include "crossings_lib.h"

Auto::Auto(const anro_msgs::crossings_autocross_msg::ConstPtr& msg)
{
    carMsg = *msg;
    currPos = Crossing::getInstance().calculateCarPosition(carMsg.previousCrossID);
    carMsg.isMsgFromAuto = false;
}

void Auto::setAvailDirs(const std::vector<int16_t> &availDirs)
{
    carMsg.availableDirections = availDirs;
    carMsg.availableDirections[currPos] = 0;
}

void Auto::setPreviousAutoID(int16_t previousAutoID)
{
    carMsg.previousAutoID = previousAutoID;
}

void Auto::setDirection(int16_t direction)
{
    carMsg.direction = direction;
}

void Auto::setNextCrossData(int16_t length, int16_t nextCrossID)
{
    carMsg.length = length;
    carMsg.nextCrossID = nextCrossID;
}

const anro_msgs::crossings_autocross_msg& Auto::getMsgToPublish() const 
{
    return carMsg;
}

int16_t Auto::getAutoID() const
{
    return carMsg.autoID;
}

