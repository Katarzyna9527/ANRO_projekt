#include "crossings_lib.h"

bool Crossing::AutosReg::deleteAutoFromRegister(const Auto& whichAuto)
{   
    // check if exist
	auto it = autosRegistry.begin();
    for( ; it != autosRegistry.end(); it++)
    {
        if(*it == &whichAuto)
        {
            autosRegistry.erase(it);
            return true;
        }
    }

	if(it == autosRegistry.end())
	{
		ROS_INFO("deleteAutoFromRegister: this car isn't in register!");
	}
	
    return false;
}

Auto* Crossing::AutosReg::getAutoFromMsg(const anro_msgs::crossings_autocross_msg::ConstPtr& autoMsg)
{
    // check if is it valid
    if(Crossing::getInstance().calculateCarPosition(autoMsg->previousCrossID) == -1)
        return 0;

    // check if exist
    for(auto it = autosRegistry.begin(); it != autosRegistry.end(); it++)
        if((*it)->getAutoID() == autoMsg->autoID)
            return *it;
    // we must create new Auto and store it
    Auto *newAuto = new Auto(autoMsg);
    autosRegistry.push_back(newAuto);

    return newAuto;
}

Crossing::AutosReg& Crossing::AutosReg::getInstance()
{
    static AutosReg instance ;
    return instance ;
}
