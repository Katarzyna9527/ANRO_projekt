#ifndef _CROSSING_H_
#define _CROSSING_H_

#include "ros/ros.h"
#include <vector>
#include <deque>

#include "crossings/autocross_msg.h"

#include "lights/LightState.h"
#include "lights/State.h"

#include "anro_msgs/cross_init.h"
#include "anro_msgs/map_cross_msg.h"

#define TURNING_BACK_ENABLE    
class Auto
{
        int currPos;
        crossings::autocross_msg carMsg;

    public:
        Auto(const crossings::autocross_msg::ConstPtr& msg);
        
        void setAvailDirs(const std::vector<int16_t> &availDirs);
        void setPreviousAutoID(int16_t previousAutoID);
        void setDirection(int16_t direction);
        void setNextCrossData(int16_t length, int16_t nextCrossID);

        int getCurrPos() const { return currPos; }
        int16_t getAutoID() const;
        int16_t getDirection() const { return carMsg.direction ; }
        const crossings::autocross_msg& getMsgToPublish() const ;
};

class Crossing
{
    private:
        // methods
        Crossing() {}
        Crossing(const Crossing&);
        Crossing& operator=(const Crossing&);
        ~Crossing() {}

        bool getCfgFromMap();
        void initCommunication();
        void initCrossData();

        void crossSubCallback(const crossings::autocross_msg::ConstPtr& crossMsg);
        void lightsSubCallback(const lights::LightState::ConstPtr& lightsMsg);
        void timeoutCallback(const ros::TimerEvent&);

        void autoAskAboutDirections(Auto& whichAuto);
        void autoWouldLikeToDrive(Auto& whichAuto, int16_t direction);
        void autoHasCrossedMe(Auto& whichAuto);

        void scanQueuesOnce();
        bool checkIfCanDrive(Auto &whichAuto);
        void pushIntoTheCrossing(Auto& whichAuto);

        class AutosReg
        {
            private :
                bool isInitiated;
                std::vector<Auto*> autosRegistry; 

                AutosReg() {}
                AutosReg(const AutosReg&);
                AutosReg& operator=(const AutosReg&);
                ~AutosReg() {}
            public :
                static AutosReg& getInstance();

                Auto*  getAutoFromMsg(const crossings::autocross_msg::ConstPtr& autoMsg);
                bool deleteAutoFromRegister(const Auto& whichAuto);
        };

        // Variables
        std::string myTopicName;
        std::string lightsTopicName;

        anro_msgs::map_cross_msg crossCfg;

        int crossSize;
        bool isInitiated;

        ros::Publisher crossPub;
        ros::Subscriber crossSub;
        ros::Subscriber lightsSub;
        ros::Timer timer;

        std::vector<int16_t> myAvailDirs;
        std::vector< std::deque<Auto*> > wait4DriveQueues;
        std::vector<int16_t> carsThatCrossed;

        bool isCrossOccupied;

        lights::LightState currLightsCfg;
        bool isLightsPublishing;

        // constants
        static const std::string DEFAULT_CROSSING_PREFIX;
        static const std::string DEFAULT_LIGHTS_PREFIX;
        enum {
            LIGHTS_CYCLE = 5,

            CROSS_PUB_BUFF_SZ =  1000,
            CROSS_SUB_BUFF_SZ =  1000,
            LIGHTS_SUB_BUFF_SZ = 1000
        }; 
            
    public:
        static Crossing& getInstance();
        int calculateCarPosition(int16_t previousCrossID);

        void printInfoAboutCrossing();

        bool initCrossing();
        void shutdownAll();
};
#endif
