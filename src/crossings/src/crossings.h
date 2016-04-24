#ifndef _CROSSING_H_
#define _CROSSING_H_

#include "ros/ros.h"
#include <vector>

class Crossing
{
    private:
        int16_t ID;
        std::vector<int16_t>* neighbours;
        std::vector<int16_t>* lengths;

    public :
        Crossing(int16_t ID, std::vector<int16_t>* neighbours, std::vector<int16_t>* lengths);
        ~Crossing();

        int16_t getID();
        std::vector<int16_t>& getNeighbours();
        std::vector<int16_t>& getLengths();
} ;

#endif
