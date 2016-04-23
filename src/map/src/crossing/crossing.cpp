#include "crossing.h"

Crossing::Crossing(int16_t ID, std::vector<int16_t>* neighbours, std::vector<int16_t>* lengths)
{
    this->neighbours = neighbours;
    this->lengths = lengths;
    this->ID = ID;
}

Crossing::~Crossing()
{
    delete neighbours;
    delete lengths;
}

int16_t Crossing::getID() { return ID; }
std::vector<int16_t>& Crossing::getNeighbours() { return *neighbours; }
std::vector<int16_t>& Crossing::getLengths() { return *lengths; }
