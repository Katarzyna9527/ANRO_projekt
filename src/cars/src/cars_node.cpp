#include "ros/ros.h"
#include "map/car_init.h"
#include "cars/car_to_car.h"
#include "cars/auto_viz.h"
#include "std_msgs/String.h"
#include "cars/autocross_msg.h"
#include <stdio.h>
#include <iostream>
#include <vector>
#include <ctime>

#include <string>

map::car_init initialCarSettings;
cars::autocross_msg message;
cars::car_to_car carCarMessage;
cars::auto_viz carVizMessage;

int multiplication = 1;

class Car
{
public:
	int carID;
	int previousCrossing;
	int nextCrossing;
	int velocity;
	int distanceToGo;
	int distanceSoFar;			//ile juz przejechal samochod od poprzedniego skrzyzowania
	int pickedDirection;
	int isMessageRecieved;		//0- nie, 1- pierwsza, 2-druga
	bool secondMessageIsSend;
	int previousCarID;
	bool iCanGo;			//true- nie ma przede mna samochodu albo jest wystarczajac daleko
	int myPatience;			//ile czekam na odp od skrzyzowania zanim rezygnuje

	Car();
	~Car();
	void updateCar();
	void pickDirection(::cars::autocross_msg message);
	void getInitialSettings();
	cars::autocross_msg initialQuestion();
	cars::autocross_msg secondQuestion();
	cars::autocross_msg lastQuestion();
	static Car& getInstance()
	{
		static Car instance;
		return instance;
	}

	bool carCarResponse(cars::car_to_car::Response &res, bool whichOne);
	bool carCarRequestCheckout(cars::car_to_car::Request &req);
	static bool carCarService(cars::car_to_car::Request &req, cars::car_to_car::Response &res)
	{
		Car& car = Car::getInstance();
		car.carCarResponse(res, car.carCarRequestCheckout(req));
		ROS_INFO("I've just send info to car with ID=%d", req.carID);
		return true;
	};
	void infoForViz();
	void publicMyInfo();


};
Car *car;

Car::~Car()
{
}

Car::Car()
{

	carID = 0;
	distanceToGo = 0;
	previousCrossing = 0;
	nextCrossing = 0;

	distanceSoFar = 0;
	velocity = 1;
	pickedDirection = 0;
	isMessageRecieved = 0;
	secondMessageIsSend = false;
	previousCarID = 0;
	iCanGo = true;
	myPatience = 15;

}

bool Car::carCarResponse(cars::car_to_car::Response &res, bool whichOne)
{
	if (whichOne)
	{
		res.carID = car->carID;
	}
	else
	{
		res.carID = 0;
	}
	res.distanceSoFar = car->distanceSoFar;
	ROS_INFO("My send car ID %d, my distance send %d", car->carID, car->distanceSoFar);

}


bool Car::carCarRequestCheckout(cars::car_to_car::Request &req)
{
	if ((req.previousCrossing != car->previousCrossing) || (req.nextCrossing != car->nextCrossing))
		return false;
	else
		return true;
}

void Car::updateCar()
{
	previousCrossing = nextCrossing;
	nextCrossing = message.nextCrossID;
	distanceToGo = (message.length);
	distanceSoFar = 0;
}

void Car::pickDirection(::cars::autocross_msg message)
{
	int avaliableDirections[4];
	for (int i = 0; i < 4; i++)
	{
		avaliableDirections[i] = message.availableDirections[i];
	}
	int a = 0;
	while (avaliableDirections[a] == 0)
	{
		a = rand() % 4;
	}
	pickedDirection = a;

}



void carCallback(const cars::autocross_msg::ConstPtr& messageFromCar)
{
	if ((messageFromCar->isMsgFromAuto == false) && (messageFromCar->autoID == car->carID))
	{
		message = *messageFromCar;
		if (messageFromCar->nextCrossID == 0) car->isMessageRecieved = 1;
		if (messageFromCar->nextCrossID != 0) car->isMessageRecieved = 2;

		if (messageFromCar->previousAutoID != 0) car->previousCarID = messageFromCar->previousAutoID;
	}
}

cars::autocross_msg Car::initialQuestion()
{
	message.isMsgFromAuto = true;
	message.autoID = carID;
	message.direction = -1;
	message.isCrossed = false;
	return message;
}

cars::autocross_msg Car::secondQuestion()
{
	message.isMsgFromAuto = true;
	message.autoID = carID;
	message.previousCrossID = previousCrossing;
	pickDirection(message);
	message.direction = pickedDirection;
	message.isCrossed = false;
	return message;
}

cars::autocross_msg Car::lastQuestion()
{
	message.isMsgFromAuto = true;
	message.autoID = carID;
	message.isCrossed = true;
	return message;
}

void Car::getInitialSettings()
{
	carID = initialCarSettings.response.carID;
	distanceToGo = (initialCarSettings.response.pathLenght)*multiplication;
	previousCrossing = initialCarSettings.response.prevCrossing;
	nextCrossing = initialCarSettings.response.nextCrossing;
	ROS_INFO("Previous crossing ID: %d", previousCrossing);
	ROS_INFO("Next crossing ID: %d", nextCrossing);
}

void Car::infoForViz()
{
	carVizMessage.autoID = car->carID;
	carVizMessage.startCrossID = car->previousCrossing;
	carVizMessage.endCrossID = car->nextCrossing;
	carVizMessage.distance = car->distanceSoFar;
}

void Car::publicMyInfo()
{
	ROS_INFO("My ID: %d, previous crossing: %d, next crossing: %d", car->carID, car->previousCrossing, car->nextCrossing);
}

void clearMessage()
{
	message.isMsgFromAuto = true;
	message.autoID = car->carID;
	message.previousCrossID = 0;
	message.direction = -1;
	message.isCrossed = false;

	message.length = 0;
	message.nextCrossID = 0;
	message.previousAutoID = 0;
}

int main(int argc, char **argv)											//główny program
{

	ros::init(argc, argv, "car_node", ros::init_options::AnonymousName);

	car = new Car();
	ROS_INFO("A car has been created :)");

	ros::NodeHandle mapNode;
	ros::NodeHandle carCarNode;
	ros::NodeHandle vizNode;

	ros::ServiceClient carToMap = mapNode.serviceClient<::map::car_init>("init_car");
	initialCarSettings.request.req = 1;

	ros::Publisher carToVizPub = vizNode.advertise<cars::auto_viz>("auto_viz", 1000);

	while (ros::ok())
	{
		if (carToMap.call(initialCarSettings))
		{
			ROS_INFO("Service initialCarSettings successfully called!");
			break;
		}
		else
		{
			ROS_INFO("Failed to call service initialCarSettings");
		}
	}

	ROS_INFO("CarID: %d", initialCarSettings.response.carID);
	car->getInitialSettings();


	srand(time(NULL));

	std::stringstream ss;
	ss << "crossing_" << car->nextCrossing;

	ros::NodeHandle carNode;
	ros::Publisher carToCrossingPub = carNode.advertise<cars::autocross_msg>(ss.str().c_str(), 1000);
	ros::Subscriber carToCrossingSub = carNode.subscribe(ss.str().c_str(), 1000, carCallback);
	std::cout << (ss.str().c_str());

	ss.clear();
	ss.str("");
	ss << "car_to_car_" << car->carID;
	ros::ServiceServer carCarService = carCarNode.advertiseService(ss.str().c_str(), Car::carCarService);
	std::cout << "\n";
	std::cout << (ss.str().c_str());

	::cars::autocross_msg message;
	ros::Rate loop_rate(1);
	ros::Rate loop_rate_2(2);


	while (ros::ok())
	{
		car->infoForViz();
		carToVizPub.publish(carVizMessage);
		if (car->previousCarID != 0 && car->previousCarID != car->carID)
		{
			ss.clear();
			ss.str("");
			ss << "car_to_car_" << car->previousCarID;
			ros::ServiceClient carCarClient = carCarNode.serviceClient<cars::car_to_car>(ss.str().c_str());
			carCarMessage.request.carID = car->carID;
			carCarMessage.request.previousCrossing = car->previousCrossing;
			carCarMessage.request.nextCrossing = car->nextCrossing;
			ROS_INFO("My Car ID %d, previous Car ID %d", carCarMessage.request.carID, car->previousCarID);
			while (ros::ok()) {
				if (carCarClient.call(carCarMessage))
				{
					ROS_INFO("Service carCarMessage successfully called!");
					ROS_INFO("My distance %d, its distance %d", car->distanceSoFar, carCarMessage.response.distanceSoFar);
					if (carCarMessage.response.distanceSoFar - car->distanceSoFar <= 2)
						car->iCanGo = false;
					else
						car->iCanGo = true;
					if (carCarMessage.response.carID == 0)
					{
						car->previousCarID = 0;
						car->iCanGo = true;
					}
					break;
				}
				else
					ROS_INFO("Fail to call carcarMessage, the car is not responding!");
			}
		}
		if (car->distanceSoFar == 1 * multiplication)
		{
			message = car->initialQuestion();
			carToCrossingPub.publish(message);
			ROS_INFO("I've just send a message to crossing %d asking for directions", car->nextCrossing);
		}
		if (car->distanceSoFar >= car->distanceToGo - 2 * multiplication)
		{
			if (car->isMessageRecieved == 1 && car->secondMessageIsSend == false)
			{
				message = car->secondQuestion();
				carToCrossingPub.publish(message);
				ROS_INFO("I've just send a message to crossing %d with picked direction", car->nextCrossing);
				car->secondMessageIsSend = true;
			}
			else
			{
				if (car->myPatience == 0)
				{
					ROS_INFO("There's no answer from the crossing, I'm out");
					return 0;
				}
				else
				{
					car->myPatience = car->myPatience - 1;
				}
			}

		}
		if ((car->distanceSoFar < car->distanceToGo) && (car->iCanGo == true))
		{
			car->distanceSoFar += 1;
		}

		if (car->distanceSoFar == car->distanceToGo)
		{
			if (car->isMessageRecieved == 2)
			{
				loop_rate_2.sleep();
				message = car->lastQuestion();
				carToCrossingPub.publish(message);
				ROS_INFO("I've just send a message to crossing %d to say I've crossed it", car->nextCrossing);
				car->secondMessageIsSend = false;
				car->updateCar();
				car->publicMyInfo();
				car->myPatience = 15 * multiplication;

				ss.clear();
				ss.str("");
				ss << "crossing_" << car->nextCrossing;
				carToCrossingPub = carNode.advertise<cars::autocross_msg>(ss.str().c_str(), 1000);
				carToCrossingSub = carNode.subscribe(ss.str().c_str(), 1000, carCallback);
				clearMessage();
			}
		}
		ROS_INFO("Distance so far %d from crossing %d to crossing %d", car->distanceSoFar, car->previousCrossing, car->nextCrossing);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
