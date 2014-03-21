#ifndef _INCLUDED_SIMULATOR_H
#define _INCLUDED_SIMULATOR_H

#include "Racetrack.h"
#include "Car.h"

class Simulator {

public:

	Simulator(Racetrack r);

private:

	Racetrack racetrack;
	Car my_car;


};

#endif