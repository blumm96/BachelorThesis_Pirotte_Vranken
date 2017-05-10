#include "collisions/Paths.h"
#include "ist/Sphere.h"
#include <vector>
#include <iostream>

using namespace std;

namespace chai3d {

	Paths::Paths(int aantalVrijheidsgraden) {
		this->aantalVrijheidsgraden = aantalVrijheidsgraden;
	}

	Paths::~Paths() {}

	void Paths::clearPositions()
	{
		positions.clear();
	}

	void Paths::addPosition(cVector3d pos)
	{
		positions.push_back(pos);
	}

} // chai3d