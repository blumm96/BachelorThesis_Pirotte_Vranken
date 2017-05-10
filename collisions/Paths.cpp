//==============================================================================
/*
This class provides feedback for the multipoint algotihm.

\author    Niels Pirotte
\author    Casper Vranken
\version   1.0.0
*/
//==============================================================================

#include "collisions/Paths.h"
#include "ist/Sphere.h"
#include <vector>
#include <iostream>

using namespace std;

namespace chai3d {

	/*
	* Constructor for the class paths
	*/
	Paths::Paths(int aantalVrijheidsgraden) {
		this->aantalVrijheidsgraden = aantalVrijheidsgraden;
	}

	/*
	* Detructor for the class paths
	*/
	Paths::~Paths() {}

	/*
	* Clear all contact points stored in the positions vector
	*/
	void Paths::clearPositions()
	{
		positions.clear();
	}

	/*
	* Add a contact point to the positions vector
	*
	* \param pos contact point to add
	*/
	void Paths::addPosition(cVector3d pos)
	{
		positions.push_back(pos);
	}

} // chai3d