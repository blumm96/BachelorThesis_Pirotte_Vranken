//==============================================================================
/*
	UHAS IMPLEMENTED CLASS

	\author Casper Vranken
	\author Niels Pirotte
*/

#ifndef SPHERE_H
#define SPHERE_H

#include <vector>

namespace chai3d {

	class Sphere {
	private:
		std::vector<Sphere*> children;
	};
}
#endif