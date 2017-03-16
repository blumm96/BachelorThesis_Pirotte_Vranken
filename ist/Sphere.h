//==============================================================================
/*
	UHAS IMPLEMENTED CLASS

	\author Casper Vranken
	\author Niels Pirotte
*/

#ifndef SPHERE_H
#define SPHERE_H

<<<<<<< HEAD
=======
#include "collisions/CGenericCollision.h"
#include "math/CVector3d.h"

>>>>>>> origin/IST
#include <vector>

namespace chai3d {

<<<<<<< HEAD
	class Sphere {
	private:
		std::vector<Sphere*> children;
=======
	// Describes where the sphere is located whithin the inner sphere tree.
	enum sphereState
	{
		SPHERE_ROOT,
		SPHERE_INTERNAL,
		SPHERE_LEAF
	};

	class Sphere {

	// VARIABLES
	private:

		// The parent of this sphere. If NULL, sphere is rootsphere and localposition is equal to (0,0,0)
		Sphere* parent;
		// The children of this sphere. If the vector is empty, the sphere is a leafnode.
		std::vector<Sphere*> children;
		// The radius of the sphere.
		float radius;
		// The position of the sphere relative to the parent. If the sphere is rootsphere, the position is the global position.
		cVector3d position;
		// The state of the sphere within the innerspheretree.
		sphereState state;

	// CONCSTRUCTOR - DESTRUCTOR
	public:

		// Constructor of sphere.
		Sphere();
		// Destructor of sphere.
		~Sphere();

	// PUBLIC METHODS
	public:
		// Computes the distance between two spheres.
		float distance(Sphere* sphere, cVector3d position1, cVector3d position2);
		// Get the position relative to the parent. 
		cVector3d getPosition();
		// Get the radius of the sphere.
		float getRadius();
		// Get the children of the sphere.
		std::vector<Sphere*> getChildren();
		// Get the parentsphere of the sphere.
		Sphere* getParent();
		// Get the state of the sphere within the innerspheretree.
		sphereState getState();
>>>>>>> origin/IST
	};
}
#endif