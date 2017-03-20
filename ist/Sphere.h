//==============================================================================
/*
	UHAS IMPLEMENTED CLASS

	\author Casper Vranken
	\author Niels Pirotte
*/

#ifndef SPHERE_H
#define SPHERE_H

#include "collisions/CGenericCollision.h"
#include "math/CVector3d.h"
#include "collisions/Triangle.h"

#include <vector>

namespace chai3d {

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
		// The depth of the sphere in the innersphere tree.
		int depth;

		// The triangle that goes with this sphere.
		Triangle* triangle;

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
		// Get the depth of the sphere in the innersphere tree.
		int getDepth();
		Triangle* getTriangle();

		void setRadius(float r);
		void setPosition(cVector3d pos);
		void setState(sphereState nstate);
		void setTriangle(Triangle* setT);

		void render();
	};
}
#endif