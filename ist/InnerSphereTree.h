#ifndef INNERSPHERETREE_H
#define INNERSPHERETREE_H

#include "collisions/CGenericCollision.h"
#include "ist/Sphere.h"

/*
	UHAS IMPLEMENTED

	\author Casper Vranken
	\author Niels Pirotte
*/

namespace chai3d {

	class InnerSphereTree : public cGenericCollision {

	// CONSTRUCTOR - DESTRUCTOR
	public:

		// Constructor of the inner sphere tree.
		InnerSphereTree();

		// Destructor of the inner sphere tree.
		~InnerSphereTree();

	// PUBLIC METHODS
	public:

		// Computes the collision between 2 inner sphere trees.
		virtual bool computeCollision(cGenericCollision* ist2, traversalSetting setting, double &collisionfeedback, int maxdiepte);
		// Get the type of tree. In this case IST.
		virtual inline CollisionTreeType getCollisionTreeType() { return CollisionTreeType::IST; };
		// Get the rootsphere of this inner sphere tree.
		Sphere* getRootSphere();

	// DATA MEMBERS
	private:

		// The rootsphere
		Sphere* rootSphere;

	// PROTECTED FUNCTIONS
	public:

		// This method is used to recursively build the collision tree.
		int buildTree(std::vector<Sphere*> leafs, const int a_depth);
	};
}

#endif