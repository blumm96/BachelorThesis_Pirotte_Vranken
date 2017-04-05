//==============================================================================
/*
	UHAS IMPLEMENTED CLASS

	This class represents a Sphere within an inner sphere tree.

	\author Casper Vranken
	\author Niels Pirotte
*/

#ifndef SPHERE_H
#define SPHERE_H

#include "collisions/CGenericCollision.h"
#include "math/CVector3d.h"
#include "collisions/Triangle.h"
#include <iostream>

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
		// The position of the sphere relative to the root. If the sphere is rootsphere, the position is (0,0,0).
		cVector3d position;
		// The state of the sphere within the inner sphere tree.
		sphereState state;
		// The depth of the sphere in the inner sphere tree.
		int depth;
		// The triangle associated with this sphere.
		Triangle *triangle;

		// The amount of children the shere has. This is only valid if the innersphertree is loaded from a file.
		int childrenAmount;

		//Help vector for drawing the sphere.
		std::vector<cVector3d*> spherePoints;

		// The rootsphere of this sphere.
		Sphere* rootSphere;

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
		// Get th triangle of this sphere.
		Triangle* getTriangle();

		/*
			
			Set the root for this sphere.

			\param r The root to be set.

		*/
		inline void setRootSphere(Sphere* r) { rootSphere = r; }

		/*
			
			Returns the root for this sphere.

			\return The root associated with this sphere.

		*/
		inline Sphere* getRootSphere() { 
			if(state != sphereState::SPHERE_ROOT) return rootSphere; 
			return this;
		}

		// Set the radius of this sphere.
		void setRadius(float r);
		// Set the position of this sphere.
		void setPosition(cVector3d pos);
		// Set the state of this sphere.
		void setState(sphereState nstate);
		// Set the triangle of this sphere.
		void setTriangle(Triangle* setT);
		// Set the parent sphere of this sphere.
		void setParent(Sphere* n_parent);
		// Set the depth of this sphere within the inner sphere tree.
		void setDepth(int d);
		// Add a child to this sphere.
		void addChild(Sphere* child);

		// Help function to render this sphere.
		void make_Sphere(cVector3d center, double r, std::vector<cVector3d*> &spherePoints);

		/*
			
			This is a help function to render the sphere.

		*/
		inline void initRender() { 

			if (spherePoints.empty()) { 
				make_Sphere(position, radius, spherePoints); 
			}

		}

		// Render the sphere.
		void render();

		/*
			
			Set the amount of children this sphere has.
			This function should only be used when loading an inner sphere tree.

			\param n_amount The amount of children this sphere has.

		*/
		inline void setChildrenAmount(int n_amount) { childrenAmount = n_amount; }

		/*
			
			Get the amount of children this sphere has.
			
			\return The amount of children this sphere has.

		*/
		inline int getChildrenAmount() { return childrenAmount; }
	};
} // chai3d
#endif