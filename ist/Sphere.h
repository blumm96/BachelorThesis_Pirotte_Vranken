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
#include "ist/InnerSphereTree.h"
#include "math/CVector3d.h"
#include "collisions/Triangle.h"
#include <iostream>
#include "ist/InnerSphereTree.h"

#include <vector>

using namespace std;

namespace chai3d {
	class InnerSphereTree;
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
		vector<Sphere*> children;
		// The radius of the sphere.
		float radius;
		// The position of the sphere relative to the root. If the sphere is rootsphere, the position is (0,0,0).
		cVector3d position;
		// The state of the sphere within the inner sphere tree.
		sphereState state;
		// The depth of the sphere in the inner sphere tree.
		int depth;
		// The triangle associated with this sphere.
		vector<Triangle*> triangles;

		float mindist;

		// The amount of children the shere has. This is only valid if the innersphertree is loaded from a file.
		int childrenAmount;

		//Help vector for drawing the sphere.
		vector<cVector3d*> spherePoints;

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
		float distance(Sphere* sphere, InnerSphereTree* tree1, InnerSphereTree* tree2);
		// Get the position relative to the parent. 
		cVector3d getPosition();
		// Get the position relative to the parent with the turning taken into account.
		cVector3d getPositionWithAngle(InnerSphereTree* tree1);
		// Get the radius of the sphere.
		float getRadius();
		// Get the children of the sphere.
		vector<Sphere*> getChildren();
		// Get the parentsphere of the sphere.
		Sphere* getParent();
		// Get the state of the sphere within the innerspheretree.
		sphereState getState();
		// Get the depth of the sphere in the innersphere tree.
		int getDepth();
		// Get the triangle of this sphere.
		vector<Triangle*> getTriangles();

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
			if (state != sphereState::SPHERE_ROOT) return rootSphere;
			return this;
		}

		// Set the radius of this sphere.
		void setRadius(float r);
		// Set the position of this sphere.
		void setPosition(cVector3d pos);
		// Set the state of this sphere.
		void setState(sphereState nstate);
		// Set the triangle of this sphere.
		void addTriangle(Triangle* setT);
		// Set the parent sphere of this sphere.
		void setParent(Sphere* n_parent);
		// Set the depth of this sphere within the inner sphere tree.
		void setDepth(int d);
		// Add a child to this sphere.
		void addChild(Sphere* child);

		// Help function to render this sphere.
		void make_Sphere(cVector3d center, double r, vector<cVector3d*> &spherePoints);

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

		inline float getMindist() { return mindist; };
		inline void setMindist(float n_dist) { mindist = n_dist; };

	}; // chai3d
}
#endif

		//implementations for computeCollision chai3d
	//	inline bool intersect(const cVector3d& a_segmentPointA, const cVector3d& a_segmentPointB) const
	//	{
	//		const int RIGHT = 0;
	//		const int LEFT = 1;
	//		const int MIDDLE = 2;

	//		double coord[3];
	//		char inside = true;
	//		char quadrant[3];
	//		int i;
	//		int whichPlane;
	//		double maxT[3];
	//		double candidatePlane[3];
	//		double dir[3];
	//		dir[0] = a_segmentPointB(0) - a_segmentPointA(0);
	//		dir[1] = a_segmentPointB(1) - a_segmentPointA(1);
	//		dir[2] = a_segmentPointB(2) - a_segmentPointA(2);

	//		// find candidate planes; this loop can be avoided if rays cast all from 
	//		// the eye (assume perspective view)
	//		for (i = 0; i<3; i++)
	//		{
	//			if (a_segmentPointA(i) - < m_min(i))
	//			{
	//				quadrant[i] = LEFT;
	//				candidatePlane[i] = m_min(i);
	//				inside = false;
	//			}
	//			else if (a_segmentPointA(i) > m_max(i))
	//			{
	//				quadrant[i] = RIGHT;
	//				candidatePlane[i] = m_max(i);
	//				inside = false;
	//			}
	//			else
	//			{
	//				quadrant[i] = MIDDLE;
	//			}
	//		}

	//		// ray origin inside boundary box
	//		if (inside)
	//		{
	//			return (true);
	//		}

	//		// calculate T distances to candidate planes
	//		for (i = 0; i<3; i++)
	//		{
	//			if (quadrant[i] != MIDDLE && dir[i] != 0.)
	//				maxT[i] = (candidatePlane[i] - a_segmentPointA(i)) / dir[i];
	//			else
	//				maxT[i] = -1.;
	//		}

	//		// get largest of the maxT's for final choice of intersection
	//		whichPlane = 0;
	//		for (i = 1; i<3; i++)
	//			if (maxT[whichPlane] < maxT[i])
	//				whichPlane = i;

	//		// check final candidate actually inside box
	//		if (maxT[whichPlane] < 0.)
	//		{
	//			return (false);
	//		}

	//		for (i = 0; i<3; i++)
	//		{
	//			if (whichPlane != i)
	//			{
	//				coord[i] = a_segmentPointA(i) + maxT[whichPlane] * dir[i];
	//				if (coord[i] < m_min(i) || coord[i] > m_max(i))
	//				{
	//					return (false);
	//				}
	//			}
	//			else
	//			{
	//				coord[i] = candidatePlane[i];
	//			}
	//		}

	//		// ray hits box
	//		return (true);
	//	}
	//};