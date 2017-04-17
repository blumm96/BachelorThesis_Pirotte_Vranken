//==============================================================================
/*
<<<<<<< HEAD
Implemented by Casper Vranken and Niels Pirotte commisioned by the University of Hasselt, Belgium
=======
Implemented by Casper Vranken and Niels Pirotte commisioned by the University of Hasselt and KU Leuven, Belgium
>>>>>>> origin/IST

Algorithm sources: 
-new geometric datastructures for collision detection and haptics - René Weller
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CollisionDetectionAlgorithmsH
#define CollisionDetectionAlgorithmsH
//------------------------------------------------------------------------------
#include "collisions/CCollisionBasics.h"
#include "collisions/CCollisionAABB.h"
#include "collisions/CCollisionAABBTree.h"
#include "collisions/CCollisionAABBBox.h"
#include "collisions/CGenericCollision.h"
#include "ist/Sphere.h"
#include "ist/InnerSphereTree.h"
#include "math/CMaths.h"
#include <iostream>
#include <limits>

using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
	//------------------------------------------------------------------------------

	//==============================================================================
	/*!
	\file  CollisionDetectionAlgorithms.h

	\brief
	Implements the algoritms for traversing a BVH-tree
	*/
	//==============================================================================

	//==============================================================================

	//traverse an AABB tree with the following algorithm:
	//algorithm page 121 in book:new geometric datastructures for collision detection and haptics
	//////////////////////////////////////////////////////////////////////
	//Algorithm 5.2 checkDistance(A, B, minDist)
	//input : A, B = spheres in the inner sphere tree
	//	in / out : minDist = overall minimum distance seen so far
	//	if A and B are leaves then
	//		// end of recursion
	//		minDist = min{ distance(A,B), minDist }
	//	else
	//		// recursion step
	//		forall children a[i] of A do
	//		forall children b[j] of B do
	//		if distance(a[i], b[j]) < minDist then
	//			checkDistance(a[i], b[j], minDist)
	//////////////////////////////////////////////////////////////////////

	//De waarde van mindist gaat recursief worden aangepast door deze functie.
	//Dit is voor AABB
	inline void checkDistance(cCollisionAABBNode* A,
		cCollisionAABBNode* B,
		double& mindist,
		cCollisionAABB* tree_A,
		cCollisionAABB* tree_B,
		int maxdiepte,
		int &huidigeDiepte,
		cVector3d myLocal,
		cVector3d BLocal,
		cVector3d& positie)
	{
		if (mindist == 0.0) return; //We only want to know if the objects are colliding or not
		if ((A->m_nodeType == cAABBNodeType::C_AABB_NODE_LEAF && B->m_nodeType == cAABBNodeType::C_AABB_NODE_LEAF) || A->m_depth == maxdiepte) {
			mindist = cMin(mindist, A->m_bbox.distance(&(B->m_bbox), myLocal, BLocal));
			if (mindist == 0.0f) {
				positie = (A->m_bbox.getCenter() + myLocal);
			}
			//std::cout << mindist << " min" << std::endl;
		}
		else {
			//recursion
			std::vector<cCollisionAABBNode> children_A = tree_A->getChildren(A);
			std::vector<cCollisionAABBNode> children_B = tree_B->getChildren(B);

			for (int i = 0; i < children_A.size(); i++) {
				for (int j = 0; j < children_B.size(); j++) {
					cCollisionAABBNode newA = children_A[i];
					cCollisionAABBNode newB = children_B[j];
					float afstand = newA.m_bbox.distance(&(newB.m_bbox), myLocal, BLocal);
					if ((afstand == 0.0f)) checkDistance(&newA, &newB, mindist, tree_A, tree_B, maxdiepte, huidigeDiepte, myLocal, BLocal, positie);
					else mindist = cMin((float)mindist, afstand);
				}
			}
		}
		//std::cout << std::endl;
	}

	/*
		This function calculates the minimum distance between two spheres while taking into account their children.
		This is done using recursion and is used for inner sphere trees.

		\param spheraA			The first sphere to check.
		\param sphereB			The second sphere to check.
		\param tree1			The tree of the first sphere.
		\param tree2			The tree of the second sphere.
		\param maxdiepte		The maximum depth the algorithm is allowed to check.
	*/
	inline float checkDistanceSphere(
		Sphere* sphereA,
		Sphere* sphereB,
		InnerSphereTree* tree1,
		InnerSphereTree* tree2,
		int maxdiepte,
		bool& stop, 
		cVector3d& pos) {

		// Return the distance between 2 spheres when we went deeper in the tree than specified.
		// This results in worse accuracy and should ideally never be called.
		float afstand = sphereA->distance(sphereB, tree1, tree2);

		// Calculate the distance between the 2 spheres. 
		// If the distance is greater than 0, The 2 spheres do not collide and the check should finish here.
		if (afstand > 0) return afstand;

		if (sphereA->getDepth() == maxdiepte) return afstand;

		// If the 2 spheres collide and we are not at the maximum depth yet, continue the recursion.
		afstand = std::numeric_limits<float>::infinity();

		// Every sphere can have multiple children.
		std::vector<Sphere*> children_A = sphereA->getChildren();
		std::vector<Sphere*> children_B = sphereB->getChildren();

		// Iterate through all of the 2 sphere's children.

		for (int i = 0; i < children_A.size(); i++) {
			Sphere* newA = children_A[i];
			for (int j = 0; j < children_B.size(); j++) {
				Sphere* newB = children_B[j];
				
				if (stop) goto checkDistanceEinde;

				// If the children aren't leaf nodes, continue the recursion.
				if ((newA->getState() != sphereState::SPHERE_LEAF) && (newB->getState() != sphereState::SPHERE_LEAF)) {
					afstand = cMin(checkDistanceSphere(newA, newB, tree1, tree2, maxdiepte, stop, pos), afstand);
				}
				// If both children are leaf nodes, calculate the distance.
				// If the distance is 0 or smaller, a collision has occured and the distance should be returned.
				else if ((newA->getState() == sphereState::SPHERE_LEAF) || (newB->getState() == sphereState::SPHERE_LEAF)) {
					
					afstand = newA->distance(newB, tree1, tree2);
					if (afstand <= 0) {
						stop = true;
						pos = (newA->getPosition() + tree1->getPosition() + newB->getPosition() + tree2->getPosition()) / 2;
						goto checkDistanceEinde;
					}

				}
			}
		}

		checkDistanceEinde: return afstand;
	}

	inline float checkDistanceSphere2Hulp(
		Sphere* sphereA,
		Sphere* sphereB,
		InnerSphereTree* tree1,
		InnerSphereTree* tree2,
		int maxdiepte,
		bool& stop,
		cVector3d& pos,
		vector<Sphere*>* pathA,
		vector<Sphere*>* pathB,
		Sphere* excludeA,
		Sphere* excludeB) {

		// Return the distance between 2 spheres when we went deeper in the tree than specified.
		// This results in worse accuracy and should ideally never be called.
		float afstand = sphereA->distance(sphereB, tree1, tree2);

		if (sphereA->getState() == sphereState::SPHERE_LEAF && afstand == 0.0) return 0.0;

		// Calculate the distance between the 2 spheres. 
		// If the distance is greater than 0, The 2 spheres do not collide and the check should finish here.
		if (afstand > 0) return afstand;
		// if the max depth is reached we should alse not proces any futher nodes
		if (sphereA->getDepth() == maxdiepte) return afstand;

		//We detected a collision
		if ((sphereA->getState() == sphereState::SPHERE_LEAF) || (sphereB->getState() == sphereState::SPHERE_LEAF)) {
			stop = true;
			pos = (sphereA->getPosition() + tree1->getPosition());
			pos = 0.5*pos;

			return 0;
		}
		
		// If the 2 spheres collide and we are not at the maximum depth yet, continue the recursion.
		afstand = numeric_limits<float>::infinity();
		// Every sphere can have multiple children.
		std::vector<Sphere*> children_A = sphereA->getChildren();
		std::vector<Sphere*> children_B = sphereB->getChildren();

		// Iterate through all of the 2 sphere's children.

		for (int i = 0; i < children_A.size(); i++) {
			Sphere* newA = children_A[i];
			if (newA == excludeA) continue;
			pathA->push_back(newA);

			for (int j = 0; j < children_B.size(); j++) {
				Sphere* newB = children_B[j];
				if (newB == excludeB) continue;
				pathB->push_back(newB);

				if (stop) {
					goto checkDistanceSphere2HulpEinde;
				}

				// If the children aren't leaf nodes, continue the recursion.
				if ((newA->getState() != sphereState::SPHERE_LEAF) && (newB->getState() != sphereState::SPHERE_LEAF)) {
					afstand = cMin(checkDistanceSphere2Hulp(newA, newB, tree1, tree2, maxdiepte, stop, pos, pathA, pathB, nullptr, nullptr), afstand);
				}
				// If both children are leaf nodes, calculate the distance.
				// If the distance is 0 or smaller, a collision has occured and the distance should be returned.
				else if ((newA->getState() == sphereState::SPHERE_LEAF) || (newB->getState() == sphereState::SPHERE_LEAF)) {

					afstand = newA->distance(newB, tree1, tree2);
					if (afstand <= 0) {
						stop = true;
						pos = (newA->getPosition() + tree1->getPosition() + newB->getPosition() + tree2->getPosition()) / 2;

						goto checkDistanceSphere2HulpEinde;
					}

				}

				if (stop) {
					goto checkDistanceSphere2HulpEinde;
				}
				pathB->pop_back();
			} // End of iteration B
			pathA->pop_back();
		} // End of iteration A
		checkDistanceSphere2HulpEinde: return afstand;
	}


	struct comp {
		bool operator()(Sphere* s1, Sphere* s2) { return ((s1->getMindist()) < (s2->getMindist())); }
	};


	inline void checkDistanceSphere3(Sphere* A,
		Sphere* B,
		float& mindist,
		InnerSphereTree* tree_A,
		InnerSphereTree* tree_B,
		int maxdiepte)
	{
		if (mindist == 0.0) return; //We only want to know if the objects are colliding or not
		if ((A->getState() == sphereState::SPHERE_LEAF && B->getState() == sphereState::SPHERE_LEAF) || A->getDepth() == maxdiepte) {
			mindist = cMin(mindist, A->distance(B, tree_A, tree_B));
		}
		else {
			//recursion
			std::vector<Sphere*> children_A = A->getChildren();
			std::vector<Sphere*> children_B = B->getChildren();

			std::sort(children_A.begin(), children_A.end(), comp());
			std::sort(children_B.begin(), children_B.end(), comp());

			for (Sphere* A : children_A) {
				for (Sphere* B : children_B) {
					Sphere* newA = A;
					Sphere* newB = B;
					float afstand = newA->distance(newB, tree_A, tree_B);

					if (newA->getMindist() > afstand) newA->setMindist(afstand);
					if (newB->getMindist() > afstand) newB->setMindist(afstand);

					if ((afstand == 0.0f)) checkDistanceSphere3(newA, newB, mindist, tree_A, tree_B, maxdiepte);
					else mindist = cMin((float)mindist, afstand);

				}
			}

			for (int i = 0; i < children_A.size(); i++) {
				for (int j = 0; j < children_B.size(); j++) {
					children_A[i]->setMindist(std::numeric_limits<float>::infinity());
					children_B[j]->setMindist(std::numeric_limits<float>::infinity());
				}
			}
		}
	}



	inline float checkDistanceSphere2(
		Sphere* sphereA,
		Sphere* sphereB,
		InnerSphereTree* tree1,
		InnerSphereTree* tree2,
		int maxdiepte,
		bool& stop,
		cVector3d& pos,
		vector<Sphere*>* pathA,
		vector<Sphere*>* pathB,
		Sphere* excludeA, 
		Sphere* excludeB) {

		if (pathA->empty() || pathB->empty()) {
			return checkDistanceSphere2Hulp(sphereA, sphereB, tree1, tree2, maxdiepte, stop, pos, pathA, pathB, nullptr, nullptr);
		}

		float dist = checkDistanceSphere2Hulp((*pathA)[pathA->size() - 1], (*pathB)[pathB->size() - 1], tree1, tree2, maxdiepte, stop, pos, pathA, pathB, excludeA, excludeB);
		if (dist == 0) return dist;

		Sphere* newExA = (*pathA)[pathA->size() - 1];
		Sphere* newExB = (*pathB)[pathB->size() - 1];

		pathA->pop_back();
		pathB->pop_back();

		//?
		return checkDistanceSphere2(sphereA, sphereB, tree1, tree2, maxdiepte, stop, pos, pathA, pathB, newExA, newExB);
	}

	

	/*
	
		Checks the distance between 2 innersphere trees starting from specified spheres.
		This can be used to optimize our algorithm significantly.

		\param tree1		The first innersphere tree to check.
		\param tree2		The second innersphere tree to check.
		\param maxdepth		The maximum depth to check.
		\param sphereA		The sphere of the first tree to start the collision detection.
		\param sphereB		The sphere of the second tree to start the collision detection.
	
	*/
	//float checkDistanceFromSphere(
	//	InnerSphereTree* tree1,
	//	InnerSphereTree* tree2,
	//	int maxdepth,
	//	Sphere* sphereA,
	//	Sphere* sphereB, 
	//	bool& stop) {

	//	float afstand = std::numeric_limits<float>::infinity();

	//	// If the collision isn't between the 2 specified spheres,
	//	// the children of the parents should be checked.
	//	if ((sphereA->getState() != sphereState::SPHERE_ROOT) && (sphereB->getState() != sphereState::SPHERE_ROOT)) {
	//		Sphere* parentA = sphereA->getParent();
	//		Sphere* parentB = sphereB->getParent();

	//		for (int i = 0; i < parentA->getChildren().size(); i++) {
	//			for (int j = 0; j < parentB->getChildren().size(); j++) {

	//				if((sphereA != parentA->getChildren()[i] || (sphereB != parentA->getChildren()[i])))
	//				afstand = checkDistance(parentA->getChildren()[i], parentB->getChildren()[j], tree1, tree2, maxdepth, stop);

	//				if (stop) goto checkDistanceFromSphereEinde;
	//			}
	//		}

	//		checkDistanceFromSphere(tree1, tree2, maxdepth, parentA, parentB, stop);

	//	}

	//checkDistanceFromSphereEinde: return afstand;

	//}

	//Help functions
	/////////////////////////////////////////////////////////////////////////////////

	//------------------------------------------------------------------------------
} // namespace chai3d
  //------------------------------------------------------------------------------

  //------------------------------------------------------------------------------
#endif
  //------------------------------------------------------------------------------
