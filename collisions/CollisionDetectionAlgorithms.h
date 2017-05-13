//==============================================================================
/*
Implemented by Casper Vranken and Niels Pirotte commisioned by the University of Hasselt and KU Leuven, Belgium

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
#include "collisions/Paths.h"
#include "math/CMaths.h"
#include <iostream>
#include <limits>

#define splitDepth 2
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
	//------------------------------------------------------------------------------
	//definitions
	bool checkLeafCollision(Sphere* leaf1, Sphere* leaf2, InnerSphereTree* tree1, InnerSphereTree* tree2, cVector3d &pos, float &afstand);

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
						pos = (newA->getPositionWithAngle(tree1));
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
		Sphere* excludeB,
		vector<Sphere*>* visNodesA,
		vector<Sphere*>* visNodesB, 
		bool multi) {

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
			pos = (sphereA->getPositionWithAngle(tree1));

			return 0;
		}
		
		// If the 2 spheres collide and we are not at the maximum depth yet, continue the recursion.
		afstand = numeric_limits<float>::infinity();
		// Every sphere can have multiple children.
		vector<Sphere*> children_A = sphereA->getChildren();
		vector<Sphere*> children_B = sphereB->getChildren();

		// Iterate through all of the 2 sphere's children.

		for (int i = 0; i < children_A.size(); i++) {
			Sphere* newA = children_A[i];

			bool stopA = false;

			if (excludeA == newA) {
				stopA = true;
			}
			else if(multi) {
				for (Sphere* test : *visNodesA) {
					if (test == newA) {
						stopA = true;
						break;
					}
				}
			}

			if (stopA) continue;

			pathA->push_back(newA);

			for (int j = 0; j < children_B.size(); j++) {
				Sphere* newB = children_B[j];
				
				bool stopB = false;

				if (excludeB == newB) {
					stopB = true;
				}
				/*else if (multi){
					for (Sphere* test : *visNodesB) {
						if (test == newB) {
							stopB = true;
							break;
						}
					}
				}*/
				
				if (stopB) continue;

				pathB->push_back(newB);

				if (stop) {
					goto checkDistanceSphere2HulpEinde;
				}

				// If the children aren't leaf nodes, continue the recursion.
				if ((newA->getState() != sphereState::SPHERE_LEAF) && (newB->getState() != sphereState::SPHERE_LEAF)) {
					afstand = cMin(checkDistanceSphere2Hulp(newA, newB, tree1, tree2, maxdiepte, stop, pos, pathA, pathB, nullptr, nullptr, visNodesA, visNodesB, multi), afstand);
				}
				// If both children are leaf nodes, calculate the distance.
				// If the distance is 0 or smaller, a collision has occured and the distance should be returned.
				else if ((newA->getState() == sphereState::SPHERE_LEAF) || (newB->getState() == sphereState::SPHERE_LEAF)) {

					afstand = newA->distance(newB, tree1, tree2);
					if (afstand <= 0) {
						stop = true;
						pos = (newA->getPositionWithAngle(tree1));

						goto checkDistanceSphere2HulpEinde;
					}
				}

				if (stop) {
					goto checkDistanceSphere2HulpEinde;
				}
				if (multi) {
					/*vector<Sphere*>::iterator it = visNodesB->begin();
					while(it != visNodesB->end()) {
						Sphere* test = *(it);
						if (test->isChild(pathB->back())) it = visNodesB->erase(it);
						else it++;
					}
					visNodesB->push_back(pathB->back());*/
				}
				pathB->pop_back();
			} // End of iteration B
			if (multi) {
				//vector<Sphere*>::iterator it = visNodesA->begin();
				//while(it != visNodesA->end()) {
					//Sphere* test = *(it);
					//if (test->isChild(pathA->back())) it = visNodesA->erase(it);
					//else it++;
				//}
				//visNodesA->push_back(pathA->back());
			}
			pathA->pop_back();
		} // End of iteration A
		checkDistanceSphere2HulpEinde: return afstand;
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
		Sphere* excludeB,
		vector<Sphere*>* visNodesA,
		vector<Sphere*>* visNodesB,
		bool multi) {

		if (pathA->empty() || pathB->empty()) {
			return checkDistanceSphere2Hulp(sphereA, sphereB, tree1, tree2, maxdiepte, stop, pos, pathA, pathB, nullptr, nullptr, visNodesA, visNodesB, multi);
		}

		float dist = checkDistanceSphere2Hulp((*pathA)[pathA->size() - 1], (*pathB)[pathB->size() - 1], tree1, tree2, maxdiepte, stop, pos, pathA, pathB, excludeA, excludeB, visNodesA, visNodesB, multi);

		if (dist == 0) return dist;

		Sphere* newExA = (*pathA)[pathA->size() - 1];
		Sphere* newExB = (*pathB)[pathB->size() - 1];

		pathA->pop_back();
		pathB->pop_back();

		//?
		return checkDistanceSphere2(sphereA, sphereB, tree1, tree2, maxdiepte, stop, pos, pathA, pathB, newExA, newExB, visNodesA, visNodesB, multi);
	}
	////////////////////////////////////////////////////////////////////////////////////
	inline bool checkDistanceRecursive(
		Sphere* A,
		Sphere* B,
		InnerSphereTree* tree1,
		InnerSphereTree* tree2,
		int maxdiepte,
		bool& stop,
		cVector3d& pos,
		Sphere* excludeA,
		Sphere* excludeB,
		vector<Sphere*>* visNodesA,
		vector<Sphere*>* visNodesB,
		Sphere* &raakpuntA,
		Sphere* &raakpuntB,
		float &schattingD,
		bool multi) {
		// Sanity check
		if (A == nullptr || B == nullptr) return false;
		// Return the distance between 2 spheres when we went deeper in the tree than specified.
		// This results in worse accuracy and should ideally never be called.
		float afstand = A->distance(B, tree1, tree2);

		if (afstand > schattingD) return false;
		schattingD = cMin(schattingD, afstand);

		if (A->getState() == sphereState::SPHERE_LEAF) {
			if (afstand == 0.0) {
				raakpuntA = A;
				raakpuntB = B;
				pos = A->getPositionWithAngle(tree1);
				stop = true;
				return true;
			}
		}

		// Calculate the distance between the 2 spheres. 
		// If the distance is greater than 0, The 2 spheres do not collide and the check should finish here.
		if (afstand > 0) return false;
		// if the max depth is reached we should alse not proces any futher nodes
		if (A->getDepth() == maxdiepte) return false;

		// Every sphere can have multiple children.
		vector<Sphere*> children_A = A->getChildren();
		vector<Sphere*> children_B = B->getChildren();

		// Iterate through all of the 2 sphere's children.

		for (int i = 0; i < children_A.size(); i++) {
			Sphere* newA = children_A[i];
			bool stopA = false;

			if (excludeA == newA) {
				continue;
			}
			else if (multi && (newA->getDepth() == splitDepth)) {
				for (int k = 0; k < visNodesA->size(); k++) {
					if ((*visNodesA)[k] == newA) {
						stopA = true;
						break;
					}
				}
			}
			if (stopA) continue;

			for (int j = 0; j < children_B.size(); j++) {
				Sphere* newB = children_B[j];
				bool stopB = false;

				if (excludeB == newB) {
					continue;
				}
				else if (multi && (newB->getDepth() == splitDepth)) {
					for (int k = 0; k < visNodesB->size(); k++) {
						if ((*visNodesB)[k] == newB) {
							stopB = true;
							break;
						}
					}
				}
				if (stopB) continue;
				
				//check the children recursive
				checkDistanceRecursive(newA, newB, tree1, tree2, maxdiepte, stop, pos, nullptr, nullptr, visNodesA, visNodesB, raakpuntA, raakpuntB, schattingD, multi);
				if (stop) {
					return true;
				}
			} // End of iteration B
		} // End of iteration A
		raakpuntA = nullptr;
		raakpuntB = nullptr;
		return false;
	}

	inline bool checkDistanceBacktrack(
		Sphere* A,
		Sphere* B,
		InnerSphereTree* tree1,
		InnerSphereTree* tree2,
		int maxdiepte,
		bool& stop,
		cVector3d& pos,
		vector<Sphere*>* visNodesA,
		vector<Sphere*>* visNodesB,
		Sphere* &raakpuntA,
		Sphere* &raakpuntB,
		float &schattingD) {

		if (A == nullptr || B == nullptr) {
			return checkDistanceRecursive(tree1->getRootSphere(), tree2->getRootSphere(), tree1, tree2, maxdiepte, stop, pos, nullptr, nullptr, visNodesA, visNodesB, raakpuntA, raakpuntB, schattingD, true);
		}

		//schattingD = std::numeric_limits<float>::infinity();
		bool raakt = checkDistanceRecursive(A->getParent(), B->getParent(), tree1, tree2, maxdiepte, stop, pos, A, B, visNodesA, visNodesB, raakpuntA, raakpuntB, schattingD, true);
		if (!raakt) {
			if ((A->getDepth() == splitDepth) && (B->getDepth() == splitDepth)) {
				visNodesA->push_back(A);
				visNodesB->push_back(B);
			}
			return checkDistanceBacktrack(A->getParent(), B->getParent(), tree1, tree2, maxdiepte, stop, pos, visNodesA, visNodesB, raakpuntA, raakpuntB, schattingD);
		}
		//we found a collision
		return true;
	}
	////////////////////////////////////////////////////////////////////////////////////

	inline bool checkDistanceSphereMultipoint(
		InnerSphereTree* tree1,
		InnerSphereTree* tree2,
		int maxdiepte,
		Paths paths,
		float &schattingD) {

		//init
		if (paths.raakpuntenA.size() != paths.raakpuntenB.size()) cout << "error: unequal touch points" << endl;
		int collisions = paths.raakpuntenA.size(); //initPaths = aantal collisiepaden opgeslagen
		vector<Sphere*> retraverseA;
		vector<Sphere*> retraverseB;

		//clear alle berekende posities
		paths.clearPositions();
		float d;

		//check als leafs van deze paden nog raken
		//erase all points not colliding
		vector<Sphere*>::iterator itA = paths.raakpuntenA.begin();
		vector<Sphere*>::iterator itB = paths.raakpuntenB.begin();
		vector<Sphere*> excludesA;
		vector<Sphere*> excludesB;

		while (itA != paths.raakpuntenA.end()) {
			Sphere* checkA = *(itA);
			Sphere* checkB = *(itB);
			if ((checkA->getState() != sphereState::SPHERE_LEAF) || (checkB->getState() != sphereState::SPHERE_LEAF)) cout << "error: unexpected collision check of non leaf spheres" << endl;
				
			cVector3d pos;
			if (checkLeafCollision(checkA, checkB, tree1, tree2, pos, d)) {
				//deze paden moeten niet meer worden bekeken
				excludesA.push_back(checkA->getParent(splitDepth));
				excludesB.push_back(checkB->getParent(splitDepth));
				paths.addPosition(pos);
			}
			else {
				//deze paden moeten wel worden gebruikt om een nieuwe collisie te zoeken
				retraverseA.push_back(checkA);
				retraverseB.push_back(checkB);
				itA = paths.raakpuntenA.erase(itA);
				itB = paths.raakpuntenB.erase(itB);
				collisions--;
				continue;
			}
			schattingD = cMin(schattingD, d);
		itA++;
		itB++;
		}

		if (collisions == paths.getDegreesFreedom()) {
			return true;
		}

		//gebruik vorige paden die niet raken om terug te zoeken in de boom
		vector<Sphere*> vorigeAs;
		vector<Sphere*> vorigeBs;
		for (int i = 0; i < retraverseA.size(); i++) {
			//tegelijkertijd worden van de paden de raakpunten geset en ze worden toegevoegd aan  excludedPaths
			Sphere* checkA = retraverseA[i];
			Sphere* checkB = retraverseB[i];

			for (int z = 0; z < vorigeAs.size(); z++) {
				if (vorigeAs[z] == checkA->getParent(splitDepth) || vorigeBs[z] == checkB->getParent(splitDepth)) {
					checkA = vorigeAs[z]->getParent();
					checkB = vorigeBs[z]->getParent();
					break;
				}
			}
			
			d = std::numeric_limits<float>::infinity();

			Sphere* raaktA = nullptr;
			Sphere* raaktB = nullptr;
			cVector3d pos;
			bool stop = false;
			if (checkDistanceBacktrack(checkA, checkB, tree1, tree2, maxdiepte, stop, pos, &excludesA, &excludesB, raaktA, raaktB, d)) {
				if (raaktA == nullptr || raaktB == nullptr) cout << "error: collisie maar paden niet geset!" << endl;
				paths.raakpuntenA.push_back(raaktA);
				paths.raakpuntenB.push_back(raaktB);

				schattingD = 0;

				paths.addPosition(pos);
				collisions++;
				if (collisions == paths.getDegreesFreedom()) return true;

				vorigeAs.push_back(raaktA->getParent(splitDepth));
				vorigeBs.push_back(raaktB->getParent(splitDepth));
			}
			else {
				schattingD = cMin(schattingD, d);
				return false;
			}
		}

		//probeer nieuwe paden te vinden
		int cs = collisions;
		for (int i = 0; i < paths.getDegreesFreedom() - cs; i++) {
			Sphere* findA = nullptr;
			Sphere* findB = nullptr;
			cVector3d pos;
			d = std::numeric_limits<float>::infinity();
			bool stop = false;
			if (checkDistanceBacktrack(nullptr, nullptr, tree1, tree2, maxdiepte, stop, pos, &excludesA, &excludesB, findA, findB, d)) {
				if (findA == nullptr || findB == nullptr) {
					cout << "error: collisie maar paden niet geset!" << endl;
					break;
				}
				paths.raakpuntenA.push_back(findA);
				paths.raakpuntenB.push_back(findB);

				schattingD = 0;
				
				paths.addPosition(pos);
				collisions++;

				//cout << "collisie: sphereA: " << findA << " - sphereB: " << findB << " - pos: " << pos << endl;
				//cout << "size of raakpunten: " << paths.raakpuntenA.size() << " - number of pos: " << paths.getPositions().size() << endl;
				if (collisions == paths.getDegreesFreedom()) return true;

				excludesA.push_back(findA->getParent(splitDepth));
				excludesB.push_back(findB->getParent(splitDepth));
			}
			else {
				schattingD = cMin(schattingD, d);
				break;
			}
		}
		if (collisions > 0) return true;
		return false;
	}

	

	//Help functions
	/////////////////////////////////////////////////////////////////////////////////

	struct comp {
		bool operator()(Sphere* s1, Sphere* s2) { return ((s1->getMindist()) < (s2->getMindist())); }
	};

	inline bool checkLeafCollision(Sphere* leaf1, Sphere* leaf2, InnerSphereTree* tree1, InnerSphereTree* tree2, cVector3d &pos, float &afstand) {

		float d = leaf1->distance(leaf2, tree1, tree2);
		if (d <= 0.0) {
			pos = leaf1->getPositionWithAngle(tree1);
			afstand = d;
			return true;
		}
		return false;
	}

	//EXTRA
	/////////////////////////////////////////////////////////////////////////////////
	//This algorithm tries to create a speed up by first checking the spheres which were closest in a previous collision detection
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


	inline void checkDistanceSphereTest(
		Sphere* sphereA,
		Sphere* sphereB,
		InnerSphereTree* tree1,
		InnerSphereTree* tree2,
		float &mindist,
		int maxdiepte,
		bool& stop,
		cVector3d& pos,
		Sphere* &pA,
		Sphere* &pB,
		int &test)
	{	
		test++;
		if (mindist == 0.0) return; //We only want to know if the objects are colliding or not
		if ((sphereA->getState() == sphereState::SPHERE_LEAF) && (sphereB->getState() == sphereState::SPHERE_LEAF) || sphereA->getDepth() == maxdiepte) {
			float testD = cMin((float)mindist, sphereA->distance(sphereB, tree1, tree2));
			if (testD != mindist) {
				pA = sphereA;
				pB = sphereB;
				mindist = testD;
				//pos = (sphereA->getPositionWithAngle(tree1));
			}
			//std::cout << mindist << " min" << std::endl;
		}
		else {
			//recursion
			std::vector<Sphere*> children_A = sphereA->getChildren();
			std::vector<Sphere*> children_B = sphereB->getChildren();

			for (int i = 0; i < children_A.size(); i++) {
				for (int j = 0; j < children_B.size(); j++) {
					Sphere* newA = children_A[i];
					Sphere* newB = children_B[j];
					float afstand = newA->distance(newB, tree1, tree2);
					//if (afstand != 0.0f) continue;
					if (afstand < mindist) checkDistanceSphereTest(newA, newB, tree1, tree2, mindist, maxdiepte, stop, pos, pA, pB, test);
					//if ((afstand == 0.0f)) checkDistanceSphereTest(newA, newB, tree1, tree2, mindist, maxdiepte, stop, pos);
					//else mindist = cMin((float)mindist, afstand);
				}
			}
		}
		//std::cout << std::endl;
	}

	inline void accurateCheck(
		InnerSphereTree *tree1,
		InnerSphereTree *tree2,
		Sphere *sphere1,
		Sphere *sphere2,
		float &kortste,
		float &langste
	) {

		if (sphere1->getState() == sphereState::SPHERE_LEAF) {
			float kafstand = sphere1->distance(sphere2, tree1, tree2);
			float lafstand = kafstand + sphere1->getRadius() * 2 + sphere2->getRadius() * 2;
			if (kafstand < kortste) kortste = kafstand;
			if (lafstand > langste) langste = lafstand;
		}
		else {
			for (unsigned int i = 0; i < sphere1->getChildren().size(); i++) {
				for (unsigned int j = 0; j < sphere2->getChildren().size(); j++) {

					accurateCheck(tree1, tree2, sphere1->getChildren()[i], sphere2->getChildren()[j], kortste, langste);

				}
			}
		}
	}

	//------------------------------------------------------------------------------
} // namespace chai3d
  //------------------------------------------------------------------------------

  //------------------------------------------------------------------------------
#endif
  //------------------------------------------------------------------------------
