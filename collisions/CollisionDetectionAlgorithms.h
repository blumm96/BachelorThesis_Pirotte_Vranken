//==============================================================================
/*
Implemented by Casper Vranken and Niels Pirotte commisioned by the University of Hasselt, Belgium

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
#include "math/CMaths.h"
#include <iostream>
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
	void checkDistance(cCollisionAABBNode* A, 
		cCollisionAABBNode* B, 
		double &mindist, 
		cCollisionAABB* tree_A,
		cCollisionAABB* tree_B,
		int maxdiepte, 
		int &huidigeDiepte, 
		cVector3d myLocal,
		cVector3d BLocal)
	{
		if (mindist == 0.0) return; //We only want to know if the objects are colliding or not
		if ((A->m_nodeType == cAABBNodeType::C_AABB_NODE_LEAF && B->m_nodeType == cAABBNodeType::C_AABB_NODE_LEAF) || A->m_depth == maxdiepte) {
			mindist = cMin(mindist, A->m_bbox.distance(&(B->m_bbox), myLocal, BLocal));
		}
		//recursion
		std::vector<cCollisionAABBNode> children_A = tree_A->getChildren(A);
		std::vector<cCollisionAABBNode> children_B = tree_B->getChildren(B);

		std::vector<cCollisionAABBNode>::iterator itA, itB;
		itA = children_A.begin(); itB = children_B.begin();
		for (itA; itA < children_A.end(); itA++) {
			for (itB; itB < children_B.end(); itB++) {
				cCollisionAABBNode newA = (*itA);
				cCollisionAABBNode newB = (*itB);
				if (newA.m_bbox.distance(&(newB.m_bbox), myLocal, BLocal) < mindist) {
					checkDistance(&newA, &newB, mindist, tree_A, tree_B, maxdiepte, huidigeDiepte, myLocal, BLocal);
				}
			}
		}
	}

	//Help functions
	/////////////////////////////////////////////////////////////////////////////////

	//------------------------------------------------------------------------------
} // namespace chai3d
  //------------------------------------------------------------------------------

  //------------------------------------------------------------------------------
#endif
  //------------------------------------------------------------------------------
