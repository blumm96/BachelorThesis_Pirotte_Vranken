//==============================================================================
/*
Software License Agreement (BSD License)
Copyright (c) 2003-2016, CHAI3D.
(www.chai3d.org)

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above
copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided
with the distribution.

* Neither the name of CHAI3D nor the names of its contributors may
be used to endorse or promote products derived from this software
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

\author    Niels Pirotte
\author    Casper Vranken
\version   1.0.0
*/
//==============================================================================

//+++++++++++++UHAS IMPLEMENTED+++++++++++++++++++++++++++++++++++++++++++++++++

//------------------------------------------------------------------------------
#include "math/CVector3d.h"
#include "collisions/CCollisionAABB.h"
#include "collisions/Voxel.h"
#include "collisions/Voxelizer.h"
#include "math/CMaths.h"
#include "Triangle.h"
#include "ist/Sphere.h"
#include "ist/InnerSphereTree.h"
//------------------------------------------------------------------------------
#include <vector>
#include <list>
#include <limits>
//------------------------------------------------------------------------------

using namespace std;
//------------------------------------------------------------------------------
namespace chai3d {
	//------------------------------------------------------------------------------

	//==============================================================================
	/*!
	\file       Voxelizer.cpp

	\brief
	Implements the voxelizer algorithm by Morris specified in his paper:
	Algorithms and Data Structures for Haptic Rendering: Curve Constraints, Distance Maps, and Data Logging.
	By Dan Morris

	How to use this class:
		1. create a voxelizer with the constructor
		2. set the object which needs to be voxelized (setObject())
		3. map from the voxel to the distances to each voxel (mapDistances())
		4. BuildInnerTree
		5. delete the voxelizer
	*/
	//==============================================================================

	//==============================================================================
	/*!
	\class      Voxelizer
	\ingroup    collisions

	*/
	//==============================================================================
		//--------------------------------------------------------------------------
		// CONSTRUCTOR & DESTRUCTOR:
		//--------------------------------------------------------------------------

		//! Constructor of Voxelizer.
		Voxelizer::Voxelizer() {
			initialize();
		}

		//! Destructor of Voxelizer.
		Voxelizer::~Voxelizer() {
			for (int i = 0; i < voxels->size(); i++) {
				delete(&voxels[i]);
			}
			delete(closest_point);
		}

		void Voxelizer::setObject(cCollisionAABB* c){
			object = c;
		}

		//--------------------------------------------------------------------------
		// METHODS:
		//--------------------------------------------------------------------------

		//INITIALIZATIE VAN DE VOXELIZER
		void Voxelizer::initialize() {
			vector<cCollisionAABBNode> object_nodes = object->getNodes();
			int root_index = object->getRoot();
			voxels = object->maakVoxels();
		}

		double Voxelizer::distance(Voxel* v1, Voxel* v2) {
			return (*(v1->getPos()) - *(v2->getPos())).length();
		}

		//most important method of this class
		//creates the inne
		void Voxelizer::mapDistances() {
			// Process each voxel on our list, one
			// at a time...

			std::vector<Voxel*>::iterator iter = (*voxels).begin();
			while (iter != (*voxels).end()) {
				// Grab the next voxel
				Voxel* v = (*iter);

				// Now we’re going to find the closest 
				// point in the tree (tree_root) to v... 

				find_closest_point(v);

				// Now output or do something useful 
				// with lowest_dist_sq and closest_point 
				// these are the values that should be 
				// associated with v in our output => distance map
				
				map_distance_to_voxel(v); 
				
				// Exploit spatial coherence by giving the next voxel 
				// a "hint" about where to start looking in the tree. 
				// it seeds 'boxes_to_descend' with a good starting point for the next voxel. 
				
				//seed_next_voxel_search();

			}

		}

		InnerSphereTree* Voxelizer::buildInnerTree(int diepte)
		{
			//build inner sphere structure

			/* paper: Inner Sphere Trees for Proximity and Penetration Queries
			In this section we describe the construction of our data
			structure.In a fist step, we want a watertight object to be
			densely filled with a set of non - overlapping spheres.The
			volume of the object should be approximated well by the
			spheres, while their number should be small.In a second step,
			we create a hierarchy over this set of spheres.*/

			priorityList.sort();

			//check the comparison
			for (int i = 0; i < priorityList.size(); i++) {
				std::cout << priorityList.front()->getMinDist() << " - ";
				priorityList.pop_front();
			}

			//Then, all
			//voxels whose centers are contained in this sphere are deleted
			//from the p - queue
			std::vector<Sphere*> innerspheres;

			while (!priorityList.empty()) {
				Voxel* v = priorityList.front();
				priorityList.pop_front();

				Sphere* s = new Sphere();
				s->setPosition(*(v->getPos()));
				s->setRadius(v->getMinDist());
				s->setState(sphereState::SPHERE_LEAF);
				s->setTriangle(closest_triangle);
				//Hier moet nog worden geset als de sphere aan de buitenkant van het object ligt of niet

				innerspheres.push_back(s);

				//delete all voxels contained in the sphere
				for (auto i = priorityList.begin(); i != priorityList.end();) {
					Voxel* check = *(i);
					if (distance(v,check)<v->getMinDist())
						i = priorityList.erase(i);
					else
						++i;
				}

				//update all other distances
				/*we have to update all voxels
				Vi with di < d and distance d(Vi; V ) < 2d.This is because
				they are now closer to the sphere around V  than to a triangle
				on the hull.Their di must now be set to the
				new free radius.*/
				
				list<Voxel*>::iterator it = priorityList.begin();
				for (it; it != priorityList.end(); it++) {
					Voxel* check = *(it);
					double dist = distance(v, check);
					if (dist < (2 * v->getMinDist())) check->setMinDist(dist - (v->getMinDist()));
				}
			}
			
			//IST hierarchy
			InnerSphereTree* tree = new InnerSphereTree();
			tree->buildTree(innerspheres, diepte);
			return tree;
		}

		// Find the closest point in our mesh to the sample point v
		void Voxelizer::find_closest_point(Voxel* v) {

			// Start with the root of the tree 
			toDescend.push_back(&object_nodes[root_index]);

			while (!toDescend.empty()) { 
				cCollisionAABBNode* node = toDescend.front();
				toDescend.pop_front();
				//Process each node
				process_node(node, v); 
			}
		}

		
		void Voxelizer::map_distance_to_voxel(Voxel* v) {
			v->setMinDist(sqrt(low_dist_sq));
			v->setTriangle(closest_triangle);
			priorityList.push_front(v);
		}
		
		//moet nog worden geïmplementeerd indien hier nog tijd voor gaat zijn

		//void Voxelizer::seed_next_voxel_search() {
		//	// Start at the node that contained our
		//	// closest point and walk a few levels
		//	// up the tree.
		//	cCollisionAABBNode* seed_node = closest_point_node;
		//	for (int i = 0; i<TREE_ASCEND_N; i++) {
		//		if (seed_node->m_depth == 0) break;
		//		else seed_node = object_nodes[seed_node->];
		//	}
		//	// Put this seed node on the search list
		//	// to be processed with the next voxel.
		//	toDescend.push_back(seed_node);
		//}

		// Examine the given node and decide whether 
		// we can discard it or whether we need to 
		// visit his children. If it’s a leaf, 
		// compute an actual distance and store 
		// it if it’s the closest so far.
		void Voxelizer::process_node(cCollisionAABBNode* n, Voxel* v) {
			//is the current node a leaf of the tree?
			bool is_leaf = false;
			if (n->m_nodeType == C_AABB_NODE_LEAF) is_leaf = true;


			if (is_leaf) {
				// compute the distance to this triangle
				float d_sq;
				//closest point on the triangle
				cVector3d* closest_pt_on_triangle;

				//find the closest point on the triangle
				closest_point_triangle(v, n->m_bbox.triangle, d_sq, closest_pt_on_triangle);

				// Is this the shortest distance so far? 
				if (d_sq < low_dist_sq) {
					// Mark him as the closest we’ve seen 
					low_dist_sq = d_sq;
					closest_point = closest_pt_on_triangle;
					closest_point_node = n;
					closest_triangle = n->m_bbox.triangle;

					// Also mark him as the "lowest upper bound", because any future boxes 
					// whose lower bound is greater than this value should be discarded. 
					lowest_upper_dist_sq = d_sq;
				}

				// This was a leaf and we’re done with him whether he was useful or not.
				return;
			}
			//if this node was not a leaf
			// Computing lower- and upper-bound distances to an axis-aligned bounding 
			// box is extremely fast; we just take the farthest plane on each axis 
			float best_dist_x = 0;
			float worst_dist_x = 0;

			// If I'm below the x range, my lowest x distance uses the minimum x, and 
			// my highest uses the maximum x 
			if (v->getPos()->x() < n->m_bbox.getLowerX()) {
				best_dist_x += n->m_bbox.getLowerX() - v->getPos()->x();
				worst_dist_x += n->m_bbox.getUpperX() - v->getPos()->x();
			}

			// If I'm above the x range, my lowest x 
			// distance uses the maximum x, and my 
			// highest uses the minimum x 
			else if (v->getPos()->x() < n->m_bbox.getUpperX()) {
				best_dist_x += v->getPos()->x() - n->m_bbox.getUpperX();
				worst_dist_x += v->getPos()->x() - n->m_bbox.getLowerX();
			}

			// If I'm _in_ the x range, x doesn't affect my lowest distance, and my 
			// highest-case distance goes to the _farther_ of the two x distances 
			else {
				float dmin = fabs(n->m_bbox.getLowerX() - v->getPos()->x());
				float dmax = fabs(n->m_bbox.getUpperX() - v->getPos()->x());
				double d_worst = (dmin > dmax) ? dmin : dmax;
				worst_dist_x += d_worst;
			}

			//repeat for y
			float best_dist_y = 0;
			float worst_dist_y = 0;

			if (v->getPos()->y() < n->m_bbox.getLowerY()) {
				best_dist_y += n->m_bbox.getLowerY() - v->getPos()->y();
				worst_dist_y += n->m_bbox.getUpperY() - v->getPos()->y();
			}

			else if (v->getPos()->y() < n->m_bbox.getUpperY()) {
				best_dist_y += v->getPos()->y() - n->m_bbox.getUpperY();
				worst_dist_y += v->getPos()->y() - n->m_bbox.getLowerY();
			}

			else {
				float dmin = fabs(n->m_bbox.getLowerY() - v->getPos()->y());
				float dmax = fabs(n->m_bbox.getUpperY() - v->getPos()->y());
				double d_worst = (dmin > dmax) ? dmin : dmax;
				worst_dist_y += d_worst;
			}

			//repeat for z
			float best_dist_z = 0;
			float worst_dist_z = 0;

			if (v->getPos()->z() < n->m_bbox.getLowerZ()) {
				best_dist_z += n->m_bbox.getLowerZ() - v->getPos()->z();
				worst_dist_z += n->m_bbox.getUpperZ() - v->getPos()->z();
			}

			else if (v->getPos()->z() < n->m_bbox.getUpperZ()) {
				best_dist_z += v->getPos()->z() - n->m_bbox.getUpperZ();
				worst_dist_z += v->getPos()->z() - n->m_bbox.getLowerZ();
			}

			else {
				float dmin = fabs(n->m_bbox.getLowerZ() - v->getPos()->z());
				float dmax = fabs(n->m_bbox.getUpperZ() - v->getPos()->z());
				double d_worst = (dmin > dmax) ? dmin : dmax;
				worst_dist_z += d_worst;
			}

			// Convert to squared distances
			float lower_dsq = best_dist_x * best_dist_x + best_dist_y * best_dist_y + best_dist_z * best_dist_z;
			float upper_dsq = worst_dist_x * worst_dist_x + worst_dist_y * worst_dist_y + worst_dist_z * worst_dist_z;

			// If his lower-bound squared distance
			// is greater than lowest_upper_dist_sq,
			// he can’t possibly hold the closest
			// point, so we can discard this box and
			// his children.
			if (lower_dsq > lowest_upper_dist_sq) return;

			// Check whether I’m the lowest
			// upper-bound that we’ve seen so far,
			// so we can later prune away
			// non-candidate boxes.
			if (upper_dsq < lowest_upper_dist_sq) {
				lowest_upper_dist_sq = upper_dsq;
			}

			// If this node _could_ contain the
			// closest point, we need to process his
			// children.
			//
			// Since we pop new nodes from the front
			// of the list, pushing nodes to the front
			// here results in a depth-first search,
			// and pushing nodes to the back here
			// results in a breadth-first search. A
			// more formal analysis of this tradeoff
			// will follow in section 3.4.
			toDescend.push_front(&object_nodes[n->m_leftSubTree]);
			toDescend.push_front(&object_nodes[n->m_rightSubTree]);
		}

		//find the closes point to a triangle
		void Voxelizer::closest_point_triangle(Voxel * v, Triangle * t, float & dst, cVector3d * closest_point)
		{
			// taken from
			// http://www.geometrictools.com/Foundation/Distance/Wm3DistVector3Triangle3.cpp
			//
			// Geometric Tools, Inc.
			// http://www.geometrictools.com
			// Copyright (c) 1998-2006. All Rights Reserved
			//
			// The Wild Magic Library (WM3) source code is supplied under the terms of
			// the license agreement
			// http://www.geometrictools.com/License/WildMagic3License.pdf
			// and may not be copied or disclosed except in accordance with the terms
			// of that agreement.
			
			dst = nearestpoint(t->p1, t->p2, t->p3, v->getPos(), closest_point);
			
		}


		/**
		* Finds the nearest distance between a point and a triangle.
		*
		* @param v0
		* The first vertice.
		* @param v1
		* The second vertice.
		* @param v2
		* The third vertice.
		* @param p
		* The point to measure from.
		* @param closest
		* The closest point to p on the triangle.
		* @param uv
		* The barycentric coordinates of the nearest point where u and v are the
		* weights for vertices 1 and 2 respectively.
		* @return The distance from point p to the nearest point on triangle v0, v1
		* and v2.
		*/
		double Voxelizer::nearestpoint(
			cVector3d* v0, cVector3d* v1, cVector3d* v2, cVector3d* p, cVector3d* cl_point)
		{
			//uv is eigenlijk een 2D vector

			cVector3d* kDiff = new cVector3d();
			cVector3d* kEdge0 = new cVector3d();
			cVector3d* kEdge1 = new cVector3d();

			cVector3d* closest = new cVector3d();
			
			kDiff->sub(*v0);
			kDiff->sub(*p);

			kEdge0->sub(*v1);
			kEdge0->sub(*v0);

			kEdge1->sub(*v2);
			kEdge1->sub(*v0);

			double fA00 = kEdge0->lengthsq();
			double fA01 = kEdge0->dot(*kEdge1);
			double fA11 = kEdge1->lengthsq();
			double fB0 = kDiff->dot(*kEdge0);
			double fB1 = kDiff->dot(*kEdge1);
			double fC = kDiff->lengthsq();
			double fDet = abs(fA00 * fA11 - fA01 * fA01);
			double fS = fA01 * fB1 - fA11 * fB0;
			double fT = fA01 * fB0 - fA00 * fB1;
			double fSqrDistance;

			if (fS + fT <= fDet) {
				if (fS < (double)0.0) {
					if (fT < (double)0.0) // region 4
					{
						if (fB0 < (double)0.0) {
							fT = (double)0.0;
							if (-fB0 >= fA00) {
								fS = (double)1.0;
								fSqrDistance = fA00 + ((double)2.0) * fB0 + fC;
							}
							else {
								fS = -fB0 / fA00;
								fSqrDistance = fB0 * fS + fC;
							}
						}
						else {
							fS = (double)0.0;
							if (fB1 >= (double)0.0) {
								fT = (double)0.0;
								fSqrDistance = fC;
							}
							else if (-fB1 >= fA11) {
								fT = (double)1.0;
								fSqrDistance = fA11 + ((double)2.0) * fB1 + fC;
							}
							else {
								fT = -fB1 / fA11;
								fSqrDistance = fB1 * fT + fC;
							}
						}
					}
					else
						// region 3
					{
						fS = (double)0.0;
						if (fB1 >= (double)0.0) {
							fT = (double)0.0;
							fSqrDistance = fC;
						}
						else if (-fB1 >= fA11) {
							fT = (double)1.0;
							fSqrDistance = fA11 + ((double)2.0) * fB1 + fC;
						}
						else {
							fT = -fB1 / fA11;
							fSqrDistance = fB1 * fT + fC;
						}
					}
				}
				else if (fT < (double)0.0) // region 5
				{
					fT = (double)0.0;
					if (fB0 >= (double)0.0) {
						fS = (double)0.0;
						fSqrDistance = fC;
					}
					else if (-fB0 >= fA00) {
						fS = (double)1.0;
						fSqrDistance = fA00 + ((double)2.0) * fB0 + fC;
					}
					else {
						fS = -fB0 / fA00;
						fSqrDistance = fB0 * fS + fC;
					}
				}
				else
					// region 0
				{
					// minimum at interior point
					double fInvDet = ((double)1.0) / fDet;
					fS *= fInvDet;
					fT *= fInvDet;
					fSqrDistance =
						fS * (fA00 * fS + fA01 * fT + ((double)2.0) * fB0) + fT
						* (fA01 * fS + fA11 * fT + ((double)2.0) * fB1) + fC;
				}
			}
			else {
				double fTmp0, fTmp1, fNumer, fDenom;

				if (fS < (double)0.0) // region 2
				{
					fTmp0 = fA01 + fB0;
					fTmp1 = fA11 + fB1;
					if (fTmp1 > fTmp0) {
						fNumer = fTmp1 - fTmp0;
						fDenom = fA00 - 2.0f * fA01 + fA11;
						if (fNumer >= fDenom) {
							fS = (double)1.0;
							fT = (double)0.0;
							fSqrDistance = fA00 + ((double)2.0) * fB0 + fC;
						}
						else {
							fS = fNumer / fDenom;
							fT = (double)1.0 - fS;
							fSqrDistance =
								fS * (fA00 * fS + fA01 * fT + 2.0f * fB0) + fT
								* (fA01 * fS + fA11 * fT + ((double)2.0) * fB1) + fC;
						}
					}
					else {
						fS = (double)0.0;
						if (fTmp1 <= (double)0.0) {
							fT = (double)1.0;
							fSqrDistance = fA11 + ((double)2.0) * fB1 + fC;
						}
						else if (fB1 >= (double)0.0) {
							fT = (double)0.0;
							fSqrDistance = fC;
						}
						else {
							fT = -fB1 / fA11;
							fSqrDistance = fB1 * fT + fC;
						}
					}
				}
				else if (fT < (double)0.0) // region 6
				{
					fTmp0 = fA01 + fB1;
					fTmp1 = fA00 + fB0;
					if (fTmp1 > fTmp0) {
						fNumer = fTmp1 - fTmp0;
						fDenom = fA00 - ((double)2.0) * fA01 + fA11;
						if (fNumer >= fDenom) {
							fT = (double)1.0;
							fS = (double)0.0;
							fSqrDistance = fA11 + ((double)2.0) * fB1 + fC;
						}
						else {
							fT = fNumer / fDenom;
							fS = (double)1.0 - fT;
							fSqrDistance =
								fS * (fA00 * fS + fA01 * fT + ((double)2.0) * fB0) + fT
								* (fA01 * fS + fA11 * fT + ((double)2.0) * fB1) + fC;
						}
					}
					else {
						fT = (double)0.0;
						if (fTmp1 <= (double)0.0) {
							fS = (double)1.0;
							fSqrDistance = fA00 + ((double)2.0) * fB0 + fC;
						}
						else if (fB0 >= (double)0.0) {
							fS = (double)0.0;
							fSqrDistance = fC;
						}
						else {
							fS = -fB0 / fA00;
							fSqrDistance = fB0 * fS + fC;
						}
					}
				}
				else
					// region 1
				{
					fNumer = fA11 + fB1 - fA01 - fB0;
					if (fNumer <= (double)0.0) {
						fS = (double)0.0;
						fT = (double)1.0;
						fSqrDistance = fA11 + ((double)2.0) * fB1 + fC;
					}
					else {
						fDenom = fA00 - 2.0f * fA01 + fA11;
						if (fNumer >= fDenom) {
							fS = (double)1.0;
							fT = (double)0.0;
							fSqrDistance = fA00 + ((double)2.0) * fB0 + fC;
						}
						else {
							fS = fNumer / fDenom;
							fT = (double)1.0 - fS;
							fSqrDistance =
								fS * (fA00 * fS + fA01 * fT + ((double)2.0) * fB0) + fT
								* (fA01 * fS + fA11 * fT + ((double)2.0) * fB1) + fC;
						}
					}
				}
			}

			// // account for numerical round-off error
			// if (fSqrDistance < (double) 0.0)
			// {
			// fSqrDistance = (double) 0.0;
			// }

			// m_kClosestPoint0 = m_rkVector;
			if (!closest->equals(NULL)) {
				closest->zero();
				kEdge0->mul(fS);
				kEdge1->mul(fT);
				closest->add(*kEdge1);
				closest->add(*kEdge1);
				closest->add(*v0);
				// m_kClosestPoint1 = v0 + fS*kEdge0 + fT*kEdge1;
			}

			cl_point = closest;

			if (fSqrDistance < 0)
				return 0;
			else
				return sqrt(fSqrDistance);
		}

	//------------------------------------------------------------------------------
} // namespace chai3d
  //------------------------------------------------------------------------------


