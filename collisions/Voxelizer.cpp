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
		Voxelizer::~Voxelizer() {}

		//--------------------------------------------------------------------------
		// METHODS:
		//--------------------------------------------------------------------------

		void Voxelizer::initialize() {
			vector<cCollisionAABBNode> object_nodes = object->getNodes();
			int root_index = object->getRoot();
			voxels = object->maakVoxels();
		}

		//distance between 2 voxels
		double Voxelizer::distance(Voxel* v1, Voxel* v2) {
			return (*(v1->getPos()) - *(v2->getPos())).length();
		}

		//most important method of this class
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
				
				map_distances(v); 
				
				// Exploit spatial coherence by giving the next voxel 
				// a "hint" about where to start looking in the tree. 
				// it seeds 'boxes_to_descend' with a good starting point for the next voxel. 
				
				//seed_next_voxel_search();

			}

		}

		InnerSphereTree* Voxelizer::buildInnerTree()
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
			//InnerSphereTree* tree = new InnerSphereTree();
			//tree->buildTree(innerspheres, 5);
			return NULL;
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

		void Voxelizer::map_distances(Voxel* v) {
			v->setMinDist(sqrt(low_dist_sq));
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
				closest_point_triangle(v, NULL, d_sq, closest_pt_on_triangle);

				// Is this the shortest distance so far? 
				if (d_sq < low_dist_sq) {
					// Mark him as the closest we’ve seen 
					low_dist_sq = d_sq;
					closest_point = closest_pt_on_triangle;
					closest_point_node = n;

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
		}

	//------------------------------------------------------------------------------
} // namespace chai3d
  //------------------------------------------------------------------------------


