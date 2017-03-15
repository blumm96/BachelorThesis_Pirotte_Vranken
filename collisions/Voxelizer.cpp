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
#include "collisions/Voxel.h"
#include "collisions/Voxelizer.h"
#include "math/CMaths.h"
//------------------------------------------------------------------------------
#include <vector>
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
		Voxelizer::Voxelizer() {}

		//! Destructor of Voxelizer.
		Voxelizer::~Voxelizer() {}


		//--------------------------------------------------------------------------
		// METHODS:
		//--------------------------------------------------------------------------

		void Voxelizer::mapDistances() {
			// Process each voxel on our list, one
			// at a time...

			std::vector<Voxel*>::iterator iter = voxels.begin();
			while (iter != voxels.end) {
				// Grab the next voxel
				Voxel* v = (*iter);

				// Now we’re going to find the closest 
				// point in the tree (tree_root) to v... 

				closest_point = find_closest_point(v);

				// Now output or do something useful 
				// with lowest_dist_sq and closest_point 
				// these are the values that should be 
				// associated with v in our output => distance map
				
				map_distances(); 
				
				// Exploit spatial coherence by giving the next voxel 
				// a "hint" about where to start looking in the tree. 
				// it seeds 'boxes_to_descend' with a good starting point for the next voxel. 
				
				seed_next_voxel_search();

			}
		}

		// Find the closest point in our mesh to the sample point v
		cVector3d* Voxelizer::find_closest_point(Voxel* v) {

			vector<cCollisionAABBNode> object_nodes = object->getNodes();
			int root_index = object->getRoot();

			// Start with the root of the tree 
			toDescend.push_back(&object_nodes[root_index]);

			while (!toDescend.empty()) { 
				cCollisionAABBNode* node = toDescend.front();
				toDescend.pop_front();
				//Process each node
				process_node(node, v); 
			}
		}

		void Voxelizer::map_distances() {
		
		}
		
		void Voxelizer::seed_next_voxel_search() {
		
		}

		void Voxelizer::process_node(cCollisionAABBNode * n, Voxel * v)
		{
		}

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
					lowest_upper_dist_sq = d_sq; }
				
				// This was a leaf and we’re done with him whether he was useful or not.
				return;
			}
			//if this node was not a leaf
			// Computing lower- and upper-bound distances to an axis-aligned bounding 
			// box is extremely fast; we just take the farthest plane on each axis 
			float best_dist = 0; 
			float worst_dist = 0;

			// If I'm below the x range, my lowest x distance uses the minimum x, and 
			// my highest uses the maximum x 
			if (v->getPos()->x() < n->m_bbox.getLowerX()) { 
				best_dist += n->m_bbox.getLowerX() - v->getPos()->x();
				worst_dist += n->m_bbox.getUpperX() - v->getPos()->x();
			}

			// If I'm above the x range, my lowest x 
			// distance uses the maximum x, and my 
			// highest uses the minimum x 
			else if (v->getPos()->x() < n->m_bbox.getUpperX()) {
				best_dist += v->getPos()->x() - n->m_bbox.getUpperX();
				worst_dist += v->getPos()->x() - n->m_bbox.getLowerX();
			}

			// If I'm _in_ the x range, x doesn't affect my lowest distance, and my 
			// highest-case distance goes to the _farther_ of the two x distances 
			else { 
				float dmin = fabs(n->m_bbox.getLowerX() - v->getPos()->x());
				float dmax = fabs(n->m_bbox.getUpperX() - v->getPos()->x());
				double d_worst = (dmin>dmax) ? dmin : dmax;
				worst_dist += d_worst;
			}

			//repeat for y

			if (v->getPos()->y() < n->m_bbox.getLowerY()) {
				best_dist += n->m_bbox.getLowerY() - v->getPos()->y();
				worst_dist += n->m_bbox.getUpperY() - v->getPos()->y();
			}

			else if (v->getPos()->y() < n->m_bbox.getUpperY()) {
				best_dist += v->getPos()->y() - n->m_bbox.getUpperY();
				worst_dist += v->getPos()->y() - n->m_bbox.getLowerY();
			}

			else {
				float dmin = fabs(n->m_bbox.getLowerY() - v->getPos()->y());
				float dmax = fabs(n->m_bbox.getUpperY() - v->getPos()->y());
				double d_worst = (dmin>dmax) ? dmin : dmax;
				worst_dist += d_worst;
			}
		}

	//------------------------------------------------------------------------------
} // namespace chai3d
  //------------------------------------------------------------------------------


