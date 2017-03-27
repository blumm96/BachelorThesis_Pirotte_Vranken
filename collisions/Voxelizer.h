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
#ifndef VoxelizerH
#define VoxelizerH
//------------------------------------------------------------------------------
#include "collisions/CCollisionAABB.h"
#include "collisions/CCollisionAABBTree.h"
#include "collisions/Voxel.h"
#include "math/CMaths.h"
#include "collisions/Triangle.h"
#include "Triangle.h"
#include "ist/InnerSphereTree.h"
//------------------------------------------------------------------------------
#include <vector>
#include <limits>
#include <list>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
	//------------------------------------------------------------------------------

	//==============================================================================
	/*!
	\file       Voxelizer.h

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
	class Voxelizer
	{
		//--------------------------------------------------------------------------
		// CONSTRUCTOR & DESTRUCTOR:
		//--------------------------------------------------------------------------

	public:

		//! Constructor of Voxelizer.
		Voxelizer();

		//! Destructor of Voxelizer.
		~Voxelizer();


		//--------------------------------------------------------------------------
		// METHODS:
		//--------------------------------------------------------------------------
		void mapDistances();
		InnerSphereTree* buildInnerTree(int diepte);

		void find_closest_point(Voxel* v);
		void setObject(cCollisionAABB* c);

	public:
		//--------------------------------------------------------------------------
		// MEMBERS:
		//--------------------------------------------------------------------------

	protected:

		//Output
		//A mapping of the voxel point vector (2) to a float value of closest distance to the object

		//1
		cCollisionAABB* object;
		//2
		//In the class voxel we can set a mindist parameter
		std::vector<Voxel*> voxels;	

		//algorithm vars
		//Boxes still to descend by the algorithm
		std::list<cCollisionAABBNode*> toDescend;

		//The smallest square distance to the object so far
		float low_dist_sq = std::numeric_limits<float>::infinity();

		//The point associated with this lowest distance
		//cVector3d* closest_point;

		//The triangle assaociated with the lowest distance
		Triangle* closest_triangle;

		// The tree node associated with the closest
		// point. We store this to help us exploit
		// spatial coherence when we move on to our
		// next voxel.
		//
		// This will always be a leaf.
		cCollisionAABBNode* closest_point_node = new cCollisionAABBNode();

		// The lowest upper-bound squared distance
		// to a box we’ve seen so far for the
		// current voxel.
		float lowest_upper_dist_sq = std::numeric_limits<float>::infinity();

		//priority que->stores voxels from far to closest
		std::list<Voxel*> priorityList;

	private:
		//--------------------------------------------------------------------------
		// HELPMETHODS:
		//--------------------------------------------------------------------------
		void map_distance_to_voxel(Voxel* v);
		//void seed_next_voxel_search();
		void process_node(cCollisionAABBNode* n, Voxel* v);
		double closest_point_triangle(Voxel* v, Triangle* t);
		void initialize();
		double nearestpoint(cVector3d* v0, cVector3d* v1, cVector3d* v2, cVector3d* p);
		double nearestpoint2(cVector3d* v0, cVector3d* v1, cVector3d* v2, cVector3d* p);
		double distance(Voxel* v1, Voxel* v2);
		//--------------------------------------------------------------------------
		// HELPMEMBERS:
		//--------------------------------------------------------------------------
		std::vector<cCollisionAABBNode> object_nodes;
		unsigned long root_index;
		std::vector<Voxel*> maakVoxels(cVector3d* max, cVector3d* min, cCollisionAABBNode* node, cCollisionAABB* tree, cVector3d* pos, float accuraatheid);
	};

	//help function for comparing two voxels
	struct compare {
		bool operator()(Voxel* v1,Voxel* v2) { return ((v1->getMinDist()) > (v2->getMinDist())); }
	};

	//------------------------------------------------------------------------------
} // namespace chai3d
  //------------------------------------------------------------------------------

  //------------------------------------------------------------------------------
#endif
  //------------------------------------------------------------------------------

