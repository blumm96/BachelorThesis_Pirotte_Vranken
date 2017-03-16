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
//------------------------------------------------------------------------------
#include <vector>
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
	class Voxelizer : public cCollisionAABB
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
		inline void setObject(cCollisionAABB* object) { this->object = object; };
		inline void addVoxel(Voxel* v) { voxels.push_back(v); };
		void mapDistances();

		cVector3d* find_closest_point(Voxel* v);

	public:
		//--------------------------------------------------------------------------
		// MEMBERS:
		//--------------------------------------------------------------------------

	protected:

		//Inputs
		//1)AABB tree structure with in the leafs pointers to the contained triangles
		//2)Voxel point array of the object -> implement a method which calculates this vector with use of the object bounding box

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
		float low_dist_sq = FLT_MAX;

		//The point associated with this lowest disctance
		cVector3d* closest_point = new cVector3d(0.0f, 0.0f, 0.0f);

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
		float lowest_upper_dist_sq = FLT_MAX;

	private:
		//--------------------------------------------------------------------------
		// HELPMETHODS:
		//--------------------------------------------------------------------------
		void map_distances();
		void seed_next_voxel_search();
		void process_node(cCollisionAABBNode* n, Voxel* v);
		void closest_point_triangle(Voxel* v, Triangle* t, float &dst, cVector3d *closest_point);
	};

	//------------------------------------------------------------------------------
} // namespace chai3d
  //------------------------------------------------------------------------------

  //------------------------------------------------------------------------------
#endif
  //------------------------------------------------------------------------------

