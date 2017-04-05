//==============================================================================
/*
This class represents a Voxel.

\author    Niels Pirotte
\author    Casper Vranken
\version   1.0.0
*/
//==============================================================================

//+++++++++++++UHAS IMPLEMENTED+++++++++++++++++++++++++++++++++++++++++++++++++

//------------------------------------------------------------------------------
#ifndef VoxelH
#define VoxelH
//------------------------------------------------------------------------------
#include "math/CMaths.h"
#include "math/CVector3d.h"
#include "collisions/Triangle.h"
//------------------------------------------------------------------------------
#include <vector>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
	//------------------------------------------------------------------------------

	//==============================================================================
	/*!
	\file       Voxel.h

	\brief
	Implements a voxel point in the world used in combination with the voxalizer to
	implement the algorithm described in the voxalizer class.
	*/
	//==============================================================================

	//==============================================================================
	/*!
	\class      Voxel
	\ingroup    collisions

	*/
	//==============================================================================
	class Voxel
	{
		//--------------------------------------------------------------------------
		// CONSTRUCTOR & DESTRUCTOR:
		//--------------------------------------------------------------------------

	public:

		//! Constructor of Voxel
		Voxel();

		//! Constructor of Voxel
		Voxel(const double &x, const double &y, const double &z);

		//! Destructor of Voxel
		~Voxel();


		//--------------------------------------------------------------------------
		// METHODS:
		//--------------------------------------------------------------------------

	public:
		//getters
		/*
			
			Get the position of the voxel.

			\return The position of the voxel.

		*/
		inline cVector3d* getPos() { return pos; };

		/*
			
			Get the minimum distance to the surface.

		*/
		inline float getMinDist() { return minDist; };

		//setters
		// Set the position of the voxel.
		void setPos(const double &x, const double &y, const double &z);
		// Set the minimum distance to the surface.
		void setMinDist(float dist);

		/*
			
			Set the triangle associated to the voxel.

			\param setT The triangle to be set.

		*/
		inline void setTriangle(Triangle* setT) { t = setT; }

	public:
		//--------------------------------------------------------------------------
		// MEMBERS:
		//--------------------------------------------------------------------------

		// The position of this voxel.
		cVector3d* pos;
		// The minimum distance to the surface.
		float minDist;
		// The triangle associated to this sphere.
		Triangle* t;

	public:

		/*
			
			An operator used to sort the voxels.

		*/
		inline bool operator <(const Voxel* voxel1) {
			return this->minDist > voxel1->minDist;
		}
	};
	//------------------------------------------------------------------------------
}	  // namespace chai3d
	  //------------------------------------------------------------------------------

	  //------------------------------------------------------------------------------
#endif
	  //------------------------------------------------------------------------------