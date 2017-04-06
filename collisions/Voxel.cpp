//==============================================================================
/*

\author    Niels Pirotte
\author    Casper Vranken
\version   1.0.0
*/
//==============================================================================

//+++++++++++++UHAS IMPLEMENTED+++++++++++++++++++++++++++++++++++++++++++++++++

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#include "math/CMaths.h"
#include "math/CVector3d.h"
#include <limits>
#include "collisions/Voxel.h"
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
	//--------------------------------------------------------------------------
	// CONSTRUCTOR & DESTRUCTOR:
	//--------------------------------------------------------------------------

	//! Constructor of Voxel
	Voxel::Voxel() {
		pos = new cVector3d(0.0, 0.0, 0.0);
		minDist = std::numeric_limits<float>::max();
	}

	//! Constructor of Voxel
	Voxel::Voxel(const double &x, const double &y, const double &z) {
		setPos(x, y, z);
	}

	//! Destructor of Voxel
	Voxel::~Voxel() {
		delete(pos);
	}


	//--------------------------------------------------------------------------
	// METHODS:
	//--------------------------------------------------------------------------

	/*

	Set the position of the voxel.

	\param x The x-component of the position.
	\param y The y-component of the position.
	\param z The z-component of the position.

	*/
	void Voxel::setPos(const double &x, const double &y, const double &z) {
		pos = new cVector3d(x, y, z);
	}

	/*
	
		Set the minimum distance to the surface.

		\param dist The new minimum distance to the surface.
		
	*/
	void Voxel::setMinDist(float dist) { minDist = dist; }


	  //------------------------------------------------------------------------------
}	  // namespace chai3d
	  //------------------------------------------------------------------------------