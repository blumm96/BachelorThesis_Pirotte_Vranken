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

		//setters
	void Voxel::setPos(const double &x, const double &y, const double &z) {
		pos = new cVector3d(x, y, z);
	}

<<<<<<< HEAD
	void Voxel::setMinDist(float dist) { minDist = dist; }


=======
>>>>>>> origin/IST
	  //------------------------------------------------------------------------------
}	  // namespace chai3d
	  //------------------------------------------------------------------------------