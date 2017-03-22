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
#ifndef TRIANGLEH
#define TRIANGLEH
//------------------------------------------------------------------------------
#include "math/CVector3d.h"


//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
	//------------------------------------------------------------------------------

	//==============================================================================
	/*!
	\file       Triangle.h

	\brief
		Implements a simple triangle;
	*/
	//==============================================================================

	//==============================================================================
	/*!
	\class      Triangle
	\ingroup    collisions

	*/
	//==============================================================================
	class Triangle
	{
		//--------------------------------------------------------------------------
		// CONSTRUCTOR & DESTRUCTOR:
		//--------------------------------------------------------------------------

	public:

		//! Constructor of Voxelizer.
		inline Triangle(cVector3d* p1, cVector3d* p2, cVector3d* p3) {
			this->p1 = p1;
			this->p2 = p2;
			this->p3 = p3;
		};

		//! Destructor of Voxelizer.
		inline ~Triangle() {};

		inline cVector3d* getCenter() {
			cVector3d* vector = new cVector3d(0, 0, 0);
			vector->set(
				(p1->x() + p2->x() + p3->x())/3,
				(p1->y() + p2->y() + p3->y())/3,
				(p1->z() + p2->z() + p3->z())/3
			);
		}


		//--------------------------------------------------------------------------
		// METHODS:
		//--------------------------------------------------------------------------
		

	public:
		//--------------------------------------------------------------------------
		// MEMBERS:
		//--------------------------------------------------------------------------
		cVector3d* p1;
		cVector3d* p2;
		cVector3d* p3;
	};

	//------------------------------------------------------------------------------
} // namespace chai3d
  //------------------------------------------------------------------------------

  //------------------------------------------------------------------------------
#endif
  //------------------------------------------------------------------------------


