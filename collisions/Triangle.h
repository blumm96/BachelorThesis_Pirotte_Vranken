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
		inline Triangle(cVector3d* px, cVector3d* py, cVector3d* pz) {
			this->p1 = px;
			this->p2 = py;
			this->p3 = pz;

			// Bereken de randen van de driehoek.
			cVector3d edge21 = cVector3d(*p2);
			cVector3d edge31 = cVector3d(*p3);
			edge21.sub(*p1);
			edge31.sub(*p1);

			// Bereken de normaal van deze driehoek.
			N = cVector3d(edge21);
			N.cross(edge31);
		};

		//! Destructor of Voxelizer.
		inline ~Triangle() {
			delete p1;
			delete p2;
			delete p3;
		};

		inline cVector3d* getCenter() {
			cVector3d vector = cVector3d(0, 0, 0);
			vector.set(
				(p1->x() + p2->x() + p3->x())/3,
				(p1->y() + p2->y() + p3->y())/3,
				(p1->z() + p2->z() + p3->z())/3
			);
			return &vector;
		}
		///////////////////////////////////////////////////////////////////////
		//
		// Source: http://web.stanford.edu/class/cs277/resources/papers/Moller1997b.pdf
		//
		// This function calculates the intersection between 2 triangles.
		// 
		// \param t2 The other triangle to test the intersection with.
		// 
		// \return If the \p t2 and the triangle this method is called from are intersecting.
		///////////////////////////////////////////////////////////////////////

		inline bool intersectie(Triangle* t2) {

			// De punten van de eerste driehoek.
			cVector3d p1[3];
			// De punten van de tweede driehoek.
			cVector3d p2[3];
			// De vertexen van de eerste driehoek geprojecteerd op L.
			cVector3d pv1[3];
			// De vertexen van de tweede driehoek geprojecteerd op L.
			cVector3d pv2[3];
			// Een signed afstand van de punten van de eerste driehoek tot het andere vlak.
			float dp1[3];
			// Een signed afstand van de punten van de tweede driehoek tot het andere vlak.
			float dp2[3];
			// Welke punt van de driehoek aan de andere kant van het vlak ligt.
			int tr[2];

			// Assign de punten van de eerste driehoek.
			p1[0] = *(this->p1);
			p1[1] = *(this->p2);
			p1[2] = *(this->p3);

			// Assign de punten van de tweede driehoek.
			p2[0] = *(t2->p1);
			p2[1] = *(t2->p2);
			p2[2] = *(t2->p3);

			// Assign de normaal van de eerste driehoek.
			cVector3d N1 = N;
			// Assign de normaal van de tweede driehoek.
			cVector3d N2 = t2->N;

			// Bereken voor deel van de plane equation.
			float d1 = (float)-(N1.dot(p1[0]));
			float d2 = (float)-(N2.dot(p2[0]));

			// Bereken de afstanden voor elke vertex van de tweede driehoek tot het plane.
			// Deze afstand is signed.
			dp2[0] = N2.dot(p1[0]) + d2;
			dp2[1] = N2.dot(p1[1]) + d2;
			dp2[2] = N2.dot(p1[2]) + d2;

			// Bereken de afstanden voor elke vertex van de eerste driehoek tot het plane.
			// Deze afstand is signed.
			dp1[0] = N1.dot(p2[0]) + d1;
			dp1[1] = N1.dot(p2[1]) + d1;
			dp1[2] = N1.dot(p2[2]) + d1;

			// Bepaal of we door moeten gaan met de berekening.
			// Bepaal ook welke van de vertexen aan de andere kant van het plane ligt.

			int a, b;

			if (dp1[0] == 0 && dp1[1] == 0 && dp1[2] == 0) {
				a = 0;
			}
			else {
				if (dp1[0] < 0) {
					if (dp1[1] < 0 && dp1[2] < 0) { 
						a = -1; 
					}
					else if (dp1[1] > 0 && dp1[2] > 0) {
						a = 1; tr[0] = 0;
					}
					else if (dp1[2] < 0) {
						a = 1; tr[0] = 1;
					}
					else if (dp1[1] < 0) {
						a = 1; tr[0] = 2;
					}
				}
				else if (dp1[1] > 0 && dp1[2] > 0) {
					a = -1;
				}
			}

			if (dp2[0] == 0 && dp2[1] == 0 && dp2[2] == 0) {
				b = 0;
			}
			else {
				if (dp2[0] < 0) {
					if (dp2[1] < 0 && dp2[2] < 0) {
						b = -1;
					}
					else if (dp2[1] > 0 && dp2[2] > 0) {
						b = 1; tr[1] = 0;
					}
					else if (dp2[2] < 0) {
						b = 1; tr[1] = 1;
					}
					else if (dp2[1] < 0) {
						b = 1; tr[1] = 2;
					}
				}
				else if (dp2[1] > 0 && dp2[2] > 0) {
					b = -1;
				}
			}

			int status;

			if (a == 0) status = 0;
			else if (a == -1 || b == -1) status = -1;
			else status = 1;

			if (status == -1) return false;
			else if (status == 0) return false;
			else if (status == 1) {

				// De richting van de lijn waar de 2 vlakken elkaar raken.
				cVector3d D = cVector3d(N1);
				D.cross(N2);

				// Projecteer de vertexen van elk punt van de twee driehoeken op L.
				for (int i = 0; i < 3; i++) {
					pv1[i] = D;
					pv1[i].cross(p1[i]);
					pv2[i] = D;
					pv2[i].cross(p2[i]);
				}

				// The first intersection point of the first triangle.
				cVector3d t11;
				// The second intersection point of the first triangle.
				cVector3d t21;
				// The first intersection point of the second triangle.
				cVector3d t12;
				// The second intersection point of the second triangle.
				cVector3d t22;

				// Bereken de intervallen.

				// Eerste driehoek.
				// Deze als p0 aan de andere kant van het vlak ligt.
				if (tr[0] == 0) {
					t11 = berekenIntersectie(pv1[0], pv1[1], dp1[0], dp1[1]);
					t21 = berekenIntersectie(pv1[0], pv1[2], dp1[0], dp1[2]);
				}
				// Deze als p1 aan de andere kant van het vlak ligt.
				else if (tr[0] == 1) {
					t11 = berekenIntersectie(pv1[1], pv1[0], dp1[1], dp1[0]);
					t21 = berekenIntersectie(pv1[1], pv1[2], dp1[1], dp1[2]);
				}
				// Anders ligt p2 aan de andere kant van het vlak.
				else {
					t11 = berekenIntersectie(pv1[2], pv1[0], dp1[2], dp1[0]);
					t21 = berekenIntersectie(pv1[2], pv1[1], dp1[2], dp1[1]);
				}

				// Tweede driehoek.
				// Deze als p0 aan de andere kant van het vlak ligt.
				if (tr[1] == 0) {
					t12 = berekenIntersectie(pv2[0], pv2[1], dp2[0], dp2[1]);
					t22 = berekenIntersectie(pv2[0], pv2[2], dp2[0], dp2[2]);
				}
				// Deze als p1 aan de andere kant van het vlak ligt.
				else if (tr[1] == 1) {
					t12 = berekenIntersectie(pv2[1], pv2[0], dp2[1], dp2[0]);
					t22 = berekenIntersectie(pv2[1], pv2[2], dp2[1], dp2[2]);
				}
				// Anders ligt p2 aan de andere kant van het vlak.
				else {
					t12 = berekenIntersectie(pv2[2], pv2[0], dp2[2], dp2[0]);
					t22 = berekenIntersectie(pv2[2], pv2[1], dp2[2], dp2[1]);
				}

				// Make sure that t11 < t21
				if (t11.length() > t21.length()) {
					cVector3d help = t11;
					t11.set(t21.x(), t21.y(), t21.z());
					t21.set(help.x(), help.y(), help.z());
				}

				// Make sure t12 < t22
				if (t12.length() > t22.length()) {
					cVector3d help = t12;
					t12.set(t22.x(), t22.y(), t22.z());
					t22.set(help.x(), help.y(), help.z());
				}

				if (t21.length() >= t12.length() && t22.length() >= t11.length()) {
					return true;
				}

				return false;
				
			}

			return false;

		}

		inline cVector3d berekenIntersectie(cVector3d pv0, cVector3d pv1, float dv0, float dv1) {
			cVector3d uitkomst = cVector3d(pv1);
			uitkomst.sub(pv0);
			uitkomst.mul(dv0 / (dv0 - dv1));
			uitkomst.add(pv0);
			return uitkomst;
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
		cVector3d N;
	};

	//------------------------------------------------------------------------------
} // namespace chai3d
  //------------------------------------------------------------------------------

  //------------------------------------------------------------------------------
#endif
  //------------------------------------------------------------------------------


