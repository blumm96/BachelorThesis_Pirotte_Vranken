#ifndef VOXELMAKER_H
#define VOXELMAKER_H

#include "math/CVector3d.h"

#define EPSILON 0.000001

namespace chai3d {

	int static triangle_intersection(cVector3d V1,  // Triangle vertices
		cVector3d   V2,
		cVector3d   V3,
		cVector3d    O,  //Ray origin
		cVector3d    D,  //Ray direction
		float* out)
	{
		cVector3d e1, e2;  //Edge1, Edge2
		cVector3d P, Q, T;
		float det, inv_det, u, v;
		float t;

		//Find vectors for two edges sharing V1
		e1 = V2 - V1;
		e2 = V3 - V1;
		//Begin calculating determinant - also used to calculate u parameter
		//CROSS(P, D, e2);
		P = D;
		P.cross(e2);
		//if determinant is near zero, ray lies in plane of triangle or ray is parallel to plane of triangle
		//det = DOT(e1, P);
		det = e1.dot(P);
		//NOT CULLING
		if (det > -EPSILON && det < EPSILON) return 0;
		inv_det = 1.f / det;

		//calculate distance from V1 to ray origin
		//SUB(T, O, V1);
		T = O;
		T.sub(V1);

		//Calculate u parameter and test bound
		u = T.dot(P) * inv_det;
		//The intersection lies outside of the triangle
		if (u < 0.f || u > 1.f) return 0;

		//Prepare to test v parameter
		Q = T;
		Q.cross(e1);

		//Calculate V parameter and test bound
		v = D.dot(Q) * inv_det;
		//The intersection lies outside of the triangle
		if (v < 0.f || u + v  > 1.f) return 0;

		t = e2.dot(Q) * inv_det;

		if (t > EPSILON) { //ray intersection
			*out = t;
			return 1;
		}

		// No hit, no win
		return 0;
	}
}
#endif