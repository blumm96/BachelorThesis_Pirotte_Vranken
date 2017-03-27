#ifndef VOXELMAKER_H
#define VOXELMAKER_H

#include "math/CVector3d.h"

#define EPSILON 0.000001

namespace chai3d {

	// Checks if a ray intersects with a triangle
	int static triangle_intersection(cVector3d V1,
		cVector3d   V2,
		cVector3d   V3,
		cVector3d    O,
		cVector3d    D,
		float* out,
		cVector3d pos)
	{	

		if (V1.x() < O.x() && V2.x() < O.x() && V3.x() < O.x()) return 0;
		if (V1.y() < O.y() && V2.y() < O.y() && V3.y() < O.y()) return 0;
		if (V1.y() > O.y() && V2.y() > O.y() && V3.y() > O.y()) return 0;
		if (V1.z() < O.z() && V2.z() < O.z() && V3.z() < O.z()) return 0;
		if (V1.z() > O.z() && V2.z() > O.z() && V3.z() > O.z()) return 0;

		cVector3d e1, e2;
		cVector3d P, Q, T;
		float det, inv_det, u, v;
		float t;

		V1.add(pos);
		V2.add(pos);
		V3.add(pos);

		e1 = V2 - V1;
		e2 = V3 - V1;
		P = D;
		P.cross(e2);
		det = e1.dot(P);
		if (det > -EPSILON && det < EPSILON) return 0;
		inv_det = 1.f / det;

		T = O - V1;

		u = T.dot(P) * inv_det;
		if (u < 0.f || u > 1.f) return 0;

		Q = T;
		Q.cross(e1);

		v = D.dot(Q) * inv_det;
		if (v < 0.f || u + v  > 1.f) return 0;

		t = e2.dot(Q) * inv_det;

		if (t > EPSILON) {
			*out = t;
			return 1;
		}

		return 0;
	}

	// This method checks if a ray starting from the origin point in direction (1,0,0) hits a triangle using the tree structure.
	void static rayBoxIntersection(cCollisionAABBNode* node,
		cCollisionAABB* tree,
		cVector3d* origin,
		int &aantalRakingen) {

		cCollisionAABBBox* box = &(node->m_bbox);

		if (node->m_nodeType == cAABBNodeType::C_AABB_NODE_LEAF) {
			float out = 0;
			if (triangle_intersection(*(node->m_bbox.triangle->p1), *(node->m_bbox.triangle->p2), *(node->m_bbox.triangle->p3), *origin, cVector3d(1, 0, 0), &out, cVector3d(0, 0, 0))) {
				aantalRakingen++;
				return;
			}
			return;
		}

		//Check if origin is in the box or if the ray hits the box
		if (origin->x() < box->getUpperX()) {
			if (origin->y() < box->getUpperY() && origin->y() > box->getLowerY()) {
				if (origin->z() < box->getUpperZ() && origin->z() > box->getLowerZ()) {
					rayBoxIntersection(&(tree->m_nodes[node->m_leftSubTree]), tree, origin, aantalRakingen);
					rayBoxIntersection(&(tree->m_nodes[node->m_rightSubTree]), tree, origin, aantalRakingen);
				}
			}
		}

	}
}
#endif