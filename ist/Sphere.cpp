#include "ist/Sphere.h"

#include "math/CVector3d.h"

#include "graphics/CDraw3D.h"

namespace chai3d {

	/*
		The constructor of a sphere.
	*/
	Sphere::Sphere() {}

	/*
		The destructor of a sphere.
	*/
	Sphere::~Sphere() {
		//if (state == sphereState::SPHERE_LEAF) return;
		//for (int i = 0; i < children.size(); i++) {
			//delete children[i];
		//}
		//delete triangle;
	}

	/*
		Calculates the distance between two spheres.

		\param sphere		The other sphere to calculate the distance between.
		\param position1	The global position of the root sphere of this sphere.
		\param position2	The global position of the root sphere of the given sphere.

		\return The disance between the two given spheres.
	*/
	float Sphere::distance(Sphere* sphere, cVector3d position1, cVector3d position2) {
		cVector3d hulp = position1 + this->getPosition() - position2 - sphere->getPosition();
		float lengte = hulp.length();
		float afstand =  lengte - sphere->getRadius() - this->getRadius();
		if (afstand > 0.0) return afstand;
		return 0.0;
	}

	/*
		Returns the position of the sphere relative to the parent.

		\return The relative position.
	*/
	cVector3d Sphere::getPosition() {
		return position;
	}

	/*
		Returns the radius of the sphere

		\return The radius.
	*/
	float Sphere::getRadius() {
		return radius;
	}

	/*
		Returns every child of the sphere. The sphere has no children if it is a leaf.

		\return The list of children.
	*/
	std::vector<Sphere*> Sphere::getChildren() {
		return children;
	}

	/*
		Returns the parent of the sphere. The sphere has no parent if it is the root.

		\return The parent of the sphere.
	*/
	Sphere* Sphere::getParent() {
		return parent;
	}

	/*
		Returns the state of the sphere within the innerspheretree.

		\return The state of the sphere.
	*/
	sphereState Sphere::getState() {
		return state;
	}
	/*
		
		Returns the depth of the sphere in the innersphere tree.

		\return The depth of the sphere.

	*/
	int Sphere::getDepth() {
		return depth;
	}

		Triangle * Sphere::getTriangle()
		{
			return triangle;
		}

	void Sphere::setRadius(float r) {
		radius = r;
	}

	void Sphere::setPosition(cVector3d pos) {
		position = pos;
	}

	void Sphere::setState(sphereState nstate) {
		state = nstate;
	}

	void Sphere::setTriangle(Triangle * setT)
	{
		triangle = setT;
	}

	void Sphere::render() {
		if (state == sphereState::SPHERE_LEAF) {
			make_Sphere(position, radius, spherePoints);
			cDrawSphere(spherePoints);
		}
	}

	// Input arguments
	// in  - VERTEX center                 : defines the center of the sphere, all points will be offset from this
	// in  - double r                      : defines the radius of the sphere
	// out - vector<VERTEX> & spherePoints : vector containing the points of the sphere
	void Sphere::make_Sphere(cVector3d center, double r, std::vector<cVector3d> &spherePoints)
	{
		const double PI = 3.141592653589793238462643383279502884197;
		spherePoints.clear();

		// Iterate through phi, theta then convert r,theta,phi to  XYZ
		for (double phi = 0.; phi < 2 * PI; phi += PI / 10.) // Azimuth [0, 2PI]
		{
			for (double theta = 0.; theta < PI; theta += PI / 10.) // Elevation [0, PI]
			{
				cVector3d point;
				double x, y, z;
				x = r * cos(phi) * sin(theta) + center.x();
				y = r * sin(phi) * sin(theta) + center.y();
				z = r            * cos(theta) + center.z();
				point.set(x, y, z);
				spherePoints.push_back(point);
			}
		}
		return;
	}
}
