#include "ist/Sphere.h"

#include "math/CVector3d.h"

namespace chai3d {

	/*
		The constructor of a sphere.
	*/
	Sphere::Sphere() {}

	/*
		The destructor of a sphere.
	*/
	Sphere::~Sphere() {
		for (int i = 0; i < children.size(); i++) {
			delete children[i];
		}
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

	void Sphere::setRadius(float r) {
		radius = r;
	}

	void Sphere::setPosition(cVector3d pos) {
		position = pos;
	}

	void Sphere::setState(sphereState nstate) {
		state = nstate;
	}
}
