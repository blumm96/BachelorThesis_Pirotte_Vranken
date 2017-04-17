#include "ist/Sphere.h"

#include "ist/InnerSphereTree.h"

#include "math/CVector3d.h"

#include "graphics/CDraw3D.h"

#include <math.h>

#include <vector>

namespace chai3d {

	/*
		The constructor of a sphere.
	*/
	Sphere::Sphere() {
		mindist = std::numeric_limits<float>::infinity();
	}

	/*
		The destructor of a sphere.
	*/
	Sphere::~Sphere() {

		//delete all sphere points
		for (int i = 0; i < spherePoints.size(); i++) {
			if(spherePoints[i] != nullptr) delete spherePoints[i];
		}

		//delete all children
		for (int i = 0; i < children.size(); i++) {
			if (children[i] != nullptr) delete children[i];
		}
	}

	/*
		Calculates the distance between two spheres.

		\param sphere		The other sphere to calculate the distance between.
		\param position1	The global position of the root sphere of this sphere.
		\param position2	The global position of the root sphere of the given sphere.

		\return The distance between the two given spheres.
	*/
	float Sphere::distance(Sphere* sphere, InnerSphereTree* tree1, InnerSphereTree* tree2) {
		cVector3d x = this->getPosition().x()*tree1->getB1();
		cVector3d y = this->getPosition().y()*tree1->getB2();
		cVector3d z = this->getPosition().z()*tree1->getB3();
		cVector3d posThis = tree1->getPosition() + x + y + z;
		
		x = sphere->getPosition().x()*tree2->getB1();
		y = sphere->getPosition().y()*tree2->getB2();
		z = sphere->getPosition().z()*tree2->getB3();
		cVector3d posS = tree2->getPosition() + x + y + z;
		cVector3d hulp = posThis - posS;
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

	/*
		
		Returns the triangle associated with this sphere.

		\return The triangle of this sphere.

	*/
	Triangle* Sphere::getTriangle()
	{
		return triangle;
	}

	/*
		
		Set the radius of this sphere.

		\param r The new radius of this sphere.

	*/
	void Sphere::setRadius(float r) {
		radius = r;
	}

	/*
	
		Set the position of this sphere.

		\param pos The new position of this sphere.

	*/
	void Sphere::setPosition(cVector3d pos) {
		position = pos;
	}

	/*
		
		Set the state of this sphere.

		\return nstate The new state of this sphere.

	*/
	void Sphere::setState(sphereState nstate) {
		state = nstate;
	}

	/*
		
		Set the triangle associated with this sphere.

		\param setT The new triangle associated with this sphere.

	*/
	void Sphere::setTriangle(Triangle* setT)
	{
		triangle = setT;
	}

	/*
		
		Set the parent of this sphere.

		\param n_parent The new parent of this sphere.

	*/
	void Sphere::setParent(Sphere* n_parent)
	{
		parent = n_parent;
	}

	/*
		
		Set the depth of this sphere within the inner sphere tree.

		\param d The new depth of this sphere.

	*/
	void Sphere::setDepth(int d)
	{
		depth = d;
	}

	/*
		
		Add a child to this sphere.

		\param child The child to be added to this sphere.

	*/
	void Sphere::addChild(Sphere* child)
	{
		children.push_back(child);
	}

	/*
		
		Render this sphere.

	*/
	void Sphere::render() {
			initRender(); //this function needs to be called if the sphere changes (center or radius)
			cDrawSphere(spherePoints);
	}

	// Input arguments
	// in  - VERTEX center                 : defines the center of the sphere, all points will be offset from this
	// in  - double r                      : defines the radius of the sphere
	// out - vector<VERTEX> & spherePoints : vector containing the points of the sphere
	void Sphere::make_Sphere(cVector3d center, double r, std::vector<cVector3d*> &spherePoints)
	{
		const double PI = M_PI;
		const int numRings = 10;
		const int numElem = 10;

		const float numRingsf = (float)numRings;
		const float numElemf = (float)numElem;

		spherePoints.clear();

		//first index = the ring
		//second index = element in the ring
		std::vector<std::vector<cVector3d*>> horRings;

		// Iterate through phi, theta then convert r,theta,phi to  XYZ
		double theta, phi;
		theta = phi = 0;

		for (int i = 0; i < numRings; i++) // Elevation [0, PI]
		{	
			std::vector<cVector3d*> row;
			row.clear();
			for (int j = 0; j < numElem; j++) // Azimuth [0, 2PI]		
			{
				cVector3d* point = new cVector3d();
				double x, y, z;
				if (i == 0) {
					x = 0 + center.x();
					y = 0 + center.y();
					z = r + center.z();
				}
				else if (i == numRings-1) {
					x = 0 + center.x();
					y = 0 + center.y();
					z = -r + center.z();
				}
				else {
					x = r * cos(phi) * sin(theta) + center.x();
					y = r * sin(phi) * sin(theta) + center.y();
					z = r            * cos(theta) + center.z();
				}
				point->set(x, y, z);
				row.push_back(point);
				phi += (2 * PI) / numElemf;
			}
			row.push_back(row[0]);
			horRings.push_back(row);
			theta += PI / numRingsf;
		}

		for (int k = 0; k < numRings-1; k++) {
			for (int l = 0; l < numElem; l++) {
				spherePoints.push_back(horRings[k][l]);
				spherePoints.push_back(horRings[k+1][l]);
				spherePoints.push_back(horRings[k+1][l+1]);
			}
		}
		return;
	}
}
