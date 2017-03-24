#include "ist/InnerSphereTree.h"
#include <iostream>

using namespace std;

namespace chai3d {

	/*
		Constructor of an inner sphere tree.
	*/
	InnerSphereTree::InnerSphereTree() {
		
	}

	/*
		Destructor of the inner sphere tree.
	*/
	InnerSphereTree::~InnerSphereTree() {
		delete rootSphere;
	}

	void InnerSphereTree::printAABBCollisionTree(int diepte) {
		cout << "INNER SPHERE TREE" << endl;
		for (int i = 0; i < spheres.size(); i++) {
			cout << "Sphere: " << spheres[i]->getPosition() << endl;
		}
	}

	/*
		This function computes if a collision has occured between two inner sphere trees.

		\param ist2					he other inner sphere tree to compute the possible collision.
		\param setting				The traversalsettings.
		\param collisionfeedback	How close the collision is.
		\param maxdiepte			The maximumdepth the function is allowed to check for collision.

		\return If to inner sphere trees have collided.
	*/
	bool InnerSphereTree::computeCollision(cGenericCollision* ist2, traversalSetting setting, double &collisionfeedback, int maxdiepte, cVector3d myLocal, cVector3d BLocal, cVector3d& positie) {
		// Sanity check
		if (ist2 == NULL) return false;
		if (this->getCollisionTreeType() != ist2->getCollisionTreeType()) return false;

		switch (setting) {
		case traversalSetting::DISTANCE: {
			InnerSphereTree* IST_B = dynamic_cast<InnerSphereTree*>(ist2);
			InnerSphereTree* IST_A = this;

			Sphere* parent_A = IST_B->getRootSphere();
			Sphere* parent_B = IST_A->getRootSphere();

			double mindist = std::numeric_limits<double>::max();
			int huidige = 0;
		}
		case traversalSetting::COMBINED: return false;
		case traversalSetting::VOLUME_PEN: return false;
		default: return false;
		}
		return true;
	}

	/*
		Get the root sphere of this inner sphere tree.

		\return The rootsphere.
	*/
	Sphere* InnerSphereTree::getRootSphere() {
		return rootSphere;
	}
	int InnerSphereTree::buildTree(std::vector<Sphere*> leafs, const int a_depth)
	{
		for (int i = 0; i < leafs.size(); i++) {
			spheres.push_back(leafs[i]);
		}
		return 0;
	}

	void InnerSphereTree::render(cRenderOptions& a_options) {
#ifdef C_USE_OPENGL
		
		glDisable(GL_LIGHTING);
		glLineWidth(1.0);
		glColor4fv(cColorf(1.0, 0, 0).getData());
		for (int i = 0; i < spheres.size(); i++) {
			spheres[i]->render();
			cout << i << endl;
		}

		glEnable(GL_LIGHTING);
#endif
	}

}