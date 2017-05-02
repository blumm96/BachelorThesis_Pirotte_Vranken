#ifndef INNERSPHERETREE_H
#define INNERSPHERETREE_H

#include "collisions/Paths.h"
#include "collisions/CGenericCollision.h"
#include "ist/Sphere.h"
#include <iostream>

/*
	UHAS IMPLEMENTED

	This class represents an inner sphere tree.

	\author Casper Vranken
	\author Niels Pirotte
*/

using namespace std;

namespace chai3d {
	class Sphere;
	class Paths;
	class InnerSphereTree : public cGenericCollision {
		// CONSTRUCTOR - DESTRUCTOR
	public:

		// Constructor of the inner sphere tree.
		InnerSphereTree();

		// Destructor of the inner sphere tree.
		virtual ~InnerSphereTree();

		// Static datamembers
	public:
		static Paths globalPath;

		// PUBLIC METHODS
	public:
		// Computes the collision between 2 inner sphere trees.
		virtual bool computeCollision(cGenericCollision* ist2, traversalSetting setting, double &collisionfeedback, int maxdiepte, cVector3d myLocal, cVector3d BLocal, cVector3d& positie);
		bool computeCollision(cGenericCollision * ist2, traversalSetting setting, double & collisionfeedback, int maxdiepte, cVector3d & positie);
		// Get the type of tree. In this case IST.
		virtual inline CollisionTreeType getCollisionTreeType() { return CollisionTreeType::IST; };
		// Render the spheres.
		virtual void render(cRenderOptions& a_options);
		// Get the rootsphere of this inner sphere tree.
		Sphere* getRootSphere();
		//! This method computes all collisions between a segment passed as argument and the attributed 3D object.
		inline virtual bool computeCollision(cGenericObject* a_object,
			cVector3d& a_segmentPointA,
			cVector3d& a_segmentPointB,
			cCollisionRecorder& a_recorder,
			cCollisionSettings& a_settings) {
			return false;
		};

		// Update the inner sphere tree. !NOT IMPLEMENTED!
		inline virtual void update() {}

		//This method prints the AABB box tree for this mesh
		virtual void printAABBCollisionTree(int maxDiepte);

		// Print all the children of a specified sphere.
		void printChildren(Sphere* s);

		/*

			Get the size of this IST.

			\return The size of the IST.

		*/
		inline double getSize() { return size; }

		// DATA MEMBERS
	private:

		// The rootsphere.
		Sphere* rootSphere;

		//Set spheres to render in the vector spheres.
		std::vector<Sphere*> spheres;

		// De positie van het model.
		cVector3d positie;

		// De grootte van de bounding box.
		double size;

		// The previus display depth of the inner sphere tree.
		int prevDisplayDepth;

		// The node on which to begin checking.
		Sphere* beginNode;

		

		// The path to follow in the collision detection.
		// This is needed for a faster checking in collision.
		std::vector<Sphere*>* path;

		//the maximum depth of the tree
		int m_maxDepth;

		//base vectors for rotation
		cVector3d* b1;
		cVector3d* b2;
		cVector3d* b3;

		// PROTECTED FUNCTIONS
	public:

		inline cVector3d getB1() { return *b1; }
		inline cVector3d getB2() { return *b2; }
		inline cVector3d getB3() { return *b3; }

		/*

			Set the rotation of the IST.

			\param n The new rotation for the base vectors of the IST.

		*/
		inline void setRotation(cMatrix3d n) {
			double x = n.getRow(0).x();
			double y = n.getRow(1).x();
			double z = n.getRow(2).x();
			b1->set(x, y, z);
			x = n.getRow(0).y();
			y = n.getRow(1).y();
			z = n.getRow(2).y();
			b2->set(x, y, z);
			x = n.getRow(0).z();
			y = n.getRow(1).z();
			z = n.getRow(2).z();
			b3->set(x, y, z);

			//cout << *b1 << " - " << *b2 << " - " << *b3 << endl;
		}

		/*

			Set the size of the IST.

			\param n_size The new size of the IST.

		*/
		inline void setSize(double n_size) { size = n_size; }

		// This method is used to recursively build the collision tree.
		int buildTree(std::vector<Sphere*> leafs, const int a_depth);

		/*

			Get all the spheres of the IST.

			\return All the spheres.

		*/
		inline std::vector<Sphere*> getSpheres() { return spheres; }

		// An implementation of the BNG algorithm used.
		void BNG(double size, Sphere* node, std::vector<Sphere*> leafs, const int a_depth, Sphere* root);

		//Add leafs to a certain node.
		void addLeafs(std::vector<Sphere*> leafs, Sphere* node, Sphere* root);

		// Create a rootsphere.
		void maakRootSphere(std::vector<Sphere*> leafs);

		// Set which spheres to render.
		void setSpheresToRender(Sphere* s, std::vector<Sphere*>& spheres);

		/*

			Get the position of the IST.

			\return The position.

		*/
		inline cVector3d getPosition() { return positie; }

		/*

			Set the position of the IST.

			\param m_position The new position of the IST.

		*/
		inline void setPosition(cVector3d m_position) { positie = m_position; }

		/*

			Set the rootsphere of this IST.

			\n_root The new rootsphere of this IST.

		*/
		inline void setRoot(Sphere* n_root) { rootSphere = n_root; }

		/*

			Get the begin node of this IST.

			\return The begin node.

		*/
		inline Sphere* getBeginNode() { return beginNode; }

		/*

			Set the begin node of this IST.

			\param n_beginNode The new beginnode of this IST.

		*/
		inline void setBeginNode(Sphere* n_beginNode) { beginNode = n_beginNode; }

		/*

			Set the path of this IST.

			\param n_path The new path of this IST.

		*/
		inline void setPath(std::vector<Sphere*>* n_path) { path = n_path; }

		/*

			Get the path of this IST.

			\return The path of this IST.

		*/
		inline std::vector<Sphere*>* getPath() { return path; }

		//implementations for computeCollision chai3d
		/*enum cCollisionISTState
		{
			C_IST_STATE_TEST_CURRENT_NODE,
			C_IST_STATE_TEST_LEFT_NODE,
			C_IST_STATE_TEST_RIGHT_NODE,
			C_IST_STATE_POP_STACK
		};

		struct cCollisionISTStack
		{
			Sphere* m_index;
			cCollisionISTState m_state;
		};*/
	};
}

#endif