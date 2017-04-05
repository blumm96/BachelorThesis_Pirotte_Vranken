#ifndef INNERSPHERETREE_H
#define INNERSPHERETREE_H

#include "collisions/CGenericCollision.h"
#include "ist/Sphere.h"

/*
	UHAS IMPLEMENTED

	\author Casper Vranken
	\author Niels Pirotte
*/

namespace chai3d {

	class InnerSphereTree : public cGenericCollision {

	// CONSTRUCTOR - DESTRUCTOR
	public:

		// Constructor of the inner sphere tree.
		InnerSphereTree();

		// Destructor of the inner sphere tree.
		virtual ~InnerSphereTree();

	// PUBLIC METHODS
	public:

		// Computes the collision between 2 inner sphere trees.
		virtual bool computeCollision(cGenericCollision* ist2, traversalSetting setting, double &collisionfeedback, int maxdiepte, cVector3d myLocal, cVector3d BLocal, cVector3d& positie);
		// Get the type of tree. In this case IST.
		virtual inline CollisionTreeType getCollisionTreeType() { return CollisionTreeType::IST; };
		// Render the leaf nodes.
		virtual void render(cRenderOptions& a_options);
		// Get the rootsphere of this inner sphere tree.
		Sphere* getRootSphere();
		//! This method computes all collisions between a segment passed as argument and the attributed 3D object.
		inline virtual bool computeCollision(cGenericObject* a_object,
			cVector3d& a_segmentPointA,
			cVector3d& a_segmentPointB,
			cCollisionRecorder& a_recorder,
			cCollisionSettings& a_settings)
		{
			return (false);
		}

		inline virtual void update() {}
		//UHAS implemented
		//This method prints the AABB box tree for this mesh
		virtual void printAABBCollisionTree(int maxDiepte);

		void printChildren(Sphere* s);

		inline double getSize() { return size; }

	// DATA MEMBERS
	private:

		// The rootsphere
		Sphere* rootSphere;

		//set spheres to render in the vector spheres
		std::vector<Sphere*> spheres;

		// De positie van het model.
		cVector3d positie;

		// De grootte van de bounding box.
		double size;

		int prevDisplayDepth;

		// The node on which to begin checking.
		Sphere* beginNode;

		std::vector<Sphere*>* path;

	// PROTECTED FUNCTIONS
	public:

		inline void setPositie(cVector3d n_positie) { positie = n_positie; }
		inline void setSize(double n_size) { size = n_size; }

		// This method is used to recursively build the collision tree.
		int buildTree(std::vector<Sphere*> leafs, const int a_depth);

		inline std::vector<Sphere*> getSpheres() { return spheres; }

		void BNG(double size, Sphere* node, std::vector<Sphere*> leafs, const int a_depth, Sphere* root);

		void addLeafs(std::vector<Sphere*> leafs, Sphere* node, Sphere* root);

		void maakRootSphere(std::vector<Sphere*> leafs);

		void setSpheresToRender(Sphere* s, std::vector<Sphere*>& spheres);

		inline cVector3d getPosition() { return positie; }
		inline void setPosition(cVector3d m_position) { positie = m_position; }

		inline void setRoot(Sphere* n_root) { rootSphere = n_root; }

		inline Sphere* getBeginNode() { return beginNode; }
		inline void setBeginNode(Sphere* n_beginNode) { beginNode = n_beginNode; }

		inline void setPath(std::vector<Sphere*>* n_path) { path = n_path; }
		inline std::vector<Sphere*>* getPath() { return path; }

	};
}

#endif