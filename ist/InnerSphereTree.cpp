#include "ist/InnerSphereTree.h"
#include "collisions/CollisionDetectionAlgorithms.h"
#include "collisions/Paths.h"
#include <iostream>
#include <math.h>
#include <limits>

using namespace std;

namespace chai3d {

	//initialize static
	Paths InnerSphereTree::globalPath = Paths(3);
	/*
		Constructor of an inner sphere tree.
	*/
	InnerSphereTree::InnerSphereTree() {
		prevDisplayDepth = m_displayDepth;
		path = new vector<Sphere*>();

		b1 = new cVector3d(1.0, 0.0, 0.0);
		b2 = new cVector3d(0.0, 1.0, 0.0);
		b3 = new cVector3d(0.0, 0.0, 1.0);
	}

	/*
		Destructor of the inner sphere tree.
	*/
	InnerSphereTree::~InnerSphereTree() {
		delete rootSphere;
		delete b1;
		delete b2;
		delete b3;
	}

	/*
		
		Print the entire collision tree.

		\param diepte The maximum depth the tree should be printed.

	*/
	void InnerSphereTree::printAABBCollisionTree(int diepte) {
		cout << endl;
		cout << "INNER SPHERE TREE" << endl;
		
		this->printChildren(rootSphere);

	}

	/*
	
		Print the children of a specified sphere.

		\param s The sphere of whom children should be printed.

	*/
	void InnerSphereTree::printChildren(Sphere* s) {

		for (int i = 0; i < s->getDepth(); i++) {
			cout << "\t";
		}

		cout << "Radius: " << s->getRadius() << " Positie: " << s->getPosition() << endl;
		for (int i = 0; i < s->getChildren().size(); i++) {
			if (s->getChildren()[i]->getState() != sphereState::SPHERE_LEAF) {
				this->printChildren(s->getChildren()[i]);
			}
			else {
				for (int j = 0; j < s->getChildren()[i]->getDepth(); j++) {
					cout << "\t";
				}
				cout << "Radius: " << s->getChildren()[i]->getRadius() << " Positie: " << s->getChildren()[i]->getPosition() << endl;
			}
		}

	}

	/*
		This function computes if a collision has occured between two inner sphere trees.

		\param ist2					he other inner sphere tree to compute the possible collision.
		\param setting				The traversalsettings.
		\param collisionfeedback	How close the collision is.
		\param maxdiepte			The maximumdepth the function is allowed to check for collision.
		\param myLocal				The local position of this IST.
		\param BLocal				The local position of ist2.
		\param positie				This parameter will be set to the position of the collision. It remains
									unchanged if no collision occurs.

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

			Sphere* parent_A = IST_A->getRootSphere();
			Sphere* parent_B = IST_B->getRootSphere();

			bool stop = false;

			// Komt uit Collision detection algorithms
			//collisionfeedback = checkDistanceSphere(parent_A, parent_B, IST_A, IST_B, maxdiepte, stop, positie);
			collisionfeedback = checkDistanceSphere(parent_A, parent_B, IST_A, IST_B, maxdiepte, stop, positie);

			//std::cout << collisionfeedback << std::endl;
			//if (collisionfeedback <= 0) return true;
			if (collisionfeedback == 0.0f) return true;
			else return false;
		}
		case traversalSetting::BACKWARDTRACK: {
			InnerSphereTree* IST_B = dynamic_cast<InnerSphereTree*>(ist2);
			InnerSphereTree* IST_A = this;

			Sphere* parent_A = IST_A->getRootSphere();
			Sphere* parent_B = IST_B->getRootSphere();

			// Komt uit Collision detection algorithms

			bool stop = false;

			vector<Sphere*> visNodesA = vector<Sphere*>();
			vector<Sphere*> visNodesB = vector<Sphere*>();
			collisionfeedback = checkDistanceSphere2(parent_A, parent_B, IST_A, IST_B, maxdiepte, stop, positie, (IST_A->getPath()), (IST_B->getPath()), nullptr, nullptr, &visNodesA, &visNodesB, false);
			/*checkDistanceSphere2(parent_A, parent_B, mindist, IST_A, IST_B, maxdiepte);
			collisionfeedback = mindist;*/

			//std::cout << collisionfeedback << std::endl;
			if (collisionfeedback <= 0) return true;
			else return false;
		
		};
		case traversalSetting::MULTIPOINT: {
			InnerSphereTree* IST_B = dynamic_cast<InnerSphereTree*>(ist2);
			InnerSphereTree* IST_A = this;

			Sphere* parent_A = IST_A->getRootSphere();
			Sphere* parent_B = IST_B->getRootSphere();

			// Komt uit Collision detection algorithms
			float schattingD = std::numeric_limits<float>::infinity();
			bool hit = checkDistanceSphereMultipoint(IST_A, IST_B, maxdiepte, InnerSphereTree::globalPath, schattingD);
			collisionfeedback = schattingD;
			
			if (hit) return true;
			return false;
		};
		case traversalSetting::ACCURATE: {
			float kortste = numeric_limits<float>::infinity();
			float verste = 0;

			InnerSphereTree* IST_B = dynamic_cast<InnerSphereTree*>(ist2);
			InnerSphereTree* IST_A = this;

			Sphere* parent_A = IST_A->getRootSphere();
			Sphere* parent_B = IST_B->getRootSphere();

			accurateCheck(IST_A, IST_B, parent_A, parent_B, kortste, verste);

			positie.set(kortste, verste, -1);

			return true;

		};
		case traversalSetting::VOLUME_PEN: return false;
		default: return false;
		}
		return true;
	}

	/*
		
		This function calculates a collision between 2 ists.

		\param ist2					The second ist to check collision with.
		\param setting				The settings for the collision detection.
		\param collisionFeedback	How far the ists are apart.
		\param maxdiepte			The maximum depth the algorithm shoud check.
		\param positie				The position the collision happens at.
		\param pa					The first sphere to start collision detection.
		\param pb					The second sphere to start collision detection.

		\return If the two ists are colliding.

	*/
	bool InnerSphereTree::computeCollision(InnerSphereTree* ist2, traversalSetting setting, double &collisionfeedback, int maxdiepte, cVector3d& positie, Sphere* &pA, Sphere* &pB) {
		// Sanity check
		if (ist2 == NULL) return false;
		if (this->getCollisionTreeType() != ist2->getCollisionTreeType()) return false;

		switch (setting) {
		case traversalSetting::DISTANCE: {

			Sphere* parent_A = this->getRootSphere();
			Sphere* parent_B = ist2->getRootSphere();

			bool stop = false;

			// Komt uit Collision detection algorithms
			//collisionfeedback = checkDistanceSphere(parent_A, parent_B, IST_A, IST_B, maxdiepte, stop, positie);
			float mindist;
			if (pA == nullptr && pB == nullptr) mindist = std::numeric_limits<float>::infinity();
			else mindist = pA->distance(pB, this, ist2);

			int test = 0;
			checkDistanceSphereTest(parent_A, parent_B, this, ist2, mindist, maxdiepte, stop, positie, pA, pB, test);

			collisionfeedback = mindist;
			//std::cout << collisionfeedback << std::endl;

			//if (collisionfeedback <= 0) return true;
			if (mindist == 0.0f) return true;
			else return false;
		}
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

	/*
		
		Build a tree out of leafs with a specified depth.

		\param leafs	The leafs on which to base the IST.
		\param a_depth	The depth that the IST should have.

		\return			0.

	*/
	int InnerSphereTree::buildTree(std::vector<Sphere*> leafs, const int a_depth)
	{	
		//set spheres to render in the vector spheres
		for (int i = 0; i < leafs.size(); i++) {
			spheres.push_back(leafs[i]);
		}

		maakRootSphere(leafs);

		BNG(size, rootSphere, leafs, a_depth);

		return 0;
	}

	/*
		
		Implementation of the BNG algorithm.

		\param size			The size of the boundingbox of the object for determining the stopcriteria.
		\param node			The node on which to build the following branch.
		\param leafs		The remaining leafs on which to build the following internal sphere.
		\param a_depth		The maximum depth of the IST.
		\param root			The rootsphere.

	*/
	void InnerSphereTree::BNG(double size, Sphere* node, std::vector<Sphere*> leafs, const int a_depth)
	{
#define TMAX 500		
		//als we de diepte hebben bereikt dan moeten we de kinderen nog toevoegen
		//cout << "diepte: " << node->getDepth() << " - aantal leafs: " << leafs.size() << endl;

		if (node->getDepth() == a_depth) {
			addLeafs(leafs, node);
			m_maxDepth = a_depth;
			return;
		}

		struct prototype {
			cVector3d pos;
			std::vector<Sphere*> lfs;
		};

		prototype w[4];

		double x = node->getPosition().x();
		double y = node->getPosition().y();
		double z = node->getPosition().z();

		double r = node->getRadius();
		
		//chose start pos prototypes
		w[0].pos.set(x + r, y + r, z);
		w[1].pos.set(x + r, y - r, z + r);
		w[2].pos.set(x - r, y + r, z - r);
		w[3].pos.set(x - r, y - r, z - r);

		//define epsilon
		double eps = 0.0001 * size;
		//first index of weights is the number of the leaf
		std::vector<std::vector<int>> weights;
		//row with prototypes per leaf
		std::vector<int> row;

		int t = 0;
		bool stop = false;

		while (!stop && (t <= TMAX)) {
			int teller = 0;
			//for every sphere we calculate the distance to each prototype and decide the weights 
			for (int j = 0; j < leafs.size(); j++) {
				double d[4];
				int n[4] = { 0,0,0,0 };

				// Posities van de leafs zijn relatief tegenover de rootsphere.
				// Bereken afstand tussen protoype en leafs
				d[0] = (leafs[j]->getPosition() - w[0].pos).length();
				d[1] = (leafs[j]->getPosition() - w[1].pos).length();
				d[2] = (leafs[j]->getPosition() - w[2].pos).length();
				d[3] = (leafs[j]->getPosition() - w[3].pos).length();

				for (int i = 0; i < 4; i++) {
					for (int k = i + 1; k < 4; k++) {
						if (d[i] < d[k]) n[k]++;
						else n[i]++;
					}
				}
				row.clear();
				for (int i = 0; i < 4; i++) row.push_back(n[i]);
				//first index of weights is the number of the leaf
				weights.push_back(row);
			}

			//calculate new prototype positions
			float L = 2 * pow((0.01 / 2.0), ((double)t / (double)TMAX));
			for (int k = 0; k < 4; k++) {
				double sumf = 0;
				cVector3d sumv = cVector3d(0, 0, 0);
				for (int i = 0; i < leafs.size(); i++) {
					float volume = pow(leafs[i]->getRadius(), 3.0)*(4.0 / 3.0)* M_PI;

					float hL = exp(-weights[i][k] / L);
					float f = hL*volume;
					cVector3d vec = f*(leafs[i]->getPosition());

					sumf += f;
					sumv += vec;
				}
				sumv = sumv / sumf;
				if((w[k].pos - sumv).length() < eps) teller++;
				if (teller == 4) {
					stop = true;
				}
				w[k].pos = sumv;
			}
			t++;
		}

		float max[4] = { 0,0,0,0 };
		for (int j = 0; j < leafs.size(); j++) {
			float mindist = numeric_limits<float>::infinity();
			float rad = leafs[j]->getRadius();
			int num;
			for (int i = 0; i < 4; i++) {
				float d = ((leafs[j]->getPosition() - w[i].pos).length());
				if (d < mindist) {
					mindist = d;
					num = i;
				}
			}
			if (max[num] < (mindist + rad)) max[num] = (mindist + rad);
			w[num].lfs.push_back(leafs[j]);
		}

		//? set new position npos
		for (int i = 0; i < 4; i++) {
			//calculate pos
			max[i] = 0;
			cVector3d npos = cVector3d(0.0, 0.0, 0.0);
			for (int z = 0; z < w[i].lfs.size(); z++) {
				npos += w[i].lfs[z]->getPosition();
			}
			w[i].pos = npos / (float)(w[i].lfs.size());

			for (int z = 0; z < w[i].lfs.size(); z++) {
				float d = (w[i].pos - w[i].lfs[z]->getPosition()).length();
				if (max[i] < (w[i].lfs[z]->getRadius() + d)) max[i] = (w[i].lfs[z]->getRadius() + d);
			}
		}
			
			//we got all wheights with a vector to their leaves
			//with all including radius of their leaves
		for (int i = 0; i < 4; i++) {
			if (node->getDepth() == 1) {
				for (int j = 0; j < w[i].lfs.size(); j++) w[i].lfs[j]->color = i+1;
			}
			if (w[i].lfs.empty()) continue;
			Sphere* s = new Sphere();
			s->setPosition(w[i].pos);
			s->setRadius(max[i]);
			s->setState(sphereState::SPHERE_INTERNAL);
			s->setDepth(node->getDepth()+1);
			s->setParent(node);
			s->setRootSphere(rootSphere);
			
			//set as child of node
			node->addChild(s);
			//recursive call
			BNG(size, s, w[i].lfs, a_depth);
		}
	}

	/*
		
		Add all the leafs to a specified node.

		\param leafs	The leafs that should be added to the node.
		\param node		The node on which to add the leafs.
		\param root		The rootsphere.

	*/
	void InnerSphereTree::addLeafs(std::vector<Sphere*> leafs, Sphere * node)
	{
		for (int i = 0; i < leafs.size(); i++) {
			leafs[i]->setParent(node);
			leafs[i]->setDepth(node->getDepth() + 1);
			leafs[i]->setRootSphere(rootSphere);
			node->addChild(leafs[i]);
		}
	}

	/*
		
		Generate a rootsphere.

		\param leafs The leafs of the IST. The rootsphere will surround all of the leafs.

	*/
	void InnerSphereTree::maakRootSphere(std::vector<Sphere*> leafs) {

		double maxD = 0;

		for (int i = 0; i < leafs.size(); i++) {

			double afstand = leafs[i]->getPosition().length() + leafs[i]->getRadius();
			if (afstand > maxD) {
				maxD = afstand;
			}
		}

		Sphere *root = new Sphere();
		root->setRadius((float)maxD);
		root->setParent(NULL);
		root->setState(sphereState::SPHERE_ROOT);
		root->setPosition(cVector3d(0,0,0));
		root->setDepth(0);

		rootSphere = root;
	}

	/*
		
		Set which sphers to render.

		\param s				The sphere that should be rendered.
		\param spheresToDraw	The vector that contains all the spheres to render. It will be adapted.

	*/
	void InnerSphereTree::setSpheresToRender(Sphere* s, std::vector<Sphere*>& spheresToDraw)
	{	
		if (s->getDepth() == m_displayDepth || s->getState() == sphereState::SPHERE_LEAF) {
			spheresToDraw.push_back(s);
			return;
		}
		for (int i = 0; i < s->getChildren().size(); i++) {
			setSpheresToRender(s->getChildren()[i], spheresToDraw);
		}
	}
	
	/*
		
		Render the IST.

		\param a_options The options to render the IST.

	*/
	void InnerSphereTree::render(cRenderOptions& a_options) {
#ifdef C_USE_OPENGL
		
		//set rendering settings
		glDisable(GL_LIGHTING);
		glLineWidth(1.0);
		glColor4fv(m_color.getData());

		if (prevDisplayDepth != m_displayDepth) {
			spheres.clear();
			setSpheresToRender(rootSphere, spheres); //command this line out to render all leafs
			prevDisplayDepth = m_displayDepth;
		}
		
		for (int i = 0; i < spheres.size(); i++) {
			spheres[i]->render();
		}

		glEnable(GL_LIGHTING);
#endif
	}
}