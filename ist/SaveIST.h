#ifndef SAVEIST_H
#define SAVEIST_H

#include "ist/InnerSphereTree.h"
#include <iostream>
#include <fstream>

using namespace std;

namespace chai3d {

	ofstream istFile;

	void saveIst(InnerSphereTree* tree, string naam);
	void printSphere(Sphere* s);

	void saveIST(InnerSphereTree* tree, string naam) {

		string fileNaam = "ist" + naam + ".txt";
		istFile.open(fileNaam);

		istFile.clear();

		string sizeS = to_string(tree->getSize());

		istFile << sizeS << endl;

		printSphere(tree->getRootSphere());

		istFile.close();
		

	}

	void printSphere(Sphere* s) {
		cVector3d positie = s->getPosition();
		string positieX = to_string(positie.x());
		string positieY = to_string(positie.y());
		string positieZ = to_string(positie.z());

		int aantalKinderen = s->getChildren().size();
		string aantalKinderenS = to_string(aantalKinderen);

		double radius = s->getRadius();
		string radiusS = to_string(radius);

		int diepte = s->getDepth();
		string diepteS = to_string(diepte);

		int state = 0;
		if (s->getState() == sphereState::SPHERE_INTERNAL) state = 1;
		else if (s->getState() == sphereState::SPHERE_LEAF) state = 2;
		string stateS = to_string(state);

		istFile << positieX << " " << positieY << " " << positieZ << " " << radiusS << " " << " " << diepte << " " << stateS << " " << aantalKinderenS << endl;

		for (int i = 0; i < aantalKinderen; i++) {
			printSphere(s->getChildren()[i]);
		}

	}

} // chai3d

#endif