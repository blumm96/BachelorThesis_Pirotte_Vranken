#ifndef SAVEIST_H
#define SAVEIST_H

#include "ist/InnerSphereTree.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <math.h>

using namespace std;

namespace chai3d {

	fstream istFile;

	void saveIST(InnerSphereTree* tree, string naam);
	void printSphere(Sphere* s);
	InnerSphereTree* loadIST(string naam);
	void readSphere(Sphere* parent, Sphere* root);
	Sphere* processString(string lijn);
	unsigned int split(const string &txt, vector<string> &strs, char ch);

	/*
	
		Saves an IST.

		\param tree The IST to save.
		\param naam The name of the tree. (can be whatever)

	*/
	void saveIST(InnerSphereTree* tree, string naam) {

		string fileNaam = "ist" + naam + ".txt";
		istFile.open(fileNaam);

		istFile.clear();

		string sizeS = to_string(tree->getSize());

		istFile << sizeS << endl;

		printSphere(tree->getRootSphere());

		istFile.close();
		

	}

	/*
		
		Write the data of a sphere to the file specified before.

		\param s The sphere to save.

	*/
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

		istFile << positieX << " " << positieY << " " << positieZ << " " << radiusS << " " << diepte << " " << stateS << " " << aantalKinderenS << endl;

		for (int i = 0; i < aantalKinderen; i++) {
			printSphere(s->getChildren()[i]);
		}

	}

	/*
		
		Load an IST.

		\param naam The name of the IST. (must be the previously choses name for the IST).
		
		\return The loaded IST.

	*/
	InnerSphereTree* loadIST(string naam) {

		string fileNaam = "ist" + naam + ".txt";
		istFile.open(fileNaam);

		string lijn;
		getline(istFile, lijn);
		double size = stod(lijn);
		getline(istFile, lijn);

		Sphere* root = processString(lijn);

		readSphere(root, root);

		istFile.close();

		InnerSphereTree* tree = new InnerSphereTree();
		tree->setSize(size);
		tree->setRoot(root);
		tree->setPositie(cVector3d(0, 0, 0));

		return tree;

	}

	/*
		
		Read a sphere. This function is called recursively.
		Call this function for the root sphere.

		\param parent The sphere from which to start.
		\param root The rootsphere.
		
		\return One of the children of the parent.

	*/
	void readSphere(Sphere* parent, Sphere* root) {

		string lijn;

		for (int i = 0; i < parent->getChildrenAmount(); i++) {

			getline(istFile, lijn);

			Sphere* s = processString(lijn);
			parent->addChild(s);
			s->setParent(parent);
			s->setRootSphere(root);

			readSphere(s, root);

		}

	}

	/*
		
		Process a string to get a sphere out of it.

		\param lijn The string to process. All data is sperated by ' '

		\return The sphere that has been processed.

	*/
	Sphere* processString(string lijn) {

		vector<string> data;
		split(lijn, data, ' ');

		double positieX = stod(data[0]);
		double positieY = stod(data[1]);
		double positieZ = stod(data[2]);

		float radius = stof(data[3]);
		int diepte = stoi(data[4]);
		int state = stoi(data[5]);
		int aantalKinderen = stoi(data[6]);

		Sphere* sphere = new Sphere();
		cVector3d vec = cVector3d(positieX, positieY, positieZ);

		sphere->setPosition(vec);
		sphere->setRadius(radius);
		sphere->setDepth(diepte);
		switch (state){
			case 0: sphere->setState(sphereState::SPHERE_ROOT);
				break;
			case 1: sphere->setState(sphereState::SPHERE_INTERNAL);
				break;
			case 2: sphere->setState(sphereState::SPHERE_LEAF);
				break;
			default: cout << ("Error in reading file.") << endl;
				break;
		}

		sphere->setChildrenAmount(aantalKinderen);

		return sphere;

	}

	/*
		
		Taken from: http://stackoverflow.com/questions/5888022/split-string-by-single-spaces

	*/
	unsigned int split(const string &txt, vector<string> &strs, char ch)
	{
		unsigned int pos = txt.find(ch);
		unsigned int initialPos = 0;
		strs.clear();

		// Decompose statement
		while (pos != string::npos) {
			strs.push_back(txt.substr(initialPos, pos - initialPos + 1));
			initialPos = pos + 1;

			pos = txt.find(ch, initialPos);
		}

		// Add the last one
		strs.push_back(txt.substr(initialPos, min(pos, txt.size()) - initialPos + 1));

		return strs.size();
	}

} // chai3d

#endif