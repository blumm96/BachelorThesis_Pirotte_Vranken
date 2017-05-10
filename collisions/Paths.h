#ifndef PATHS_H
#define PATHS_H

#include "math/CVector3d.h"
#include "ist/Sphere.h"
#include <vector>
#include <iostream>

using namespace std;

namespace chai3d {
	class Paths {

	// DATA MEMBERS
	private:
		int aantalVrijheidsgraden;
		vector<cVector3d> &positions = vector<cVector3d>();

	// CONSTRUCTOR - DESTRUCTOR
	public:
		Paths(int aantalVrijheidsgraden);
		~Paths();

	// PUBLIC METHODS
	public:
		//voor te testen, getters en setters moeten nog worden toegevoegd
		vector<Sphere*> &raakpuntenA = vector<Sphere*>();
		vector<Sphere*> &raakpuntenB = vector<Sphere*>();

		void clearPositions();
		void addPosition(cVector3d pos);

	// INLINE
	public:
		inline unsigned int getNumberOfCollisions() { return positions.size(); }
		inline cVector3d getCollision(unsigned int i) { 
			if (i < 0 || i >= positions.size()) {
				//cout << "error in Paths.h: index out of range!" << endl;
				return cVector3d(0, 0, 0);
			}
			return positions[i]; 
		}
		inline int getDegreesFreedom() { return aantalVrijheidsgraden; }

	};

} // -- chai3d

#endif
