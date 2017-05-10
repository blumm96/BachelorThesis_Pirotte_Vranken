//==============================================================================
/*
This class provides feedback for the multipoint algotihm.

\author    Niels Pirotte
\author    Casper Vranken
\version   1.0.0
*/
//==============================================================================

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
		//Interface with colliding spheres found by multipoint algorithm
		vector<Sphere*> &raakpuntenA = vector<Sphere*>();
		vector<Sphere*> &raakpuntenB = vector<Sphere*>();

		//Clear all contact points found by multipoint algorithm
		void clearPositions();
		//Add contact point
		void addPosition(cVector3d pos);

	// INLINE
	public:
		// Returns number of collisions found by multipoint algorithm
		inline unsigned int getNumberOfCollisions() { return positions.size(); }

		//Get a contact point
		inline cVector3d getCollision(unsigned int i) { 
			if (i < 0 || i >= positions.size()) {
				//cout << "error in Paths.h: index out of range!" << endl;
				return cVector3d(0, 0, 0);
			}
			return positions[i]; 
		}

		//Returns the number of contact points configured for the multipoint algorithm
		inline int getDegreesFreedom() { return aantalVrijheidsgraden; }
	};

} // -- chai3d

#endif
