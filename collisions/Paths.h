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
		vector<vector<Sphere*>> pathsA;
		vector<vector<Sphere*>> pathsB;
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

		vector<Sphere*> getA(int index);
		vector<Sphere*> getB(int index);

		vector<Sphere*> getSpheresAtDepthA(int index);
		vector<Sphere*> getSpheresAtDepthB(int index);

		void popBackA(int index);
		void popBackB(int index);

		void pushBackA(Sphere* s, int index);
		void pushBackB(Sphere* s, int index);

		void pushBackA(vector<Sphere*> newPath);
		void pushBackB(vector<Sphere*> newPath);

		int size();

		void clearPositions();
		void addPosition(cVector3d pos);

	// PRIVATE METHODS
	private:
		vector<Sphere*> get(int index, bool a);
		vector<Sphere*> getSpheresAtDepth(int index, bool a);
		void popBack(int index, bool a);
		void pushBack(Sphere* s, int index, bool a);
		void pushBack(vector<Sphere*> newPath, bool a);

	// INLINE
	public:
		inline unsigned int getPositionSize() { return positions.size(); }
		inline cVector3d getPositions(unsigned int i) { 
			if (i < 0 || i >= positions.size()) {
				//cout << "error in Paths.h: index out of range!" << endl;
				return cVector3d(0, 0, 0);
			}
			return positions[i]; 
		}
		inline int getAantalVrijheidsgraden() { return aantalVrijheidsgraden; }

	};

} // -- chai3d

#endif
