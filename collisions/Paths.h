#ifndef PATHS_H
#define PATHS_H

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

	// CONSTRUCTOR - DESTRUCTOR
	public:
		Paths(int aantalVrijheidsgraden);
		~Paths();

	// PUBLIC METHODS
	public:
		vector<Sphere*> getA(int index);
		vector<Sphere*> getB(int index);

		vector<Sphere*> getSpheresAtDepthA(int index);
		vector<Sphere*> getSpheresAtDepthB(int index);

		void popBackA(int index);
		void popBackB(int index);

		void pushBackA(Sphere* s, int index);
		void pushBackB(Sphere* s, int index);

	// PRIVATE METHODS
	private:
		vector<Sphere*> get(int index, bool a);
		vector<Sphere*> getSpheresAtDepth(int index, bool a);
		void popBack(int index, bool a);
		void pushBack(Sphere* s, int index, bool a);

	};

} // -- chai3d

#endif
