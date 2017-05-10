# Collision detection application
Uitbreiding op de CHAI3D library is hier af te halen: [Zip-bestand] (http://www.chai3d.org/download/releases).
## How to:
Opmerking: in het volgende is **object** is een pointer naar een **cMesh**.
- **Bouwen van een IST**
  Include volgende header files:

  **#include <ist/InnerSphereTree.h>
  #include <collisions/Voxelizer.h>
  #include "ist/SaveIST.h"**

  Gebruik volgende namespaces:

  **using namespace std;
  using namespace chai3d;**

  ---
  **_In de initialisatie stap van de main()_**
  ---

  1. Maak een AABBtree van het object

