# Collision detection application
Uitbreiding op de CHAI3D library is hier af te halen: [Zip-bestand] (http://www.chai3d.org/download/releases/).
## How to:
Opmerking: in het volgende is **object** is een pointer naar een **cMesh**.
- **Bouwen van een IST**
  Include volgende header files:

  **#include <ist/InnerSphereTree.h>**  
  **#include <collisions/Voxelizer.h>**  
  **#include "ist/SaveIST.h"**  

  Gebruik volgende namespaces:

  **using namespace std;  
  using namespace chai3d;**  

  ---
  **_In de initialisatie stap van de main()_**
  ---

  1. Maak een AABBtree van het object
    **object->createAABBCollisionDetector(double radius);**
    **//creëert een box hiërarchie**
    **//radius => bij opbouw van een box rond een vertex wordt deze box een kubus met ribbe gelijk aan radius**

  2. maak een voxelizer aan voor het object
    **Voxelizer* voxelizerObject = new Voxelizer();**
    **//maakt een distance-map tussen voxels en een object welke is voorgesteld in Box structuur (AABBtree)**
    **//bouwt hieruit innerspheres op**

  3. set het object in de voxelizer
    **cCollisionAABB* colliderObject = dynamic_cast<cCollisionAABB*>(object->getCollisionDetector());**
    **voxelizerObject->setObject(colliderObject);**

  4. stel accuraatheid van de voxelizer in
    **voxelizerObject->setAccuraatheid(n);**
    **//deze accuraatheid (n) bepaalt de afstand tussen de voxels. Afstand tussen voxels in de x-richting (dx) = Lengte over de /x-as van de oriented bounding box van het object gedeeld door n.**
    **//Analoog voor y en z richting.**

  5. begin met distance mapping
    **voxelizerObject->initialize();**

  6. bouw de InnerSpheres op en hieruit wordt de IST opgebouwd m.b.v. het BNG algoritme
    **InnerSphereTree* istObject;**
    **istObject = voxelizerObject->buildInnerTree(diepte, locale pos object, maximale lengte object);**

  7. verwijder de voxelizer
    **delete voxelizerObject;**

  8. sla de InnerSphereTree op, zodat IST opbouw niet meer moet worden uitgevoerd, maar kan worden ingeladen
    **saveIST(istObject, "filename");**

  Stap 1-8 kan worden overgeslagen indien de IST al eerder berekend is en opgeslagen
  **InnerSphereTree* istObject;**
  **istObject = loadIST("filename");**

  9. De IST moet nu nog worden geset als de collision detector van het object
    **onderkaak->setCollisionDetector(istOnderkaak);**


    
