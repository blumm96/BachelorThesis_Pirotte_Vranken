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
    **Voxelizer\* voxelizerObject = new Voxelizer();**  
    **//maakt een distance-map tussen voxels en een object welke is voorgesteld in Box structuur (AABBtree)**  
    **//bouwt hieruit innerspheres op**  

  3. set het object in de voxelizer  
    **cCollisionAABB\* colliderObject = dynamic_cast<cCollisionAABB\*>(object->getCollisionDetector());**  
    **voxelizerObject->setObject(colliderObject);**  

  4. stel accuraatheid van de voxelizer in  
    **voxelizerObject->setAccuraatheid(n);**  
    **//deze accuraatheid (n) bepaalt de afstand tussen de voxels. Afstand tussen voxels in de x-richting (dx) = Lengte over de /x-as van de oriented bounding box van het object gedeeld door n.**  
    **//Analoog voor y en z richting.**  

  5. begin met distance mapping  
    **voxelizerObject->initialize();**  

  6. bouw de InnerSpheres op en hieruit wordt de IST opgebouwd m.b.v. het BNG algoritme  
    **InnerSphereTree\* istObject;**  
    **istObject = voxelizerObject->buildInnerTree(diepte, locale pos object, maximale lengte object);**  

  7. verwijder de voxelizer  
    **delete voxelizerObject;**  

  8. sla de InnerSphereTree op, zodat IST opbouw niet meer moet worden uitgevoerd, maar kan worden ingeladen  
    **saveIST(istObject, "filename");**  

  Stap 1-8 kan worden overgeslagen indien de IST al eerder berekend is en opgeslagen  
  **InnerSphereTree\* istObject;**  
  **istObject = loadIST("filename");**  

  9. De IST moet nu nog worden geset als de collision detector van het object  
    **onderkaak->setCollisionDetector(istOnderkaak);**  

- **Collision detection tussen 2 opgebouwde trees van 2 objecten (zelfde type collision detector)**
  ---
  **_In de haptic thread._**  
  ---
  
  **world->computeCollision(object1, object2, traversalSetting, distance, maxdiepte, \*position);**  
  //de traversalSetting bepaald het gebruikte algoritme voor het afgaan van de binary trees  
  //Deze algoritmes zijn gedefinieerd in de file: CollisionDetectionAlgorithms.h  
  //De huidige opties zijn:  
    * Distance -> algoritme 5.2 p.120 in het boek: __New Geometric Data Structures for Collision Detection and Haptics.__  
    * Volume Penetration (not yet implemented) -> algoritme 5.3 p123.  
    * Backward track -> houdt het vorige pad bij waaruit een collisie volgde en zoekt vanuit dit pad opnieuw naar collisies.  
    * Multipoint -> houdt rekening met mogelijk meerdere collisies en werkt analoog met optie c. Er kan een splitting depth worden ingesteld door de **#define splitDeth 2** aan te passen vanboven in de CollisionDetectionAlgorithms.h header file. Deze bepaalt op welke diepte raakpunten niet meer dezelfde parent mogen hebben. Het aantal collisies kan worden opgevraagd met:  
      **InnerSphereTree::globalPath.getNumberOfCollisions();**  
      Waarna elke positie van collisie kan worden opgevraagd met:  
        **InnerSphereTree::globalPath.getCollisions(index);  //deze geeft een cVector3d terug**  

Wanneer de objecten worden bewogen of worden geroteerd moet dit wel worden doorgegeven aan de collision detectors:  
**istObject->setRotation(object->getLocalRot());  
istObject->setPosition(object->getLocalPos());**  


  

