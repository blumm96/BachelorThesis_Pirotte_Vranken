# Collision detection application
Uitbreiding op de CHAI3D library is hier af te halen: [Zip-bestand] (http://www.chai3d.org/download/releases/).
## How to:
Opmerking: in het volgende is **object** is een pointer naar een **cMesh**.
- **Bouwen van een IST**
  Include volgende header files:  
```c
#include <ist/InnerSphereTree.h>  
#include <collisions/Voxelizer.h>  
#include "ist/SaveIST.h"  
```
  Gebruik volgende namespaces:  
```c
using namespace std;  
using namespace chai3d;  
```
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
    **//deze accuraatheid (n) bepaalt de afstand tussen de voxels.  
    //Afstand tussen voxels in de x-richting (dx) = Lengte over de x-as van de oriented bounding box van het object gedeeld door n.**  
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
```c  
world->computeCollision(object1, object2, traversalSetting, distance, maxdiepte, *position);
```
  de traversalSetting bepaald het gebruikte algoritme voor het afgaan van de binary trees  
  Deze algoritmes zijn gedefinieerd in de file: CollisionDetectionAlgorithms.h  
  De huidige opties zijn:  
    * Distance -> algoritme 5.2 p.120 in het boek: __New Geometric Data Structures for Collision Detection and Haptics.__  
    * Volume Penetration (not yet implemented) -> algoritme 5.3 p123.  
    * Backward track -> houdt het vorige pad bij waaruit een collisie volgde en zoekt vanuit dit pad opnieuw naar collisies.  
    * Multipoint -> houdt rekening met mogelijk meerdere collisies en werkt analoog met optie c. Er kan een splitting depth worden ingesteld door de **#define splitDeth 2** aan te passen vanboven in de CollisionDetectionAlgorithms.h header file. Deze bepaalt op welke diepte raakpunten niet meer dezelfde parent mogen hebben. Het aantal collisies kan worden opgevraagd met:  

```c
InnerSphereTree::globalPath.getNumberOfCollisions();  
```
Waarna elke positie van collisie kan worden opgevraagd met:  
```c
InnerSphereTree::globalPath.getCollisions(index);  //deze geeft een cVector3d terug  
```
Wanneer de objecten worden bewogen of worden geroteerd moet dit wel worden doorgegeven aan de collision detectors:  
```c
istObject->setRotation(object->getLocalRot());  
istObject->setPosition(object->getLocalPos());  
```
- **Collision detection a.d.h.v. de PQP library – (https://github.com/GammaUNC/PQP)**  
  Include volgende header file:
```c
#include "PQP/PQP.h"  
```
  Maak volgende globale variabelen aan:  
```c
//collision detection with pqp lib  
PQP_Model* m1;  
PQP_Model* m2;    
//position and rotation of m1 for pqp collision detection  
PQP_REAL T1[3];  
PQP_REAL R1[3][3];    
//position and rotation of m2 for pqp collision detection  
PQP_REAL T2[3];  
PQP_REAL R2[3][3];    
```  
---
**_In de initialisatie stap van de main()_**   

---

Laat de modellen m1 en m2 in bij het inladen van de mesh  
```c
m1 = new PQP_Model();  
m2 = new PQP_Model();    
bool fileload;  
fileload = bovenkaak->loadFromFile2(RESOURCE_PATH("Path_model1"), \*m1);  
fileload = bovenkaak->loadFromFile2(RESOURCE_PATH("Path_model2"), \*m2);  
//Er worden nu PQP modellen gemaakt vanuit de stl files.  
```  
---
**_In de Haptic Thread while loop:_**  

---

Voor een distance query zoals beschreven op github:  
```c
double distance_pqp;  
setPosAndRot1();  
setPosAndRot2();    
//distance with pqp lib  
PQP_DistanceResult dres;  
double rel_err = 0.0, abs_err = 0.0;  
PQP_Distance(&dres, R1, T1, m1, R2, T2, m2, rel_err, abs_err);  
distance_pqp = dres.Distance();  
```  
Voor een collision query zoals beschreven op github  
```c
int colliding;  
//colliding querry with pqp  
PQP_CollideResult cres;  
QP_Collide(&cres, R1, T1, m1, R2, T2, m2);  
colliding = cres.Colliding();    
```
Bij elke beweging of rotatie moeten zoals bij collisie detectie met ISTs of AABBs ook de positie en de rotatie van de modellen worden geset:  
```c	
setPosAndRot(T1, R1, object1);  
setPosAndRot(T2, R2, object2);  
```
- **Hoe grotere snelheden behalen:**  
Door in de Haptic thread enkel naar collisies te zoeken wanneer een zekere minimum afstand is afgelegd. We voeren een globale variabele in:  
```c
cVector3d traveledDistance;**  
```
Bij elke verplaatsing wordt deze vector aangepast:  
```c
traveledDistance += displacement;  
```

---
**_In de Haptic Thread_**  
  
---
We make een variabele aan:  
```c
float minDist = 0;  
```
  
---  
**_In de Haptic loop krijgen (while lus in Haptic Thread)_**  
  
---

```c
If(traveledDistance.length() > minDist){  
	//Doe collisie detectie => een schatting van de min. afstand door het collisie detectie algoritme = schattingD  
	minDist = schattingD;  
	traveledDistance->zero();  
}  
```

  
  


