# Collision detection application
The CHAI3D-library can be downloaded here: http://www.chai3d.org/download/releases/ .
## How to:
Note: in the following text **object** is a pointer to a **cMesh**.
- **Building an IST**
  Include following header files:  
```c
#include <ist/InnerSphereTree.h>  
#include <collisions/Voxelizer.h>  
#include "ist/SaveIST.h"  
```
  Use following namespaces:  
```c
using namespace std;  
using namespace chai3d;  
```
---
**_In the initialisation of the main()_**  

---

  1. Maak an AABBtree of the object.  
    **object->createAABBCollisionDetector(double radius);**  
    **//creates a box-hierarchy**  
    **//radius => if building a box around a vertex, this box will become a cube with the sides equal to the radius**

  2. Make a voxelizer for the object.
    **Voxelizer\* voxelizerObject = new Voxelizer();**  
    **//Make a distance map between the voxels and the object which is represented in the box-hierarchy(AABBtree)**  
    **//Build inner sphere trees out of this.**

  3. Set the object in the voxelizer.
    **cCollisionAABB\* colliderObject = dynamic_cast<cCollisionAABB\*>(object->getCollisionDetector());**  
    **voxelizerObject->setObject(colliderObject);**  

  4. Set the accuracy of the voxelizer.  
    **voxelizerObject->setAccuraatheid(n);**  
    **//This accuracy (n) determines the distance between the voxels.  
    //The distance between the voxels in the x-direction (dx) is equal to the length in the x-direction of the oriented bounding box of the object divided by n.**  
    **//The same goes for the y- and z-direction.**  

  5. Start the distance mapping.
    **voxelizerObject->initialize();**  

  6. Build the inner sphere trees, and out of this build the inner sphere trees with the BNG algorithm.
    **InnerSphereTree\* istObject;**  
    **istObject = voxelizerObject->buildInnerTree(depth, local pos object, maximum length object);**  

  7. Delete the voxelizer.
    **delete voxelizerObject;**  

  8. Save the inner sphere tree, this way the IST only has to be build once and can be loaded afterwards. 
    **saveIST(istObject, "filename");**  

  Step 1-8 can be skipped if the IST has been build and saved before.
  **InnerSphereTree\* istObject;**  
  **istObject = loadIST("filename");**  

  9. The IST now only has te be set as the collision detector of the object.  
    **lowerJaw->setCollisionDetector(istLowerJaw);**  

- **Collision detection between two build trees of two objects (same type of collision detector)**  
  
---
**_In the haptics thread._**  

---
```c  
world->computeCollision(object1, object2, traversalSetting, distance, maxdiepte, *position);
```
  Thee traversalSetting determins the used algorithm to traverse the binary tree.  
  These algorithms are defined in the following file: CollisionDetectionAlgorithms.h  
  The current options are:  
    * Distance -> algorithm 5.2 p.120 in: __New Geometric Data Structures for Collision Detection and Haptics.__  
    * Volume Penetration (not yet implemented) -> algorithm 5.3 p123.  
    * Backward track -> saves the path of the previous collision and uses this path to check for a new collision. This results in a faster collision detection.  
    * Multipoint -> searches for multiple points and works in a same manner as c). A splitting depth can be set by altering the **#define splitDeth 2** in the top of the CollisionDetectionAlgorithms.h header file. This determins at which depth the collision point can't have the same parent. The number of collisions can be get by:  

```c
InnerSphereTree::globalPath.getNumberOfCollisions();  
```
After which the position of every collision can be get with:  
```c
InnerSphereTree::globalPath.getCollisions(index);  //This returns a cVector3d.  
```
When the objects move or rotate, the collision detectors have to be updated:  
```c
istObject->setRotation(object->getLocalRot());  
istObject->setPosition(object->getLocalPos());  
```
- **Collision detection with the PQP library – (https://github.com/GammaUNC/PQP)**  
  Include the following header file:
```c
#include "PQP/PQP.h"  
```
  Make the following global variables:  
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
**_In the initialisation step of the main()_**   

---

Load the models m1 and m2 while loading in the meshes.  
```c
m1 = new PQP_Model();  
m2 = new PQP_Model();    
bool fileload;  
fileload = bovenkaak->loadFromFile2(RESOURCE_PATH("Path_model1"), \*m1);  
fileload = bovenkaak->loadFromFile2(RESOURCE_PATH("Path_model2"), \*m2);  
//PQP models can be made from the STL-files now.  
```  
---
**_Meanwhile in the haptics thread:_**  

---

For a distance query as described on github:  
```c
double distance_pqp;  
   
//distance with pqp lib  
PQP_DistanceResult dres;  
double rel_err = 0.0, abs_err = 0.0;  
PQP_Distance(&dres, R1, T1, m1, R2, T2, m2, rel_err, abs_err);  
distance_pqp = dres.Distance();  
```  
For a collision query as described on github:  
```c
int colliding;  
//colliding querry with pqp  
PQP_CollideResult cres;  
QP_Collide(&cres, R1, T1, m1, R2, T2, m2);  
colliding = cres.Colliding();    
```
With every movement or rotation, we have to, just likes with collision detection with ISTs or AABBs, set the position and rotation of the models.  
```c	
setPosAndRot(T1, R1, object1);  
setPosAndRot(T2, R2, object2);  
```
- **How to achieve greater speeds:**  
Only search for collisions in the haptics thread when the models have moved a certain distance. We add a global variable:  
```c
cVector3d traveledDistance;  
```
With every movement the variable is altered:  
```c
traveledDistance += displacement;  
```

---
**_In the Haptics Thread_**  
  
---
A variable is declared:  
```c
float minDist = 0;  
```
  
---  
**_In the Haptics loop we get (while loop ins Haptic Thread)_**  
  
---

```c
If(traveledDistance.length() > minDist){  
	//Do a collision detection => an approximation of the minimum distance by the collision detection algorithm = schattingD  
	minDist = schattingD;  
	traveledDistance->zero();  
}  
```

  
  


