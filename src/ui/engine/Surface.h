#ifndef SURFACE_H
#define SURFACE_H



#include "Vertex.h"
#include "Vectors.h"

#include <vector>

using namespace std;

class Triangle;
class Camera;
struct BoundingRect;

class Surface {
public:
  Surface();
  ~Surface();
  
  void addTriangle(Triangle* _triangle);
  bool removeTriangle(Triangle* _triangle);
  void clearAllTriangles();
  
  Vector3 getNormal() const;
  Vector3 getCenter() const;  
  BoundingRect getBoundingRectScreenSpace(Camera* _camera) const;
  
  static Surface* GenSurfaceFromTriangle(Triangle* _triangle);
  
private:
  void __calculateNormal();
  void __calculateCenter();
    
private:
  vector<Triangle*> __triangles;
  Vector3 __normal;
  Vector3 __center;
  
  
};

#endif