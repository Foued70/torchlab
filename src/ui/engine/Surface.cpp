#include "Surface.h"
#include "Mesh.h"
#include "Camera.h"
#include "utils.h"

Surface::Surface() :
  __normal(Vector3({0.0f, 0.0f, 0.0f})),
  __center(Vector3({0.0f, 0.0f, 0.0f}))
{
}

Surface::~Surface() {
  __triangles.clear();
}
  
void 
Surface::addTriangle(Triangle* _triangle) {
  __triangles.push_back(_triangle);
  __calculateNormal();
  __calculateCenter();
}

bool 
Surface::removeTriangle(Triangle* _triangle) {
  for(int t = 0; t < __triangles.size(); t++) {
    if (__triangles[t] == _triangle) {
      __triangles.erase(__triangles.begin()+t);
      __calculateNormal();
      __calculateCenter();
      return true;
    }
  }
  return false;
}

void 
Surface::clearAllTriangles() {
  __triangles.clear();
}
  
Vector3 
Surface::getNormal() const {
  return __normal;
}

Vector3
Surface::getCenter() const {
  return __center;
}

BoundingRect 
Surface::getBoundingRectScreenSpace(Camera* _camera) const {
  BoundingRect boundingRect;
  for(int t = 0; t < __triangles.size(); t++) {
    for(int v = 0; v < 3; v++) {
      Position worldSpacePosition = __triangles[t]->vertices[v]->vertexPosition;
      Vector3 worldSpaceVertex = Vector3({worldSpacePosition.x, worldSpacePosition.y, worldSpacePosition.z});
      Vector3 camSpaceVertex = _camera->worldToCamera(worldSpaceVertex);
      if (t == v == 0) {
        boundingRect.min.x = camSpaceVertex.x;
        boundingRect.min.y = camSpaceVertex.y;
        boundingRect.max.x = camSpaceVertex.x;
        boundingRect.max.y = camSpaceVertex.y;
        continue;
      }
      if ( camSpaceVertex.x < boundingRect.min.x ) {
        boundingRect.min.x = camSpaceVertex.x;
      }
      else if ( camSpaceVertex.x > boundingRect.max.x ) {
        boundingRect.max.x = camSpaceVertex.x;
      }
      if ( camSpaceVertex.y < boundingRect.min.y ) {
        boundingRect.min.y = camSpaceVertex.y;
      }
      else if ( camSpaceVertex.y > boundingRect.max.y ) {
        boundingRect.max.y = camSpaceVertex.y;
      }
    }
  }
  return boundingRect;
}



void 
Surface::__calculateNormal() {
  __normal = Vector3({0.0f, 0.0f, 0.0f});
  for(int t = 0; t < __triangles.size(); t++) {
    Vector3 triangleNormal = Vector3({0.0f, 0.0f, 0.0f});
    for(int v = 0; v < 3; v++) {
      Vertex* current = __triangles[t]->vertices[v];
      Vertex* next = __triangles[t]->vertices[(v+1)%3];
    
      triangleNormal.x += ((current->vertexPosition.y - next->vertexPosition.y) * (current->vertexPosition.z + next->vertexPosition.z));
      triangleNormal.y += ((current->vertexPosition.z - next->vertexPosition.z) * (current->vertexPosition.x + next->vertexPosition.x));
      triangleNormal.z += ((current->vertexPosition.x - next->vertexPosition.x) * (current->vertexPosition.y + next->vertexPosition.y));
    }
    __normal += triangleNormal;
  }
  __normal.normalize();
}

void 
Surface::__calculateCenter() {
  __center = Vector3({0.0f, 0.0f, 0.0f});
  unsigned int vertexCount = 0;
  for(int t = 0; t < __triangles.size(); t++) {
    for(int v = 0; v < 3; v++) {
      __center.x += __triangles[t]->vertices[v]->vertexPosition.x;
      __center.y += __triangles[t]->vertices[v]->vertexPosition.y;
      __center.z += __triangles[t]->vertices[v]->vertexPosition.z;
      
      vertexCount++;      
    }
  }
  __center /= vertexCount;
}

Surface* 
Surface::GenSurfaceFromTriangle(Triangle* _triangle) {
  if (!_triangle) {
    log(WARN, "Cannot generate a surface from a NULL triangle pointer.");
    return NULL;
  }
  Surface* surface = new Surface();
  surface->addTriangle(_triangle);
  return surface;
}