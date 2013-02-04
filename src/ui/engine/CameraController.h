#ifndef CAMERACONTROLLER_H
#define CAMERACONTROLLER_H

#include "Controller.h"
#include "Vectors.h"
#include <list>

using namespace std;

class Camera;
class SteeringComponent;

/*
struct CameraGoal {
  Vector3 __eye;
  Vector3 __center;
  
  CameraGoal() : 
    __eye(Vector3({0.0f, 0.0f, 0.0f})),
    __center(Vector3({0.0f, 0.0f, 0.0f}))
  {}
      
  CameraGoal(const Vector3& _eye, const Vector3& _center) :
    __eye(_eye),
    __center(_center)
  {}
};
*/

class CameraController : public Controller<Camera> {
public:
  CameraController();
  ~CameraController();
  
  virtual void update();
  
  void flyTo(const Vector3& _destination);
  
protected:
  virtual void updateState();
  virtual bool needsUpdate();
  
private:
  bool __atGoal();
  
private:
  SteeringComponent* __steeringEye;
  SteeringComponent* __steeringCenter;
};

#endif