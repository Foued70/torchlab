#ifndef CONTROLLER_MANAGER_H
#define CONTROLLER_MANAGER_H

#include "Singleton.h"
#include <list>

using namespace std;

class DynamicsBase;
class Camera;
class CameraController;

class ControllerManager : public Singleton< ControllerManager > {
public:
  ControllerManager();
  ~ControllerManager();
  
  void update();
  bool needsUpdate();
  bool isActive(DynamicsBase* _controller);
  void activate(DynamicsBase* _controller);
  void deactivate(DynamicsBase* _controller);
  
  CameraController* createCameraController();
  CameraController* createCameraController(Camera* _camera);
  
private:
  void __deleteAllControllers();
  
private:
  list<DynamicsBase*> __allControllers;
  list<DynamicsBase*> __activeControllers;
  
};

#endif