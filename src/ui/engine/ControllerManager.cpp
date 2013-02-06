#include "ControllerManager.h"

#include "CameraController.h"
#include "Camera.h"

ControllerManager::ControllerManager() {
}

ControllerManager::~ControllerManager() {
  __deleteAllControllers();
}

void ControllerManager::__deleteAllControllers() {
  while(!__allControllers.empty()) delete __allControllers.front(), __allControllers.pop_front();
  __activeControllers.clear();
}

void ControllerManager::update() {
  auto it = __activeControllers.begin();
  while(it != __activeControllers.end() && !__activeControllers.empty()) {
    DynamicsBase* temp = (*it);
    temp -> update();
    
    if (!isActive(temp)) {
      it--;
    }
    else {
      it++;
    }
  }
}

bool ControllerManager::needsUpdate() {
  return !__activeControllers.empty();
}

bool ControllerManager::isActive(DynamicsBase* _controller) {
  for(auto it = __activeControllers.begin(); it != __activeControllers.end(); it++) {
    if ((*it) == _controller ) {
      return true;
    }
  }
  return false;
}

void ControllerManager::activate(DynamicsBase* _controller) {
  if ( !isActive(_controller) ) {
     __activeControllers.push_back(_controller);
     log(PARAM, "Controller Activated");
  }
}

void ControllerManager::deactivate(DynamicsBase* _controller) {
  if ( isActive(_controller) ) {
    __activeControllers.remove(_controller);
    log(PARAM, "Controller Deactivated");
  }
}

CameraController* ControllerManager::createCameraController() {
  CameraController* temp = new CameraController();
  __allControllers.push_back(temp);
  return temp;
}

CameraController* ControllerManager::createCameraController(Camera* _camera) {
  CameraController* temp = new CameraController();
  __allControllers.push_back(temp);
  temp -> possess(_camera);
  return temp;
}