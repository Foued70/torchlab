#include "CameraController.h"
#include "Camera.h"
#include "ControllerManager.h"
#include "SteeringComponent.h"
#include "Engine.h"

CameraController::CameraController() : 
  Controller<Camera>(),
  __steeringEye(NULL),
  __steeringCenter(NULL)
{
  __steeringEye = new SteeringComponent();
  __steeringCenter = new SteeringComponent();
  
  log(CONSTRUCTOR, "CameraController constructed.");
}

CameraController::~CameraController() {
  delete __steeringEye;
  delete __steeringCenter;
}

void CameraController::update() {
  if (needsUpdate()) {   
    __steeringEye->update();
    __steeringCenter->update();

    __pawn->setEyePosition(__steeringEye->getPosition());
    __pawn->setCenterPosition(__steeringCenter->getPosition());
  }
  else {
    updateState();
  }
}

void 
CameraController::updateState() {
  switch (__state) {
    case IDLE:
      if(needsUpdate()) {
        __state = DYNAMIC;
        ControllerManager::GetSingleton().activate(this);
      }
      break;
    case DYNAMIC:
      if(!needsUpdate()) {
        __state = IDLE;
        ControllerManager::GetSingleton().deactivate(this);
      }
      break;
    default:
      __state = IDLE;
      break;
  }
}

void 
CameraController::flyTo(const Vector3& _destination) {
  const float goalPadding = 5.0f;
  const float viewingHeight = 2.25f;
  Vector3 eyeOffsetDirection = __pawn->getEye() - _destination;
  eyeOffsetDirection.normalize();
  Vector3 eyePosition = _destination + (eyeOffsetDirection*goalPadding);   
  Vector3 ground;
  if ( Engine::GetSingleton().raycast(eyePosition, Vector3({eyePosition.x, eyePosition.y, eyePosition.z - 1.0f}), ground) ) {
    eyePosition.z = ground.z + viewingHeight;
  }
  __steeringEye->setPosition(__pawn->getEye());
  __steeringCenter->setPosition(__pawn->getCenter());
  __steeringEye->moveTo(eyePosition, SMOOTH_DAMP); //no max speed
  __steeringCenter->moveTo(_destination, SMOOTH_DAMP); //no max speed
  updateState();
}

bool 
CameraController::needsUpdate() {
  return ( __steeringEye->needsUpdate() || __steeringCenter->needsUpdate() );
}