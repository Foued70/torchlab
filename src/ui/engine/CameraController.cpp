#include "CameraController.h"
#include "Camera.h"
#include "ControllerManager.h"
#include "SteeringComponent.h"
#include "Engine.h"
#include "Surface.h"

CameraController::CameraController() : 
  Controller<Camera>(),
  __zoom(1.0f),
  __flightTime(0.35f),
  __flightMode(FREE_FLIGHT),
  __steeringEye(NULL),
  __steeringCenter(NULL),
  __selectedSurface(NULL)
{
  __steeringEye = new SteeringComponent();
  __steeringCenter = new SteeringComponent();
  
  log(CONSTRUCTOR, "CameraController constructed.");
}

CameraController::~CameraController() {
  delete __steeringEye;
  delete __steeringCenter;
  
  if (__selectedSurface) {
    delete __selectedSurface;
  }
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

float 
CameraController::getZoom() const {
  return __zoom;
}

void 
CameraController::setZoom(float _zoom) {
  __zoom = _zoom;
  Vector3 newEyePosition = __pawn->getCenter() - (__pawn->lookDirection() * __zoom);
  __pawn->setEyePosition(newEyePosition);
}

void
CameraController::setPositionAndRotation(const Vector3& _position, const Vector3& _rotation) {
  __pawn->setEyePosition(_position);
  __pawn->setCenterPosition(_position + Vector3({0.0f, -1.0f, 0.0f}));
  
  log(PARAM, "in rotation: %f %f %f", _rotation.x, _rotation.y, _rotation.z);
  log(PARAM, "camera center position was set to: %f %f %f", __pawn->getCenter().x, __pawn->getCenter().y, __pawn->getCenter().z);
  
  //TO DO: Currently, x-axis rotation(roll) is not supported. This needs to be availible for pose alignment
  __pawn->rotateCenterAroundCamera(_rotation.y, _rotation.z);
  
  log(PARAM, "camera center position now set to: %f %f %f", __pawn->getCenter().x, __pawn->getCenter().y, __pawn->getCenter().z);
}

void 
CameraController::rotate(float _deltaX, float _deltaY) {
  const float rotationSpeed = 0.001;
  const float surfaceRotationSpeed = rotationSpeed * 1.0f;
  switch(__flightMode) {
    case FREE_FLIGHT:
      __pawn->rotateCenterAroundCamera(_deltaX*rotationSpeed, _deltaY*rotationSpeed);
      break;
    case SURFACE_FLIGHT:
      __pawn->rotateAroundCenter(_deltaX*surfaceRotationSpeed, _deltaY*-surfaceRotationSpeed);
      break;
    default:
      __flightMode = FREE_FLIGHT;
      break;
  }
}

void 
CameraController::flyTo(const Vector3& _destination) {
  __flightMode = FREE_FLIGHT;
  const float goalPadding = 8.0f;
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
  __steeringEye->moveTo(eyePosition, SOFT, __flightTime);
  __steeringCenter->moveTo(_destination, SOFT, __flightTime);
  updateState();
}

void 
CameraController::selectSurface(Surface* _surface, const Vector3& _startAim) {
  __flightMode = SURFACE_FLIGHT;
  if (__selectedSurface) {
    delete __selectedSurface;
  }
  __selectedSurface = _surface;
  
  float viewDistance = 5.0f;
  
  Vector3 viewOffset = __selectedSurface->getNormal()*viewDistance;
  if (dot(__pawn->getEye() - _startAim, viewOffset) < 0.0f) {
    viewOffset = -viewOffset;
  }
  
  Vector3 eyeGoal = _startAim + viewOffset;
  Vector3 centerGoal = _startAim;
  
  __steeringEye->setPosition(__pawn->getEye());
  __steeringCenter->setPosition(__pawn->getCenter());
  __steeringEye->moveTo(eyeGoal, FAST_BOUNCE, __flightTime);
  __steeringCenter->moveTo(centerGoal, FAST_BOUNCE, __flightTime);
  updateState();
}

bool 
CameraController::needsUpdate() {
  return ( __steeringEye->needsUpdate() || __steeringCenter->needsUpdate() );
}