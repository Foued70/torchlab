#include "SteeringComponent.h"
#include "SmoothDampInterpolate.h"
#include "BezierInterpolate.h"

SteeringComponent::SteeringComponent() : 
  PhysicsComponent()
{
}

SteeringComponent::~SteeringComponent() {
  __endAllTransitions();
}

void
SteeringComponent::update() {
  if (!__transitions.empty()) {
    auto it = __transitions.begin();
    while(it != __transitions.end()) {
      (*it)->tick();
      if ((*it)->isComplete()) {
        Transition* temp = (*it);
        it = __transitions.erase(it);
        delete temp;
      }
      else {
        it++;
      }
    }
  }
}

bool 
SteeringComponent::needsUpdate() {
  return !__transitions.empty();
}

void 
SteeringComponent::moveTo(const Vector3& _goal, const MOVE_BEHAVIOR& _behavior, const float& _speed) {
  switch(_behavior) {
    case LINEAR :{
      __endAllTransitions();
      __transitions.push_back(new BezierInterpolate(&__position.x, _goal.x, _speed, BEZIER_LINEAR, BEZIER_LINEAR));
      __transitions.push_back(new BezierInterpolate(&__position.y, _goal.y, _speed, BEZIER_LINEAR, BEZIER_LINEAR));
      __transitions.push_back(new BezierInterpolate(&__position.z, _goal.z, _speed, BEZIER_LINEAR, BEZIER_LINEAR));
      break;
    }
    case SOFT :{
      __endAllTransitions();
      __transitions.push_back(new BezierInterpolate(&__position.x, _goal.x, _speed, BEZIER_SLOW, BEZIER_SLOW));
      __transitions.push_back(new BezierInterpolate(&__position.y, _goal.y, _speed, BEZIER_SLOW, BEZIER_SLOW));
      __transitions.push_back(new BezierInterpolate(&__position.z, _goal.z, _speed, BEZIER_SLOW, BEZIER_SLOW));      
      break;
    }
    case FAST_BOUNCE :{
      __endAllTransitions();
      Vector2 startBezierHandleX = Vector2({0.0f, 1.09f});
      Vector2 endBezierHandleX = Vector2({0.0f, 1.0f});
      __transitions.push_back(new BezierInterpolate(&__position.x, _goal.x, _speed, startBezierHandleX, endBezierHandleX));
      
      Vector2 startBezierHandleY = Vector2({0.0f, 1.09f});
      Vector2 endBezierHandleY = Vector2({0.0f, 1.0f});
      __transitions.push_back(new BezierInterpolate(&__position.y, _goal.y, _speed, startBezierHandleY, endBezierHandleY));
      
      Vector2 startBezierHandleZ = Vector2({0.0f, 1.09f});
      Vector2 endBezierHandleZ = Vector2({0.0f, 1.0f});
      __transitions.push_back(new BezierInterpolate(&__position.z, _goal.z, _speed, startBezierHandleZ, endBezierHandleZ));
      break;
    }
    default:
      break;
  }
}

void 
SteeringComponent::__endAllTransitions() {
  while(!__transitions.empty()) delete __transitions.front(), __transitions.pop_front();
}