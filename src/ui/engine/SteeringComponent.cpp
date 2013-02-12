#include "SteeringComponent.h"
#include "SmoothDampInterpolate.h"

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
      if (_speed <= 0.0f) {
        __transitions.push_back(new LinearInterpolate<Vector3>(&__position, _goal, 1.0f));
      }
      else {
        Vector3 offset = _goal - __position;
        float transitionLength = offset.magnitude() / _speed;
        __transitions.push_back(new LinearInterpolate<Vector3>(&__position, _goal, transitionLength));
      }
      
      break;
    }
    case SMOOTH_DAMP :{
      __endAllTransitions();
      __transitions.push_back(new SmoothDampInterpolate<Vector3>(&__position, _goal, 0.29f, _speed));
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