#ifndef STEERING_COMPONENT_H
#define STEERING_COMPONENT_H

#include "PhysicsComponent.h"
#include <list>

using namespace std;

class Transition;

enum MOVE_BEHAVIOR {
  LINEAR,
  SMOOTH_DAMP
};

class SteeringComponent : public PhysicsComponent {
public:
  SteeringComponent();
  ~SteeringComponent();
  
  virtual void update();
  bool needsUpdate();
  void moveTo(const Vector3& _goal, const MOVE_BEHAVIOR& _behavior = LINEAR, const float& _speed = 0.0f);

protected:
  void __endAllTransitions();
  
protected:
  list<Transition*> __transitions;
};

#endif