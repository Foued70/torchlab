#ifndef PHYSICS_COMPONENT_H
#define PHYSICS_COMPONENT_H

#include "Vectors.h"

class PhysicsComponent {
public:
  PhysicsComponent();
  ~PhysicsComponent();
  
  virtual void update();
  void applyForce(const Vector3& _force);
  Vector3 getPosition();
  void setPosition(const Vector3& _position);
  
protected:
  void __tickPhysics();
  
protected:
  Vector3 __position;
  Vector3 __velocity;
  Vector3 __acceleration;
  Vector3 __force;
  
  Vector3 __rotation;
  Vector3 __rotationVelocity;
  Vector3 __rotationAcceleration;
  Vector3 __rotationForce;
  
  Vector3 __maxVelocity;
};

#endif