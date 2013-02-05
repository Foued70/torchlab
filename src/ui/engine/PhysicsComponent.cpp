#include "PhysicsComponent.h"

#include "TimeManager.h"

PhysicsComponent::PhysicsComponent() :
  __position(Vector3({0.0f, 0.0f, 0.0f})),
  __velocity(Vector3({0.0f, 0.0f, 0.0f})),
  __acceleration(Vector3({0.0f, 0.0f, 0.0f})),
  __force(Vector3({0.0f, 0.0f, 0.0f})),
  __rotation(Vector3({0.0f, 0.0f, 0.0f})),
  __rotationVelocity(Vector3({0.0f, 0.0f, 0.0f})),
  __rotationAcceleration(Vector3({0.0f, 0.0f, 0.0f})),
  __rotationForce(Vector3({0.0f, 0.0f, 0.0f})),
  __maxVelocity(Vector3({50.0f, 50.0f, 50.0f}))
{
  
}

PhysicsComponent::~PhysicsComponent() {
  
}

void 
PhysicsComponent::update() {
  __tickPhysics();
}

void
PhysicsComponent::__tickPhysics() {
  double deltaTime = TimeManager::GetSingleton().getDeltaTime();
  
  __acceleration = __force;
  __force = Vector3({0, 0, 0});
  __velocity += __acceleration * deltaTime;
  __velocity = min(__velocity, __maxVelocity);
  
  Vector3 drag = -__velocity;
  drag.normalize();
  drag *= 500.0f;
  
  Vector3 instDrag = drag*deltaTime;
  __velocity = (instDrag.magnitude() < __velocity.magnitude()) ? (__velocity + instDrag) : Vector3({0,0,0});
    
  __position += __velocity * deltaTime;
}

void 
PhysicsComponent::applyForce(const Vector3& _force) {
  __force += _force;
}

Vector3 
PhysicsComponent::getPosition() {
  return __position;
}

void 
PhysicsComponent::setPosition(const Vector3& _position) {
  __position = _position;
}