#ifndef SMOOTH_DAMP_INTERPOLATION_H
#define SMOOTH_DAMP_INTERPOLATION_H

#include "LinearInterpolate.h"

template< typename T >
class SmoothDampInterpolate : public LinearInterpolate<T> {
public:
  SmoothDampInterpolate(T* _value, const T& _endValue, const float& _smoothLength = 1.0f, const float& _maxSpeed = 0.0f);
  ~SmoothDampInterpolate();
  
  virtual void tick();
  virtual bool isComplete();
  
protected:
  virtual T clamp(const T& _a, const T& _min, const T& _max);
  
protected:
  T __velocity;
  float __smoothLength;
  float __maxSpeed;
};

template < typename T >
SmoothDampInterpolate<T>::SmoothDampInterpolate(T* _value, const T& _endValue, const float& _smoothLength, const float& _maxSpeed) : LinearInterpolate<T>(_value, _endValue, -1.0f),
  __velocity(T()),
  __smoothLength(_smoothLength),
  __maxSpeed(_maxSpeed)
{
}

template < typename T >
SmoothDampInterpolate<T>::~SmoothDampInterpolate() {
}
  
template < typename T > 
void 
SmoothDampInterpolate<T>::tick() {
  if ( !isComplete() ) {
    float deltaTime = TimeManager::GetSingleton().getDeltaTime();
    __smoothLength = (0.0001f < __smoothLength) ? __smoothLength : 0.0001f;
    __smoothLength = (__smoothLength < 1.0f) ? __smoothLength : 1.0f;    
    float smoothing = (2.0f / __smoothLength);
    float deltaSmoothing = smoothing * deltaTime;
    float spring = 1.0f / (1.0f + deltaSmoothing + (0.48f*deltaSmoothing*deltaSmoothing) + (0.235f)*deltaSmoothing*deltaSmoothing*deltaSmoothing);
    T negativeOffset = (*LinearInterpolate<T>::__value) - LinearInterpolate<T>::__endValue;
    if (__maxSpeed > 0.0f) {
      float maxSpeed = __maxSpeed * __smoothLength;
      negativeOffset = clamp(negativeOffset, T(-maxSpeed), T(maxSpeed));
    }
    T adjustedEnd = (*LinearInterpolate<T>::__value) - negativeOffset;

    T num7 = (__velocity + (negativeOffset*smoothing)) * deltaTime;
    __velocity = (__velocity - (num7*smoothing)) * spring;

    (*LinearInterpolate<T>::__value) = adjustedEnd + ((negativeOffset + num7) * spring);
  }
}

template <>
float 
SmoothDampInterpolate<float>::clamp(const float& _a, const float& _min, const float& _max) {
  float result = _a;
  result = (result < _max) ? result : _max;
  result = (result > _min) ? result : _min;
  return result;
}

template <>
Vector3 
SmoothDampInterpolate<Vector3>::clamp(const Vector3& _a, const Vector3& _min, const Vector3& _max) {
  Vector3 result = _a;
  result.x = (result.x < _max.x) ? result.x : _max.x;
  result.y = (result.y < _max.y) ? result.y : _max.y;
  result.z = (result.z < _max.z) ? result.z : _max.z;
  
  result.x = (result.x > _min.x) ? result.x : _min.x;
  result.y = (result.y > _min.y) ? result.y : _min.y;
  result.z = (result.z > _min.z) ? result.z : _min.z;
  return result;
}

template < typename T >
bool
SmoothDampInterpolate<T>::isComplete() {
  return true;
}

template <>
bool
SmoothDampInterpolate<float>::isComplete() {
  float currentOffset = LinearInterpolate<float>::__endValue - (*LinearInterpolate<float>::__value);
  float goalOffset = LinearInterpolate<float>::__endValue - (LinearInterpolate<float>::__startValue);
  
  return (currentOffset / goalOffset) > 0.99f;
}

template <>
bool
SmoothDampInterpolate<Vector3>::isComplete() {
  const float goalThreshold = 0.5f;
  return distanceSquared((*LinearInterpolate<Vector3>::__value), LinearInterpolate<Vector3>::__endValue) < (goalThreshold*goalThreshold);
}


#endif