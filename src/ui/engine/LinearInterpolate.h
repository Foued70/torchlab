#ifndef LINEAR_INTERPOLATE_H
#define LINEAR_INTERPOLATE_H

#include "Transition.h"

#include "utils.h"

template < typename T >
class LinearInterpolate : public Transition {
public:
  LinearInterpolate(T* _value, const T& _endValue, const float& _transitionLength);
  ~LinearInterpolate();
  
  virtual void tick();
  virtual bool isComplete();
  
protected:
  T* __value;
  T __startValue;
  T __endValue;
  T __offset;
  float __transitionLength;
  float __progress;

};

template < typename T >
LinearInterpolate<T>::LinearInterpolate(T* _value, const T& _endValue, const float& _transitionLength) : Transition(),
  __value(_value),
  __startValue(*_value),
  __endValue(_endValue),
  __transitionLength(_transitionLength),
  __progress(0.0f)
{
  __offset = __endValue - __startValue;
}

template < typename T >
LinearInterpolate<T>::~LinearInterpolate() {
  
}
  
template < typename T > 
void 
LinearInterpolate<T>::tick() {
  if ( !isComplete() ) {
    float deltaTime = TimeManager::GetSingleton().getDeltaTime();
    float deltaProgress = deltaTime / __transitionLength;
    __progress += deltaProgress;
    __progress = (__progress <= 1.0f) ? __progress : 1.0f;
    T currentOffset = __offset * ((__progress*__transitionLength) / __transitionLength);
    (*__value) = __startValue + currentOffset;
  }
}

template < typename T >
bool 
LinearInterpolate<T>::isComplete() {
  return (__progress >= 1.0f);
}

#endif