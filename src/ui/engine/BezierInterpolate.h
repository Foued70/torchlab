#ifndef BEZIER_INTERPOLATE_H
#define BEZIER_INTERPOLATE_H

#include "Vectors.h"   

enum BEZIER_HANDLE_TYPE {
  BEZIER_LINEAR,
  BEZIER_SLOW,
  BEZIER_FAST
};

class BezierInterpolate : public LinearInterpolate<float> {
public:
  BezierInterpolate(float* _value, float _endValue, float _transitionLength, const Vector2& _startHandle, const Vector2& _endHandle);
  BezierInterpolate(float* _value, float _endValue, float _transitionLength, BEZIER_HANDLE_TYPE _inBehavior, BEZIER_HANDLE_TYPE _outBehavior);
  ~BezierInterpolate();
  
  virtual void tick();
  virtual bool isComplete();
  
private:
  float __calculateValue(float _progress);
  
protected:
  Vector2 __startHandle;
  Vector2 __endHandle;
};

BezierInterpolate::BezierInterpolate(float* _value, float _endValue, float _transitionLength, const Vector2& _startHandle, const Vector2& _endHandle) :
  LinearInterpolate<float>(_value, _endValue, _transitionLength),
  __startHandle(_startHandle),
  __endHandle(_endHandle)
{
}

BezierInterpolate::BezierInterpolate(float* _value, float _endValue, float _transitionLength, BEZIER_HANDLE_TYPE _startBehavior, BEZIER_HANDLE_TYPE _endBehavior) :
  LinearInterpolate<float>(_value, _endValue, _transitionLength)
{
  switch(_startBehavior) {
    case BEZIER_LINEAR:
      __startHandle = Vector2({0.5f, 0.5f});
      break;
    case BEZIER_SLOW:
      __startHandle = Vector2({0.5f, 0.0f});
      break;
    case BEZIER_FAST:
      __startHandle = Vector2({0.0f, 0.5f});
      break;
  }
  switch(_endBehavior) {
    case BEZIER_LINEAR:
      __endHandle = Vector2({0.5f, 0.5f});
      break;
    case BEZIER_SLOW:
      __endHandle = Vector2({0.5f, 1.0f});
      break;
    case BEZIER_FAST:
      __endHandle = Vector2({1.0f, 0.5f});
      break;
  }
}

BezierInterpolate::~BezierInterpolate() {
}

void 
BezierInterpolate::tick() {
  if ( !isComplete() ) {
    float deltaTime = TimeManager::GetSingleton().getDeltaTime();
    float deltaProgress = deltaTime / LinearInterpolate<float>::__transitionLength;
    LinearInterpolate<float>::__progress += deltaProgress;
    LinearInterpolate<float>::__progress = (LinearInterpolate<float>::__progress <= 1.0f) ? LinearInterpolate<float>::__progress : 1.0f;
    (*LinearInterpolate<float>::__value) = __calculateValue(LinearInterpolate<float>::__progress) * (LinearInterpolate<float>::__endValue-LinearInterpolate<float>::__startValue) + LinearInterpolate<float>::__startValue;
  }
}

bool 
BezierInterpolate::isComplete() {
  return (LinearInterpolate<float>::__progress >= 1.0f);
}

float
BezierInterpolate::__calculateValue(float _progress) {
  Vector2 pointOnCurve;
  pointOnCurve  = __startHandle * (3.0f*(1.0f-_progress)*(1.0f-_progress)*_progress);
  pointOnCurve += __endHandle * (3.0f*(1.0f-_progress)*_progress*_progress);
  pointOnCurve += _progress*_progress*_progress;
  return pointOnCurve.y;
}

#endif