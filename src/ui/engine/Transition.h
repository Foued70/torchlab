#ifndef TRANSITION_H
#define TRANSITION_H

#include "TimeManager.h"

class Transition {
public:
  virtual void tick()=0;
  virtual bool isComplete()=0;
  
};

#endif