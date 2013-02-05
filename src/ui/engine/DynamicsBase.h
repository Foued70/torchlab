#ifndef DYNAMICS_BASE_H
#define DYNAMICS_BASE_H

#ifndef NULL
#define NULL 0
#endif

#include "utils.h"

enum DYNAMICS_STATE {
  IDLE,
  DYNAMIC  
};

class DynamicsBase {
public:
  DynamicsBase() : __state(IDLE) {}
  ~DynamicsBase() {}  
  virtual void update()=0;
  
protected:
  virtual void __updateState()=0;
  
protected:
  DYNAMICS_STATE __state;
};

#endif