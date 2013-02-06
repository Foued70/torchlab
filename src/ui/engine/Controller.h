#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "DynamicsBase.h"

template <class Pawn>
class Controller : public DynamicsBase {
public:
  Controller() :
    DynamicsBase(),
    __pawn(NULL) {}
    
  ~Controller() { unpossess(); }
  
  virtual void update() {};
  
  void possess(Pawn* _pawn) {
    if(__pawn != NULL) {
      //Reserving this branch incase detaching methods on __pawn need to happen.
      __pawn = _pawn;
    }
    else {
      __pawn = _pawn;
    }
  }
  
  void unpossess() {
    if (__pawn != NULL) {
      __pawn = NULL;
    }
  }
   
protected:
virtual void __updateState() {};
  
protected:
  Pawn* __pawn;
  
};

#endif