#ifndef TIMEMANAGER_H
#define TIMEMANAGER_H

#include "Singleton.h"

class TimeManager : public Singleton< TimeManager > {
public:
  TimeManager();
  
  void start();
  void tick();
  double getDeltaTime() const;
  double getTimeSeconds() const;
  static unsigned long long getTime();
  static void logTime();
  void logDeltaTime() const;

private:
  unsigned long long __startTime;
  unsigned long long __previousTime;
  unsigned long long __currentTime;
  double __deltaTime;
  const double __maxTimeStep = (1.0/30.0);
  
  
};

#endif