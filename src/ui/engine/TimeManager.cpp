#include <sys/time.h>

#include "TimeManager.h"
#include "utils.h"

TimeManager::TimeManager()
{
  log(CONSTRUCTOR, "TimeManager constructed.");
}

void 
TimeManager::start() {
  __previousTime = getTime();
  __currentTime = getTime();
  __deltaTime = 0.0;
  __deltaTime = __maxTimeStep;
}

void 
TimeManager::tick() {
  __previousTime = __currentTime;
  __currentTime = getTime();
  __deltaTime = double(__currentTime - __previousTime) / 1000.0;
  
  //Limiting timestep to stop simulations from spiraling out of control during spikes
  __deltaTime = (__maxTimeStep > __deltaTime) ? __deltaTime : __maxTimeStep;
}

double 
TimeManager::getDeltaTime() const {
  return __deltaTime;
}

double TimeManager::getTimeSeconds() const {
  return getTime() / 1000.0;
}

unsigned long long 
TimeManager::getTime() {
  //Get the current time in milliseconds
  timeval currentTime;
  gettimeofday(&currentTime, NULL);
  return (currentTime.tv_sec * 1000) + (currentTime.tv_usec / 1000) + 0.5;
}

void 
TimeManager::logTime() {
  log(PARAM, "Current system time: %i seconds", int(getTime() / 1000));
}

void 
TimeManager::logDeltaTime() const {
  log(PARAM, "Time delta for current tick: %f seconds", (float)__deltaTime);
}

