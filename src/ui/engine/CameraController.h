#ifndef CAMERACONTROLLER_H
#define CAMERACONTROLLER_H

#include "Controller.h"
#include "Vectors.h"
#include <list>
using namespace std;

class Camera;
class SteeringComponent;
class Surface;

enum FLIGHT_MODE {
  FREE_FLIGHT,
  SURFACE_FLIGHT
};

class CameraController : public Controller<Camera> {
public:
  CameraController();
  ~CameraController();
  
  virtual void update();
  
  float getZoom() const;
  void setZoom(float _zoom);
  void rotate(float _x, float _y);
  void flyTo(const Vector3& _destination);
  void selectSurface(Surface* _surface, const Vector3& _startAim);
  
protected:
  virtual void updateState();
  virtual bool needsUpdate();
  
private:
  bool __atGoal();
  
private:
  float __zoom;
  float __flightTime;
  FLIGHT_MODE __flightMode;
  SteeringComponent* __steeringEye;
  SteeringComponent* __steeringCenter;
  Surface* __selectedSurface;
};

#endif