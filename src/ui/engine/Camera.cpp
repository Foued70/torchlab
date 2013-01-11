#include <iostream>
#include <cmath>

#include "Camera.h"
#include "MatricesManager.h"
#include "utils.h"

static const double PI = 3.1415265359;
static const double PIdiv180 = PI/180.0;
static const double PIdiv2 = PI/2;

using namespace std;

Camera::Camera() :
		__fovy(45.0),
		__zNear(0.0001),
		__zFar(1000.0),
		__eye({0, 0, 0}),
		__center({0, 0, 0}),
		__up({0, 0, 1}),
		__matrices(MatricesManager::GetSingleton()) {
	log(CONSTRUCTOR, "Camera constructed.");
}

Camera::Camera(GLfloat _x, GLfloat _y, GLfloat _z) :
		__fovy(45.0),
		__zNear(0.0001),
		__zFar(1000.0),
		__eye({_x, _y, _z}),
		__center({0, 0, 0}),
		__up({0, 0, 1}),
		__matrices(MatricesManager::GetSingleton()) {

    log(CONSTRUCTOR, "Camera (%f, %f, %f) constructed.", __eye.x, __eye.y, __eye.z);
}

Camera::~Camera() {
	log(DESTRUCTOR, "Camera destructed.");
}

void
Camera::setProjection() {
  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);
  log(PARAM, "setProjection %d, %d", viewport[2], viewport[3]);
  
	__windowWidth = viewport[2];
	__windowHeight = viewport[3];
	
	GLfloat aspect = (GLfloat) __windowWidth / __windowHeight;
	
	__matrices.sPerspective(__fovy, aspect, __zNear, __zFar);
}

void
Camera::setView() {
  __matrices.sLookAt(__eye, __center, __up);
  
  // we have modelViewMatrix set, we can count the normal matrix now.
  __matrices.produceNormalMatrix();
}


void
Camera::calcUp() {
  Vector3 dir = lookDirection();
  if (dir == Z_AXIS || dir == -Z_AXIS) {
    // use our last up value
    __up[2] = 0;
    __up.normalize();
  }
  else {
    Vector3 perpUpDir = cross(dir, Z_AXIS);
    perpUpDir.normalize();
    __up = cross(perpUpDir, dir);
  }
}



void
Camera::moveEye(GLfloat _x, GLfloat _y, GLfloat _z) {
  Vector3 forwardDir = lookDirection();
  Vector3 rightDir = rightDirection();
  __eye += forwardDir * _z;
  __eye += rightDir * _x;
  __eye += __up * _y;
  
  // move the center, but not along the line of sight, for now
  // the z direction changes the range
  // __center += forwardDir * _z;
  __center += rightDir * _x;
  __center += __up * _y;
}


void
Camera::rotateAroundCenter(GLfloat _x, GLfloat _y) {
  const float RADS = 0.02;
  float xAngle = _x * RADS;
  float yAngle = _y * RADS;
  
  // move eye as if center is at origin
  __eye -= __center;
  
  // rotate around a horizontal axis (y rotation)
  Vector3 unitEye = __eye;
  unitEye.normalize();
  float zAxisAngle = acos(dot(unitEye, Z_AXIS));
  
  if (zAxisAngle - yAngle < 0) {
    __eye = Z_AXIS * __eye.magnitude();
    // rotate up for x instead of eye
    __up = rotateAxisAngle(__up, Z_AXIS, xAngle);
  }
  else if (zAxisAngle - yAngle > PI) {
    __eye = Z_AXIS * -__eye.magnitude();
    // rotate up for x instead of eye
    __up = rotateAxisAngle(__up, Z_AXIS, xAngle);
  }
  else {
    Vector3 axis = cross(unitEye, __up).normalize();
    __eye = rotateAxisAngle(__eye, axis, yAngle);
    // rotate around Z-up (x rotation)
    __eye = rotateAxisAngle(__eye, Z_AXIS, xAngle);
  }

  // move eye back into position
  __eye += __center;
  calcUp();
}


void
Camera::lookAt(GLfloat x, GLfloat y, GLfloat z) {
	__center = Vector3({x, y, z});
}

Vector3 Camera::getEye() {
		return __eye;
}

Vector3 Camera::getCenter() {
		return __center;
}

Vector3 Camera::rightDirection() {
  Vector3 dir = lookDirection();
  return cross(dir, __up).normalize();
}


Vector3 Camera::lookDirection() {
  return (__center - __eye).normalize();
}


