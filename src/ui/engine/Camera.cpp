#include <iostream>
#include <cmath>

#include "Camera.h"
#include "MatricesManager.h"
#include "FrameBuffer.h"
#include "utils.h"

static const double PI = 3.1415265359;
static const double PIdiv180 = PI/180.0;
static const double PIdiv2 = PI/2;

using namespace std;

Camera::Camera() :
		__fovy(60.0f),
		__zNear(0.00001f),
		__zFar(1000.0),
		__eye({0, 0, 0}),
		__center({0, 0, 0}),
		__up({0, 0, 1}),
		__matrices(MatricesManager::GetSingleton()) {
	log(CONSTRUCTOR, "Camera constructed.");
}

Camera::Camera(GLfloat _x, GLfloat _y, GLfloat _z) :
		__fovy(60.0f),
		__zNear(0.00001f),
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
  float zFactor = dot(dir, Z_AXIS);
  if ( zFactor > 0.999f || zFactor < -0.999f ) {
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
Camera::setEyePosition(GLfloat _x, GLfloat _y, GLfloat _z) {
  __eye = Vector3( {_x, _y, _z} );
  calcUp();
}

void 
Camera::setEyePosition(const Vector3& _position) {
  __eye = _position;
  calcUp();
}

void 
Camera::setCenterPosition(GLfloat _x, GLfloat _y, GLfloat _z) {
  __center = Vector3( {_x, _y, _z} );
  calcUp();
}

void 
Camera::setCenterPosition(const Vector3& _position) {
  __center = _position;
  calcUp();
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
Camera::rotateCenterAroundCamera(GLfloat _x, GLfloat _y) {
  float xAngle = _x;
  float yAngle = _y;
  Vector3 eyePosition = __eye;
  
  // Move center and eye so that eye is at the origin
  __eye =  Vector3({0.0f, 0.0f, 0.0f});
  __center -= eyePosition;
  
  // Rotate around a horizontal axis (y rotation)
  Vector3 unitCenter = __center;
  unitCenter.normalize();
  float zAxisAngle = acos(dot(unitCenter, Z_AXIS));
  
  if (zAxisAngle - yAngle < 0) {
    __center = Z_AXIS * __center.magnitude();
    // rotate up for x instead of eye
    __up = rotateAxisAngle(__up, Z_AXIS, xAngle);
  }
  else if (zAxisAngle - yAngle > PI) {
    __center = Z_AXIS * -__center.magnitude();
    // rotate up for x instead of eye
    __up = rotateAxisAngle(__up, Z_AXIS, xAngle);
  }
  else {
    Vector3 axis = cross(unitCenter, __up).normalize();
    __center = rotateAxisAngle(__center, axis, yAngle);
    // rotate around Z-up (x rotation)
    __center = rotateAxisAngle(__center, Z_AXIS, xAngle);
  }
  
  // move camera back into position
  __eye = eyePosition;
  __center += eyePosition;
  calcUp();
}

void
Camera::rotateAroundCenter(GLfloat _x, GLfloat _y) {
  float xAngle = _x;
  float yAngle = _y;
  
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

Vector3 
Camera::getEye() const {
		return __eye;
}

Vector3 
Camera::getCenter() const {
		return __center;
}

float 
Camera::getNearPlane() const {
  return __zNear;
}

float 
Camera::getFarPlane() const {
  return __zFar;
}

Vector3 
Camera::rightDirection() {
  Vector3 dir = lookDirection();
  return cross(dir, __up).normalize();
}


Vector3 
Camera::lookDirection() {
  return (__center - __eye).normalize();
}

Vector3 
Camera::cameraToWorld(GLfloat _x, GLfloat _y) const { 
  Vector4 screenPosition = Vector4({  (float)_x/(float)__windowWidth, 
                                      (float)_y/(float)__windowHeight, 
                                      FrameBuffer::GetSingleton().readDepthPixel(_x, _y), 
                                      1.0f });
                                                                      
  screenPosition.r = (screenPosition.r * 2.0f) - 1.0f;
  screenPosition.g = 1.0f - (screenPosition.g * 2.0f); // Invert Y
  screenPosition.b = (screenPosition.b * 2.0f) - 1.0f;
  
  log(PARAM, "screenPosition= %f %f %f", screenPosition.r, screenPosition.g, screenPosition.b);
    
  sMat16 modelViewMatrix = MatricesManager::GetSingleton().getModelViewMatrix();
  sMat16 projectionMatrix = MatricesManager::GetSingleton().getProjectionMatrix();
  sMat16 projectionModelViewMatrix = projectionMatrix * modelViewMatrix;
  sMat16 inverseCameraTransform = projectionModelViewMatrix.inversion();
    
  modelViewMatrix.logAllComponents("ModelViewMatrix");
  projectionMatrix.logAllComponents("projectionMatrix");
  projectionModelViewMatrix.logAllComponents("projectionModelViewMatrix");
  inverseCameraTransform.logAllComponents("inverseCameraTransform");
  Vector4 viewPosition = inverseCameraTransform * screenPosition;
  
  log(PARAM, "Camera %f %f to world= %f %f %f", _x, _y, viewPosition.r, viewPosition.g, viewPosition.b );
  
  viewPosition /= viewPosition.a;
  
  log(PARAM, "Camera %f %f to world= %f %f %f", _x, _y, viewPosition.r, viewPosition.g, viewPosition.b );
  
  return Vector3({viewPosition.r, viewPosition.g, viewPosition.b});
}

Vector3 
Camera::worldToCamera(const Vector3& _positionWorld) const {
  sMat16 modelViewMatrix = MatricesManager::GetSingleton().getModelViewMatrix();
  sMat16 projectionMatrix = MatricesManager::GetSingleton().getProjectionMatrix();
  Vector4 positionCamera = (projectionMatrix*modelViewMatrix) * Vector4({_positionWorld.x, _positionWorld.y, _positionWorld.z, 1.0f});
  
  return Vector3({positionCamera.r, positionCamera.g, positionCamera.b});
}


