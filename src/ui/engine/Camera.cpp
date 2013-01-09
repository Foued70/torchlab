#include <iostream>
#include <cmath>

#include "Camera.h"
#include "MatricesManager.h"
#include "utils.h"

static const double PI = 3.1415265359;
static const double PIdiv180 = PI/180.0;
static const double PIdiv2 = PI/2;

using namespace std;

Camera::Camera(const cType &_type) :
		__type(_type),
		__fovy(45.0),
		__zNear(0.0001),
		__zFar(1000.0),
		__eye({0, 0, 0}),
		__center({0, 0, 0}),
		__up({0, 0, 1}),
		__range(20),
		__matrices(MatricesManager::GetSingleton()) {
	log(CONSTRUCTOR, "Camera constructed.");
}

Camera::Camera(GLfloat _x, GLfloat _y, GLfloat _z, const cType &_type) :
		__type(_type),
		__fovy(45.0),
		__zNear(0.0001),
		__zFar(1000.0),
		__eye({_x, _y, _z}),
		__center({0, 0, 0}),
		__up({0, 0, 1}),
		__range(20),
		__matrices(MatricesManager::GetSingleton()) {

  if (__type == SPHERICAL)
      __eye.normalize();
	
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
	if (__type == FPP)
		__matrices.sLookAt(__eye, __center + __eye, __up);
	else if (__type == SPHERICAL)
		__matrices.sLookAt((__eye * __range) + __center, __center, __up);
	
	// we have modelViewMatrix set, we can count the normal matrix now.
	__matrices.produceNormalMatrix();
}

void
Camera::moveCamera(GLfloat movX, GLfloat movY, GLfloat movZ) {
	if (__type == FPP) {
		__eye.x += (__center.x * movZ);
		__eye.z += (__center.z * movZ);
		__eye.x += (__center.z * movX * -1);
		__eye.z += (__center.x * movX);
		__eye.y += movY / 10;
	} else if (__type == SPHERICAL) {
		__center.x += (__eye.x * movZ * -0.1);
		__center.z += (__eye.z * movZ * -0.1);
		__center.x += (__eye.z * movX * 0.1);
		__center.z += (__eye.x * movX * -0.1);
		__center.y += movY / 10;
	}
  // log(PARAM, "Camera::moveCamera: (%f, %f, %f)", __center.x, __center.y, __center.z);
}


void
Camera::rotateCamera(GLfloat _x, GLfloat _y, GLfloat) {
	if (__type == FPP) {
		/* Update the angle */
		__angle.x -= (GLfloat)(_x / 100);
		__angle.y += (GLfloat)(_y / 200);
	
		if (__angle.y > 90 * PIdiv180) __angle.y = 90 * PIdiv180;
		else if (__angle.y < -90 * PIdiv180) __angle.y = -90 * PIdiv180;
	
		// vector up
		__center.y = sin(__angle.y);
	
		// count new position on x-z
		__center.x = cos(__angle.x - 90);
		__center.z = sin(__angle.x - 90);
	
		__center.normalize();
	} else if (__type == SPHERICAL) {
		__angle.x += (GLfloat)(_x / 200);
		__angle.y -= (GLfloat)(_y / 200);
		
		__eye.x = -1 * cos(__angle.y) * sin(__angle.x - 90);
		__eye.y = sin(__angle.y);
		__eye.z = cos(__angle.y) * cos(__angle.x - 90);
		
		__eye.normalize();
	}
	
  // log(PARAM, "Camera::rotateCamera: center(%f, %f, %f)  eye(%f, %f, %f)", __center.x, __center.y, __center.z, __eye.x, __eye.y, __eye.z);
  
}

void
Camera::lookAt(GLfloat x, GLfloat y, GLfloat z) {
	__center = sVector3D({x, y, z});
	if (__type == FPP)
		__center.normalize();

  // log(PARAM, "Camera::LookAt: (%f, %f, %f)", __center.x, __center.y, __center.z);
}

sVector3D Camera::getEye() {
		return __eye;
}

sVector3D Camera::getCenter() {
		return __center;
}


