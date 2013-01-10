#include <cmath>
#include <iostream>

#include "MatricesManager.h"
#include "utils.h"

using namespace std;

static const double PI = 3.1415265359;
static const double PIdiv180 = PI/180.0;

MatricesManager::MatricesManager() {
	__modelViewMatrix.loadIdentity();
	__projectionMatrix.loadIdentity();
	__normalMatrix.loadIdentity();
	
	log(CONSTRUCTOR, "MatricesManager constructed.");
}

void
MatricesManager::sLookAt(const Vector3& _eye, const Vector3& _center, const Vector3& _up) {
	Vector3 dir = _center - _eye;
	dir.normalize();
	Vector3 perpUpDir = cross(dir, _up);
	perpUpDir.normalize();
	Vector3 up = cross(perpUpDir, dir);
	dir *= -1;
	
	__modelViewMatrix.loadIdentity();
	
	__modelViewMatrix.setRow(0, perpUpDir);
	__modelViewMatrix.setRow(1, up);
	__modelViewMatrix.setRow(2, dir);
	
	sMat16 translation;
	translation.loadIdentity();
	translation[12] = -_eye[0];
	translation[13] = -_eye[1];
	translation[14] = -_eye[2];
	
	__modelViewMatrix *= translation;
}

void
MatricesManager::sPerspective(GLfloat _fovy, GLfloat _aspect, GLfloat _zNear, GLfloat _zFar) {
	if (_zNear == 0) return;
	
	__projectionMatrix.loadIdentity();
	
	GLfloat f = 1 / (tan((_fovy * PIdiv180) / 2)); // ctg(_fovy/2)
	
	__projectionMatrix[0] = f / _aspect;
	__projectionMatrix[5] = f;
	__projectionMatrix[10] = (_zFar + _zNear) / (_zNear - _zFar);
	__projectionMatrix[14] = (2 * _zFar * _zNear) / (_zNear -_zFar);
	__projectionMatrix[11] = -1;
}

void
MatricesManager::sOrtho(GLfloat _left, GLfloat _right,
		GLfloat _bottom, GLfloat _top,
		GLfloat _nearVal, GLfloat _farVal) {
	
	__projectionMatrix.loadIdentity();
	
	__projectionMatrix[0] = 2 / (_right - _left);
	__projectionMatrix[5] = 2 / (_top - _bottom);
	__projectionMatrix[10] = -2 / (_farVal - _nearVal);
	__projectionMatrix[12] = -((_right + _left) / (_right - _left));
	__projectionMatrix[13] = -((_top + _bottom) / (_top - _bottom));
	__projectionMatrix[14] = -((_farVal + _nearVal) / (_farVal - _nearVal));
	__projectionMatrix[15] = 1;
}

void
MatricesManager::translate(const Vector3& _trans) {
	sMat16 temp;
	temp.loadIdentity();
	temp.setColumn(3, _trans);
	
	__modelViewMatrix *= temp;
}

void
MatricesManager::scale(const Vector3& _scale) {
	sMat16 temp;
	temp[0] = _scale.x;
	temp[5] = _scale.y;
	temp[10] = _scale.z;
	temp[15] = 1;
	
	__modelViewMatrix *= temp;
}

void
MatricesManager::rotate(GLfloat _angle, Axis _axis) {
	sMat16 temp;
	temp.loadIdentity();
	
	double radangle = _angle * PIdiv180; // convert to radians
	
	double c = cos(radangle);
	double s = sin(radangle);
	
	double anti_c = 1 - c;
	
	Vector3 rot;
	switch (_axis) {
		case X:
			rot = Vector3({1.0, 0.0, 0.0});
			break;
		case Y:
			rot = Vector3({0.0, 1.0, 0.0});
			break;
		case Z:
			rot = Vector3({0.0, 0.0, 1.0});
			break;
	}
	
	temp[0] = rot.x * rot.x * (anti_c) + c;
	temp[1] = rot.x * rot.y * (anti_c) + (rot.z * s);
	temp[2] = rot.x * rot.z * (anti_c) - (rot.y * s);
	
	temp[4] = rot.x * rot.y * (anti_c) - (rot.z * s);
	temp[5] = rot.y * rot.y * (anti_c) + c;
	temp[6] = rot.y * rot.z * (anti_c) + (rot.x * s);
	
	temp[8] = rot.x * rot.z * (anti_c) + (rot.y * s);
	temp[9] = rot.y * rot.z * (anti_c) - (rot.x * s);
	temp[10] = rot.z * rot.z * (anti_c) + c;
	
	temp[15] = 1;
	
	__modelViewMatrix *= temp;
}

void
MatricesManager::produceNormalMatrix() {
	__normalMatrix = __modelViewMatrix.getMinor(3, 3).inversion();
	__normalMatrix.transpose();
}

void
MatricesManager::storeModelViewMatrix() {
	__MVStack.push(__modelViewMatrix);
}

void
MatricesManager::restoreModelViewMatrix() {
	__modelViewMatrix = __MVStack.top();
	__MVStack.pop();
}

void
MatricesManager::storeProjectionMatrix() {
	__PStack.push(__projectionMatrix);
}

void
MatricesManager::restoreProjectionMatrix() {
	__projectionMatrix = __PStack.top();
	__PStack.pop();
}
