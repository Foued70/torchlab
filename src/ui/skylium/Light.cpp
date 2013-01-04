#include <iostream>

#include "Light.h"

#include "Skylium.h"
#include "ShaderDataHandler.h"

#include "config.h"
#include "utils.h"

using namespace std;

Light::Light() :
		__working(true),
		__ambientLight({1.0, 1.0, 1.0, 0.0}),
		__diffuseLight({1.0, 1.0, 1.0, 0.0}),
		__specularLight({1.0, 1.0, 1.0, 0.0}),
		__lightSrc({0.0, 0.0, 0.0}),
		__shaders(ShaderDataHandler::GetSingleton()) {
	log(CONSTRUCTOR, "Light (0, 0, 0) constructed.");
}

Light::Light(const sVector3D &_position) :
		__working(true),
		__ambientLight({1.0, 1.0, 1.0, 0.0}),
		__diffuseLight({1.0, 1.0, 1.0, 0.0}),
		__specularLight({1.0, 1.0, 1.0, 0.0}),
		__lightSrc(_position),
		__shaders(ShaderDataHandler::GetSingleton()) {
	log(CONSTRUCTOR, "Light (%f, %f, %f) constructed.", __lightSrc.x, __lightSrc.y, __lightSrc.z);
}

Light::Light(GLfloat _x, GLfloat _y, GLfloat _z) :
		__working(true),
		__ambientLight({1.0, 1.0, 1.0, 0.0}),
		__diffuseLight({1.0, 1.0, 1.0, 0.0}),
		__specularLight({1.0, 1.0, 1.0, 0.0}),
		__lightSrc({_x, _y, _z}),
		__shaders(ShaderDataHandler::GetSingleton()) {
	log(CONSTRUCTOR, "Light (%f, %f, %f) constructed.", __lightSrc.x, __lightSrc.y, __lightSrc.z);

}

Light::~Light() {
	log(DESTRUCTOR, "Light destructed.");
	
}

void
Light::setAmbient(const sColor& _col) {
	__ambientLight = _col;
	log(PARAM, "Ambient colour set (%f, %f, %f, %f).", _col.r, _col.g, _col.b, _col.a);

}

void
Light::setDiffuse(const sColor& _col) {
	__diffuseLight = _col;
	log(PARAM, "Diffuse colour set (%f, %f, %f, %f).", _col.r, _col.g, _col.b, _col.a);
}

void
Light::setSpecular(const sColor& _col) {
	__specularLight = _col;
	log(PARAM, "Specular colour set (%f, %f, %f, %f).", _col.r, _col.g, _col.b, _col.a);
}

void
Light::setSrcPos(const sVector3D& _pos) {
	__lightSrc = _pos;
	log(PARAM, "Light source set (%f, %f, %f).", _pos.x, _pos.y, _pos.z);
}

void
Light::move(const sVector3D& _mov) {
	__lightSrc += _mov;
}

void
Light::toggle() {
	__working = !__working;
}

void
Light::makeLight(unsigned _count) const {
	if (!__working)
		return;
	string c = T2String(_count);
	__shaders.updateData("sLightSource[" + c + "].ambient", __ambientLight);
	__shaders.updateData("sLightSource[" + c + "].diffuse", __diffuseLight);
	__shaders.updateData("sLightSource[" + c + "].specular", __specularLight);
	__shaders.updateData("sLightSource[" + c + "].position", sVector4D( {
			__lightSrc[0],
			__lightSrc[1],
			__lightSrc[2],
			1.0f
		} ));
	__shaders.updateData("sLightSource[" + c + "].constantAttenuation", 1.0f);
	__shaders.updateData("sLightSource[" + c + "].linearAttenuation", 0.0f);
	__shaders.updateData("sLightSource[" + c + "].quadraticAttenuation", 0.0f);
}