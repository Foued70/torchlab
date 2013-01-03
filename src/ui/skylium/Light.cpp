/*
     _    _      _   _                  
    | |  (_)__ _| |_| |_   __ _ __ _ __ 
    | |__| / _` | ' \  _|_/ _| '_ \ '_ \
    |____|_\__, |_||_\__(_)__| .__/ .__/
           |___/             |_|  |_|   
       
    Copyright (C) 2011  Michał Garapich garrappachc@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>

#include "Light.h"

#include "Skylium.h"
#include "ShaderDataHandler.h"

#include "defines.h"
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