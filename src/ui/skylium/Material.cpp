/*
     __  __      _           _      _                
    |  \/  |__ _| |_ ___ _ _(_)__ _| |  __ _ __ _ __ 
    | |\/| / _` |  _/ -_) '_| / _` | |_/ _| '_ \ '_ \
    |_|  |_\__,_|\__\___|_| |_\__,_|_(_)__| .__/ .__/
                                          |_|  |_|  
                                          
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

#include "Material.h"

#include "Texture.h"
#include "Skylium.h"
#include "Shader.h"
#include "ShaderDataHandler.h"

#include "defines.h"
#include "config.h"
#include "utils.h"

using namespace std;

Material::Material(const string &_name) :
		name(_name),
		__mAmbient({0.2, 0.2, 0.2, 1.0}),
		__mDiffuse({0.8, 0.8, 0.8, 1.0}),
		__mSpecular({0.0, 0.0, 0.0, 1.0}),
		__mEmission({0.0, 0.0, 0.0, 1.0}),
		__mAlpha(1.0),
		__mShininess(0),
		__textures(0),
		__shaders(ShaderDataHandler::GetSingleton()) {
	log(CONSTRUCTOR, "Material (\"%s\") constructed.", name.c_str());
}

Material::Material(const Material &_orig) :
		name(_orig.name),
		__mAmbient(_orig.__mAmbient),
		__mDiffuse(_orig.__mDiffuse),
		__mSpecular(_orig.__mSpecular),
		__mEmission(_orig.__mEmission),
		__mAlpha(_orig.__mAlpha),
		__mShininess(_orig.__mShininess),
		__textures(0),
		__shaders(ShaderDataHandler::GetSingleton()) {
	__textures = _orig.__textures;
	log(CONSTRUCTOR, "Material (\"%s\") constructed as a copy.", name.c_str());
}

Material::~Material() {
	log(DESTRUCTOR, "Material (\"%s\") destructed.", name.c_str());
}

void
Material::loadMaterial(const sColor &_param, unsigned _type) {
	if (_type & MATERIAL_AMBIENT)
		__mAmbient = _param;
	else if (_type & MATERIAL_DIFFUSE)
		__mDiffuse = _param;
	else if (_type & MATERIAL_SPECULAR)
		__mSpecular = _param;
}

void
Material::loadAlpha(GLfloat _alpha) {
	__mAlpha = _alpha;
}

void
Material::loadShininess(GLfloat _shininess) {
	__mShininess = _shininess;
}

void
Material::setTextures() {
	if (__textures.empty())
		return;
}

void
Material::setMaterial() {
	
	__shaders.updateData("sFrontMaterial.ambient", __mAmbient);
	__shaders.updateData("sFrontMaterial.diffuse", __mDiffuse);
	__shaders.updateData("sFrontMaterial.specular", __mSpecular);
	__shaders.updateData("sFrontMaterial.shininess", __mShininess);
	__shaders.updateData("sFrontMaterial.emission", __mEmission);
	
	for (unsigned i = 0; i < __textures.size(); ++i)
		__textures[i] -> setTexture(i);
}

void
Material::unsetTextures() {
	if (__textures.empty())
		return;
	
	for (unsigned i = 0; i < __textures.size(); ++i)
		__textures[i] -> unsetTexture(i);
}

void
Material::appendTexture(Texture *_tex) {
	__textures.push_back(_tex);
}