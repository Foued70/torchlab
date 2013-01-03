/*
     _    _      _   _     _    
    | |  (_)__ _| |_| |_  | |_  
    | |__| / _` | ' \  _|_| ' \ 
    |____|_\__, |_||_\__(_)_||_|
           |___/                

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

#ifndef LIGHT_H
#define LIGHT_H

#include "opengl.h"
#include "Vectors.h"

class Scene;
class ShaderDataHandler;

class Light {
	
	friend class Scene;
	
public:
	/**
	 * Default ctor, (0, 0, 0).
	 */
	Light();

	/**
	 * Ctor that gets the light's source in sVector format.
	 * @param pos Position of the light's source.
	 */
	Light(const sVector3D&);

	/**
	 * Ctor that gets light source in (x, y, z) format.
	 * @param x Light source's X coord.
	 * @param y Y coord.
	 * @param z Z coord.
	 */
	Light(GLfloat, GLfloat, GLfloat);
	
	/**
	 * Just some output.
	 */
	virtual ~Light();
	
	/**
	 * Sets lights colours.
	 * @param col Colour.
	 */
	void setAmbient(const sColor&);
	void setDiffuse(const sColor&);
	void setSpecular(const sColor&);
	
	/**
	 * Sets light source's position.
	 * @param pos Position in (x, y, z).
	 */
	void setSrcPos(const sVector3D&);
	
	/**
	 * Adds the movement vector to the source vector.
	 * @param mov Movement vector.
	 */
	void move(const sVector3D&);
	
	/**
	 * Light on/off
	 */
	void toggle();
	
	/**
	 * Sends data to the shader.
	 * @param count Number of the light.
	 */
	void makeLight(unsigned) const;
	
private:
	
	/* Is light on? */
	bool __working;

	/* Ambient light */
	sColor __ambientLight;

	/* Diffuse light */
	sColor __diffuseLight;

	/* Specular light */
	sColor __specularLight;

	/* Light source's position */
	sVector3D __lightSrc;
	
	ShaderDataHandler& __shaders;

};
#endif // LIGHT_H