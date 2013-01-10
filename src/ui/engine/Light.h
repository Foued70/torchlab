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
	Light(const Vector3&);

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
	void setSrcPos(const Vector3&);
	
	/**
	 * Adds the movement vector to the source vector.
	 * @param mov Movement vector.
	 */
	void move(const Vector3&);
	
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
	Vector3 __lightSrc;
	
	ShaderDataHandler& __shaders;

};
#endif // LIGHT_H