#ifndef CAMERA_H
#define CAMERA_H

#include <stack>

#include "opengl.h"
#include "Vectors.h"
#include "Matrices.h"

class MatricesManager;

/**
 * WARNING
 * In the FPP camera __center is a vector, not the "lookAt" point.
 * It is normalized vector, which specifies the "lookAt" for the (0,0,0) observer.
 * That is why gluLookAt(..) is being called with center as __center + __eye.
 */

class Camera {

public:
	
	/**
	 * Default ctor, camera in (0,0,0) position.
	 */
	Camera();
	
	/**
	 * Ctor that gets camera's location coords.
	 * @param x X coord.
	 * @param y Y coord.
	 * @param z Z coord.
	 * @param type Camera type. FPP, TPP, SPHERICAL
	 */
	Camera(GLfloat, GLfloat, GLfloat);
	
	/**
	 * Destructor just sends some output.
	 */
	virtual ~Camera();
	
	/**
	 * GL_PROJECTION;
	 * Called only when parameters change.
	 */
	void setProjection();

	/**
	 * GL_MODELVIEW;
	 * Called only when parameters change.
	 */
  void setView();

  void calcUp();

	/**
	 * Moves the eye relative to the view/screen.
	 * @param x horizontal.
	 * @param y vertical.
	 * @param z forward/back.
	 */
	void moveEye(GLfloat, GLfloat, GLfloat);

	/**
	 * Rotates the camera around center by relative x and y values.
	 * @param x X rotation.
	 * @param y Y rotation.
	 * @param z Z rotation.
	 */
  void rotateAroundCenter(GLfloat _x, GLfloat _y);
	
  /**
	 * Sets the "lookAt" point.
	 * @param x X coord.
	 * @param y Y coord.
	 * @param z Z coord.
	 */
	void lookAt(GLfloat, GLfloat, GLfloat);
	
	/**
	 * @return The __eye's coords.
	 */
	Vector3 getEye();
	
	/**
	 * @return The __center's coords.
	 */
	Vector3 getCenter();
   
  Vector3 rightDirection();
  Vector3 lookDirection();
	
private:
	
	/*** setProjection ***/
	GLfloat __fovy;
	GLfloat __zNear;
	GLfloat __zFar;

	/*** setView ***/
	
	/* Camera's position */
	Vector3 __eye;

	/* LookAt position/vector */
	Vector3 __center;

	/* Up vector, (0, 0, 1) */
	Vector3 __up;
	
	/* Angle of the camera */
	Vector3 __angle;
	
	/* Window dimensions */
	GLsizei __windowHeight;
	GLsizei __windowWidth;
	
	MatricesManager& __matrices;
  
  
};

#endif // CAMERA_H