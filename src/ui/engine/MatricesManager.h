#ifndef MATRICESMANAGER_H
#define MATRICESMANAGER_H

#include <stack>

#include "opengl.h"
#include "Singleton.h"
#include "Matrices.h"
#include "Vectors.h"

enum Axis { X, Y, Z };

class MatricesManager : public Singleton< MatricesManager > {
public:
	// default ctor
     MatricesManager();
	
	/* Access to matrices is sometimes useful */
	sMat16 & getModelViewMatrix() { return __modelViewMatrix; }
	const sMat16 & getModelViewMatrix() const { return __modelViewMatrix; }
	
	sMat16 & getProjectionMatrix() { return __projectionMatrix; }
	const sMat16 & getProjectionMatrix() const { return __projectionMatrix; }
	
	sMat9 & getNormalMatrix() { return __normalMatrix; }
	const sMat9 & getNormalMatrix() const { return __normalMatrix; }
	
	/**
	 * Produces a viewing transform.
	 * @param eye Eye point.
	 * @param center The reference point position.
	 * @param up The UP Vector.
	 */
	void sLookAt(const Vector3&, const Vector3&, const Vector3&);
	
	/**
	 * Sets a perspective projection matrix.
	 * @param fovy The field of view angle, in degrees, in the 'y' direction.
	 * @param aspect Species the aspect ratio x:y.
	 * @param zNear Specifies the distance from the viewer to the near clipping plane (>0).
	 * @param zFar Specifies the distance from the viewer to the far clipping plane (>0).
	 */
	void sPerspective(GLfloat, GLfloat, GLfloat, GLfloat);
	
	/**
	 * Generates a pararell projection.
	 * @param left Specify the coordinates for the left vertical clipping planes.
	 * @param right Specify the coordinates for the right vertical clipping planes.
	 * @param bottom Specify the coordinates for the bottom horizontal clipping planes.
	 * @param top Specify the coordinates for the top horizontal clipping planes.
	 * @param nearVal Specify the distances to the nearer depth clipping planes.
	 * @param farVal Specify the distances to the farther depth clipping planes.
	 */
	void sOrtho(GLfloat, GLfloat, GLfloat, GLfloat, GLfloat, GLfloat);
	
	/**
	 * Generates the Normal Matrix.
	 * Normal matrix is the transpose of the inverse of the modelView Matrix.
	 */
	void produceNormalMatrix();
	
	/**
	 * Translate by (x, y, z). The current ModelView Matrix is being multiplied
	 * by the translation matrix.
	 * http://www.opengl.org/sdk/docs/man/xhtml/glTranslate.xml
	 * @param trans Translation vector.
	 */
	void translate(const Vector3&);
	
	/**
	 * Scale along the x, y and z axes. The current ModelView Matrix is being
	 * multiplied by this scale matrix.
	 * http://www.opengl.org/sdk/docs/man/xhtml/glScale.xml
	 * @param scale Scale vector.
	 */
	void scale(const Vector3&);
	
	/**
	 * Procudes a rotation of {x,y,z}-angle along {x,y,z}-axis. The current
	 * ModelView Matrix is being multiplied.
	 * http://www.opengl.org/sdk/docs/man/xhtml/glRotate.xml
	 * @param rotation Rotation vector.
	 */
	void rotate(GLfloat, Axis);
	
	/**
	 * Pushes the current matrix on the stack.
	 */
	void storeModelViewMatrix();
	void storeProjectionMatrix();
	
	/**
	 * Pops the matrix from the stack.
	 */
	void restoreModelViewMatrix();
	void restoreProjectionMatrix();
	
	
	
private:
	
	sMat16 __modelViewMatrix;
	
	sMat16 __projectionMatrix;
	
	sMat9 __normalMatrix;
	
	std::stack< sMat16 > __MVStack;
	
	std::stack< sMat16 > __PStack;
	
	std::stack< sMat9 > __NStack;
	
	
	
};

#endif // MATRICESMANAGER_H
