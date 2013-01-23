#ifndef SHADER_H
#define SHADER_H

#include <string>

#include "Object.h"
#include "Vectors.h"
#include "Matrices.h"

#define GL_FRAGMENT_SHADER 0x8B30
#define GL_VERTEX_SHADER	0x8B31
#define GL_COMPILE_STATUS 0x8B81
#define GL_LINK_STATUS 0x8B82

/**
 * High-level shader program.
 */
class Shader {

public:
	/**
	 * Ctor that gets one argument.
	 * @param fileName Source file name for both - vertex and fragment shaders.
	 * 		Vertex shader source file is fileName +".vert", fragment - +".frag".
	 */
	Shader(const std::string&);
	
	/**
	 * Ctor that gets two arguments.
	 * @param vertexCode Full source code for vertex shader.
	 * @param fragmentCode Full source code for fragment shader.
	 */
	Shader(const std::string&, const std::string&);
	
	/**
	 * Detaches and destroys the shader.
	 */
	virtual ~Shader();

	/**
	 * Compiles and links the shader program.
	 * @return False if something went wrong.
	 */
	bool make(  GLuint = 0, const std::string& = "sVertex",
			        GLuint = 1, const std::string& = "sTexCoord",
			        GLuint = 2, const std::string& = "sNormal",
              GLuint = 3, const std::string& = "sFaceIndex"
    		);

	/**
	 * Toggles on/off.
	 */
	void toggle();
	
	/**
	 * Bind shader to an object.
	 * @param dest Pointer to the object that the shader has to be bound to.
	 */
	void bind(Object*);
	
	/**
	 * Unbinds shader from the object.
	 */
	void unbind(Object*);
	
	/**
	 * @return True, if the shader is bound to an object.
	 */
	bool isBound(Object*);
	
	/**
	 * Sends some data to the shader.
	 * @param name Name of variable in the shader.
	 * @param params Vector of N float to be sent.
	 */
	void setUniformFloat(const std::string&, const sVectorBase< GLfloat >&) const;
	void setUniformFloat(const std::string&, GLfloat) const;
	void setUniformInt(const std::string&, GLint) const;	
	/**
	 * Sends a matrix to the shader.
	 * @param name Name of variable in the shader.
	 * @param matrix Matrix of 16 floats to be sent.
	 */
	void setMatrixFloat(const std::string&, const sMat16&) const;
	void setMatrixFloat(const std::string&, const sMat9&) const;
	
	/**
	* Used to lookup the location of a specifed attribute in the shader.
	* @param name Name of attribute in shader
	*/
	GLint getAttribLocation(const std::string&) const;

private:
  std::string __name;
  std::string __vertFile;
	std::string __fragFile;
	
	std::string __vertCode;
	std::string __fragCode;

	GLuint __vertexShader;
	GLuint __fragmentShader;

	GLuint __shaderProgram;

	GLboolean __isRunning;
	
	bool __isCompiled;

};

#endif // SHADER_H
