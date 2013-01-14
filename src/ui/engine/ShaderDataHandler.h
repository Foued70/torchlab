#ifndef SHADERDATAHANDLER_H
#define SHADERDATAHANDLER_H

#include <iostream>
#include <map>
#include <string>

#include "opengl.h"
#include "Singleton.h"

#include "Vectors.h"
#include "Matrices.h"

class Shader;
class MatricesManager;

class ShaderDataHandler : public Singleton< ShaderDataHandler > {
public:
	/* Default ctor. */
     ShaderDataHandler();
	
	/**
	 * Updates some data.
	 * @param uniformName Name of the uniform in the shader - also name stored
	 * 				in ShaderDataHandler class.
	 * @param value Value of the uniform variable.
	 */
	void updateData(const std::string&, const Vector2&);
	void updateData(const std::string&, const Vector3&);
	void updateData(const std::string&, const Vector4&);
	void updateData(const std::string&, GLfloat);
	void updateSampler2D(const std::string&, GLint);
	
	/**
	 * Opens the stream.
	 * @param shader Active shader.
	 */
	void openStream(const Shader*);
	
	/**
	 * Closes the stream.
	 */
	void closeStream() { __isStreamOpen = false; }

	
public:
	
	/**
	 * Sends all stored data to the shader.
	 * @param shader Active shader.
	 */
	void __sendDataToShader(const Shader*);
	
	std::map< std::string, Vector2 > __2Dvectors;
	
 	std::map< std::string, Vector3 > __3Dvectors;
	
	std::map< std::string, Vector4 > __4Dvectors;
	
	std::map< std::string, GLfloat > __values;
	
	std::map< std::string, GLuint > __textures;
	
	MatricesManager& __matrices;
	
	bool __isStreamOpen;
	
	const Shader* __activeShader;
	
};

#endif // SHADERDATAHANDLER_H
