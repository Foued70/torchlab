#ifndef FRAME_BUFFER_H
#define FRAME_BUFFER_H


#include <string>
#include "opengl.h"

//class Shader;
//class ShaderDataHandler;
//class Mesh;
//class Material;
//class Object;

class FrameBuffer
{
public:
	FrameBuffer();
	~FrameBuffer();
	
	void bind();
	void unbind();
	
	void renderToTexture();
	void renderDebugMesh();
	
	void printInfo();
	GLint readPixel(GLint x, GLint y);
	void saveToFile(const std::string& _filename);
	
	//void renderDebugTexture();
	
private:
	void __generateFrameBuffer();
	
	void __generateDepthBuffer();
	
	void __generateTexture();
	
	bool __initialize();
	
	bool __evaluateConfiguration();
	
	//void __loadFullscreenBillboard();
	
private:
	GLuint __width;
	GLuint __height;
	/* Pointer to GL's frame buffer object */
	GLuint __frameBufferID;
	
	/* Pointer to GL's render buffer object that houses that depth buffer */
	GLuint __depthBufferID;
	
	/* Pointer to the texture that the  be rendered to */
	GLuint __textureID;
	GLuint __textureID2;
	
	/* Pointer to billboard debugging object */
	//Object* __object;
	
	
	/* Pointer to the fbo's shader */
	//Shader* __shader;
	
	/* Pointer to a billboard mesh, used to display the fbo as a texture when debugging */
	//Mesh* __mesh;
	
	//Material* __material;
	
	//ShaderDataHandler& __shaders;
	
	
	
	//GLuint __billboardVBO;
	
	//GLuint __texID;
	//GLuint __billboardProgram;
		
};

#endif