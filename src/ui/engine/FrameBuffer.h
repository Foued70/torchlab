#ifndef FRAME_BUFFER_H
#define FRAME_BUFFER_H


#include <string>
#include "opengl.h"

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

private:
	void __generateFrameBuffer();
	
	void __generateDepthBuffer();
	
	void __generateTexture();
	
	bool __initialize();
	
	bool __evaluateConfiguration();
	
private:
	GLuint __width;
	GLuint __height;
	/* Pointer to GL's frame buffer object */
	GLuint __frameBufferID;
	
	/* Pointer to GL's render buffer object that houses that depth buffer */
	GLuint __depthBufferID;
	
	/* Pointer to the texture that the  be rendered to */
	GLuint __textureID0;
	GLuint __textureID1;		
};

#endif