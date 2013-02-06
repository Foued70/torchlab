#ifndef FRAME_BUFFER_H
#define FRAME_BUFFER_H

#include <string>
#include "Singleton.h"
#include "opengl.h"

enum RENDER_PASS {
    COLOR_PASS,
    PICKING_PASS,
    DEPTH_PASS
};

class FrameBuffer : public Singleton< FrameBuffer >
{
public:
	FrameBuffer();
	~FrameBuffer();
	
	void bind();
	void unbind();
  
  bool initialize();
	
	void renderToTexture();
	void renderDebugMesh();
	
	void printInfo();
	GLint readPixel(const GLuint& x, const GLuint& y, const GLuint& channel);
  GLfloat readDepthPixel(const GLuint& x, const GLuint& y);
	void saveToFile(const std::string& _filename);
  
  /* Method to send color pass data to the default framebuffer */
  void displayToWindow();

private:
	void __generateFrameBuffer();
	
	void __generateDepthBuffer();
	
	void __generateTexture();
	
	bool __evaluateConfiguration();
  
  GLuint __getAttachmentIndex(const RENDER_PASS&) const;
  void __setReadBuffer(const RENDER_PASS&);
	
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