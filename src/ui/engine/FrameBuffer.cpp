#include "FrameBuffer.h"

//#include "Shader.h"
//#include "ShaderDataHandler.h"
//#include "Mesh.h"
//#include "Material.h"
//#include "Object.h"
#include "utils.h"

#include <fstream>

using namespace std;

FrameBuffer::FrameBuffer() :
	__width(800),
	__height(600),
	__frameBufferID(0),
	__depthBufferID(0),
	__textureID0(0),
	__textureID1(0)
{
	log(CONSTRUCTOR, "Frame Buffer Constructed");
	
	if ( !__initialize() ) {
		//TO DO: exit routine or place framebuffer in some sort of "disabled" state
	}
	
	unbind();
}

bool FrameBuffer::__initialize()
{		
	__generateFrameBuffer();
	__generateDepthBuffer();
	__generateTexture();
	
	return __evaluateConfiguration();
}

FrameBuffer::~FrameBuffer()
{	
	//glDeleteBuffers(1, &__billboardVBO);
	//glDeleteProgram(__billboardProgram);
	
	//GLuint frameBuffers[1] = {__frameBufferID};
	//glDeleteFrameBuffers(1, frameBuffers);
	
	//GLuint renderBuffers[1] = {__renderBufferID};
	//glDeleteRenderbuffers(1, renderBuffers);
	
	//glDeleteTextures(1, &__debugTexture);
	//glDeleteRenderBuffers(1, &__depthBuffer);
}

void FrameBuffer::__generateFrameBuffer()
{
	glGenFramebuffers(1, &__frameBufferID);
	bind();
}

void FrameBuffer::bind()
{
	glBindFramebuffer(GL_FRAMEBUFFER, __frameBufferID);
}

void FrameBuffer::unbind()
{
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void FrameBuffer::__generateDepthBuffer()
{
	glGenRenderbuffers(1, &__depthBufferID);
	glBindRenderbuffer(GL_RENDERBUFFER, __depthBufferID);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, __width, __height);
	glFramebufferRenderbuffer(	GL_DRAW_FRAMEBUFFER, 
								GL_DEPTH_ATTACHMENT, 
								GL_RENDERBUFFER,
								__depthBufferID );
	glBindRenderbuffer(GL_RENDERBUFFER, __depthBufferID);
}

void FrameBuffer::__generateTexture()
{
	glGenTextures(1, &__textureID0);
	glBindTexture(GL_TEXTURE_2D, __textureID0);
	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); 
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, __width, __height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
	glBindTexture(GL_TEXTURE_2D, 0);
	
	glFramebufferTexture2D(	GL_FRAMEBUFFER, 
							GL_COLOR_ATTACHMENT0,
							GL_TEXTURE_2D, 
							__textureID0, 
							0);
	
	glGenTextures(1, &__textureID1);
	glBindTexture(GL_TEXTURE_2D, __textureID1);
	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); 
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32UI, __width, __height, 0, GL_RGB_INTEGER, GL_UNSIGNED_INT, NULL);
	glBindTexture(GL_TEXTURE_2D, 0);
	
	glFramebufferTexture2D(	GL_FRAMEBUFFER, 
							GL_COLOR_ATTACHMENT1,
							GL_TEXTURE_2D,  
							__textureID1, 
							0);

}

bool FrameBuffer::__evaluateConfiguration()
{	
	switch ( glCheckFramebufferStatus(GL_FRAMEBUFFER) ) {
		case GL_FRAMEBUFFER_COMPLETE:
			return true;
			
		case GL_FRAMEBUFFER_UNDEFINED:
			log(WARN, "Frame buffer %d configuration failed. Frame buffer target set to default framebuffer. Default frame buffer is undefined.", (int)__frameBufferID);
			return false;
			
		case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
			log(WARN, "Frame buffer %d configuration failed. Attachment incomplete.", (int)__frameBufferID);
			return false;
			
		case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
			log(WARN, "Frame buffer %d configuration failed. Frame buffer requires at least one texture attachment.", (int)__frameBufferID);
			return false;
			
		case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
			log(WARN, "Frame buffer %d configuration failed. Incomplete draw buffer.", (int)__frameBufferID);
			return false;
			
		case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
			log(WARN, "Frame buffer %d configuration failed. Incomplete read buffer.", (int)__frameBufferID);
			return false;
			
		case GL_FRAMEBUFFER_UNSUPPORTED:
			log(WARN, "Frame buffer %d configuration failed. Attached image is of an unsupported format.", (int)__frameBufferID);
			return false;
			
		case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
			log(WARN, "Frame buffer %d configuration failed. Incomplete multisample.", (int)__frameBufferID);
			return false;
			
		case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS:
			log(WARN, "Frame buffer %d configuration failed. Incomplete layer targets.", (int)__frameBufferID);
			return false;

		default:
			log(WARN, "Frame buffer %d configuration failed. Unknown return status from glCheckFramebufferStatus(GL_FRAMEBUFFER). %d", (int)__frameBufferID, glCheckFramebufferStatus(GL_FRAMEBUFFER));
			return false;
		
	}
}

void FrameBuffer::renderToTexture()
{
	bind();

	GLenum attachments[3] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2};
  glDrawBuffers(3, attachments);  
}

void FrameBuffer::printInfo()
{
	bind();

	int res;
	
	int maxColorAttachments;
	
    glGetIntegerv(GL_MAX_COLOR_ATTACHMENTS, &res);
    log(WARN, "Max Frame Buffer Color Attachments: %d", res);

	GLint buffer;
	int i = 0;
	do {
		glGetIntegerv(GL_DRAW_BUFFER0+i, &buffer);

		if (buffer != GL_NONE) {
			log(WARN, "Shader Output Location %d - color attachment %d", i, buffer - GL_COLOR_ATTACHMENT0);
			
			glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER, buffer, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE, &res);
			log(WARN, "Attachment Type: %s", (res==GL_TEXTURE) ? "Texture":"Render Buffer");
			
			glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER, buffer, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, &res);
			log(WARN, "Attachment object name: %d", res);
		}
		++i;

	} while (buffer != GL_NONE);
		
	unbind();

}

GLint FrameBuffer::readPixel(GLint x, GLint y)
{
	//OpenGL saves its framebuffer pixels upsidedown. Need to flip y to get intended data.
	GLuint flippedY = (__height -1) - y;
  
  //R, G, B channels
  GLuint pixel[3];
	
	//Currently reads the Red component of the Pixel.
	//glReadBuffer chooses which output to read from.
	
	bind();
	glReadBuffer(GL_COLOR_ATTACHMENT1);
	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glReadPixels(	(GLsizei)x,
				 	(GLsizei)flippedY,
					(GLsizei)1,
					(GLsizei)1,
					GL_RGB_INTEGER,
					GL_UNSIGNED_INT,
					&pixel );
  
  glReadBuffer(GL_NONE);
	unbind();
	return (GLint)pixel[0];
}

void FrameBuffer::saveToFile(const string& _filename)
{
	const unsigned int channels = 3;
  const unsigned int bytesPerChannel = 4;
	unsigned int fileSizeIn = __width*__height*channels;
  unsigned int fileSizeOut = fileSizeIn*bytesPerChannel;
  
  GLuint* pixelsIn = new GLuint[fileSizeIn];
	char* pixelsOut = new char[fileSizeOut];
  
	string __filename = _filename; __filename.append(".raw");
	
	bind();
	glReadBuffer(GL_COLOR_ATTACHMENT1);
	glPixelStorei(GL_PACK_ALIGNMENT, 1);
  /*
	glReadPixels(   0,
				 	        0,
					        (GLsizei)__width,
                  (GLsizei)__height,
					        GL_RGB_INTEGER,
					        GL_UNSIGNED_INT,
					        pixelsIn );
                  */
  
	glBindTexture(GL_TEXTURE_2D, __textureID1);
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB_INTEGER, GL_UNSIGNED_INT, pixelsIn);
	glBindTexture(GL_TEXTURE_2D, 0);
  
  glReadBuffer(GL_NONE);
  unbind();

  //Convert Data to a viewable format
  unsigned int rowOffset;
  unsigned int columnOffset; 
  for(unsigned int row = 0; row < __height; row++) {
    rowOffset = (row * __width * channels);
    for(unsigned int column = 0; column < __width; column++) {
      columnOffset = (column * channels);
      for(unsigned int channel = 0; channel < channels; channel++) {
          pixelsOut[(rowOffset*bytesPerChannel) + (columnOffset*bytesPerChannel) + (channel*bytesPerChannel) + 0] = char(pixelsIn[rowOffset + columnOffset + channel] >> 24);
          pixelsOut[(rowOffset*bytesPerChannel) + (columnOffset*bytesPerChannel) + (channel*bytesPerChannel) + 1] = char(pixelsIn[rowOffset + columnOffset + channel] >> 16);
          pixelsOut[(rowOffset*bytesPerChannel) + (columnOffset*bytesPerChannel) + (channel*bytesPerChannel) + 2] = char(pixelsIn[rowOffset + columnOffset + channel] >> 8);
          pixelsOut[(rowOffset*bytesPerChannel) + (columnOffset*bytesPerChannel) + (channel*bytesPerChannel) + 3] = char(pixelsIn[rowOffset + columnOffset + channel]);
      }
    }
  }
	
  /*
	//OpenGL populates the pixel data bottom to top.
	//Flipping data vertically for correct orientation in external applications.
	for(unsigned int row = 0; row < (__height/2); row++) {
		unsigned int oppositeRow = __height - 1 - row;
		unsigned int rowSize = __width*channels;
		for(unsigned int pixelChannel = 0; pixelChannel < rowSize; pixelChannel++) {
			swap(pixelsOut[pixelChannel+(row*rowSize)], pixelsOut[pixelChannel+(oppositeRow*rowSize)]);
		}
	}
  */

	ofstream imageFile(__filename.c_str(), ios::out | ios::binary);
	imageFile.write(pixelsOut, fileSizeOut);
	delete[] pixelsIn;
  delete[] pixelsOut;
}
