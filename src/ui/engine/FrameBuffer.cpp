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
	__textureID(0),
	__textureID2(0)
{
	log(CONSTRUCTOR, "Frame Buffer Constructed");
	//log(WARN, "sFragColor attribute location = %d", (int)__shader -> getAttribLocation("sFragColor"));
	
	if ( !__initialize() ) {
		//TO DO: exit routine or place framebuffer in some sort of "disabled" state
	}
	
	unbind();
	
	printInfo();
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
	glGenTextures(1, &__textureID);
	glBindTexture(GL_TEXTURE_2D, __textureID);
	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); 
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, __width, __height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
	glBindTexture(GL_TEXTURE_2D, 0);
	
	glFramebufferTexture2D(	GL_FRAMEBUFFER, 
							GL_COLOR_ATTACHMENT0,
							GL_TEXTURE_2D, 
							__textureID, 
							0);
	
	glGenTextures(1, &__textureID2);
	glBindTexture(GL_TEXTURE_2D, __textureID2);
	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); 
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, __width, __height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
	glBindTexture(GL_TEXTURE_2D, 0);
	
	glFramebufferTexture2D(	GL_FRAMEBUFFER, 
							GL_COLOR_ATTACHMENT1,
							GL_TEXTURE_2D,  
							__textureID2, 
							0);

}

bool FrameBuffer::__evaluateConfiguration()
{
	//GLenum DrawBuffers[1] = {GL_COLOR_ATTACHMENT0};
	//glDrawBuffers(1, DrawBuffers);
	
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

void FrameBuffer::renderDebugMesh()
{
	//__shader -> toggle();
	
	//__mesh -> show();
	
	//__shader -> toggle();
}

void FrameBuffer::printInfo()
{
	bind();

	int res;
	
	int maxColorAttachments;

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
	GLuint redValue;
	
	glReadPixels(	x,
				 	y,
					(GLsizei)1,
					(GLsizei)1,
					GL_RED,
					GL_UNSIGNED_BYTE,
					&redValue );
	
	return redValue;
}

void FrameBuffer::saveToFile(const string& _filename)
{
	const unsigned int width = 800, height = 600, channels = 4;
	unsigned int fileSize = width*height*channels;
	char* pixels = new char[fileSize];
	string __filename = _filename; __filename.append(".raw");
	
	bind();
	
	//glReadBuffer(GL_COLOR_ATTACHMENT0+(GLuint)1);
	//glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
	glReadBuffer(GL_COLOR_ATTACHMENT1);
	//(GL_UNPACK_ALIGNMENT, 1);
	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	
	glBindTexture(GL_TEXTURE_2D, __textureID2);
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
	glBindTexture(GL_TEXTURE_2D, 0);
	/*
	glReadPixels(	0,
					0,
					(GLsizei)width,
					(GLsizei)height,
					GL_RGBA,
					GL_UNSIGNED_BYTE,
					pixels);
	*/
	
	//OpenGL populates the pixel data bottom to top.
	//Flipping data vertically for correct orientation in external applications.
	for(unsigned int row = 0; row < (height/2); row++) {
		unsigned int oppositeRow = height - 1 - row;
		unsigned int rowSize = width*channels;
		for(unsigned int pixelChannel = 0; pixelChannel < rowSize; pixelChannel++) {
			swap(pixels[pixelChannel+(row*rowSize)], pixels[pixelChannel+(oppositeRow*rowSize)]);
		}
	}
	
	ofstream imageFile(__filename.c_str(), ios::out | ios::binary);
	imageFile.write(pixels, fileSize);
	delete[] pixels;
	
	unbind();
}

/*
void FrameBuffer::__loadFullscreenBillboard()
{
	static const GLfloat billboardMesh[] = { 
		-1.0f, -1.0f, 0.0f,
		 1.0f, -1.0f, 0.0f,
	    -1.0f,  1.0f, 0.0f,
	    -1.0f,  1.0f, 0.0f,
	     1.0f, -1.0f, 0.0f,
	     1.0f,  1.0f, 0.0f,
	};
	
	__billboardVBO = 0;
	glGenBuffers(1, &__billboardVBO);
	glBindBuffer(GL_ARRAY_BUFFER, billboardMesh);
	glBufferData(GL_ARRAY_BUFFER, sizeof(billboardMesh), billboardMesh, GL_STATIC_DRAW);
	
	__billboardProgram = LoadShaders("shaders/debugFBO.vert", "shaders/debugFBO.frag");
	__texID = glGetUniformLocation(billboardProgram, "fboData");
	//GLuint __vertexID = glGetUniformLocation(billboardProgram, "vertexID");
}

void FrameBuffer::renderDebugTexture()
{
	unbind();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	glUseProgram(__billboardProgram);
	
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, __debugTexture);
	glUniform1i(__texID, 0);
	//glUniform1i(__vertexID, lookupmeshdata and get vertex index and add 1 to it so that there is a difference between nothing and index 0);
	
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, __billboardVBO);
	glVertexAttribPointer(
		0,			
		3,
		GL_FLOAT,
		GL_FALSE,
		0,
		(void*)0
	);
	
	glDrawArrays(GL_TRIANGLES, 0, 6);
	glDisableVertexAttribArray(0);
	
	//swap buffers
	
	//draw()
}
*/
