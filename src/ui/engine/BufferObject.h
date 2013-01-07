#ifndef BUFFEROBJECT_H
#define BUFFEROBJECT_H

#include "GPUMemory.h"
#include "opengl.h"

#define ELEMENTS_ARRAY	0
#define DATA_ARRAY		1

struct vboData {
	GLuint	vboID;
	GLenum	target;
	
	size_t	dataSize;
	size_t	dataCount;
	
	bool		bound;
};

class BufferObject {
	
	/*
	 * This class keeps the VAO data.
	 */
	
public:
	
	/**
	 * Default constructor. Sets all values to 0.
	 */
	BufferObject();
	
	/**
	 * Prepares the room in GPU's buffer.
	 * @param u Usage - GL VBO usage.
	 */
	inline void prepareRoom(vboUsage _u = STATIC_DRAW) { __gpu.prepareRoom(*this, _u); }
	
	/**
	 * Sends data to VBOs.
	 * @param target 0 or 1.
	 * @param data Elements pointer.
	 * @return False if something went wrong - see log.
	 */
	inline void sendData(short unsigned _t, void* _d) { __gpu.sendData(*this, _t, _d); }
	
	/**
	 * Queries OpenGL to get the amount of used memory.
	 * @return Size of buffer.
	 */
	inline unsigned getBufferSize() { return __gpu.getBufferSize(*this); }
	
	/**
	 * Deletes the buffers.
	 */
	inline void deleteBuffers() { __gpu.deleteBuffers(*this); }
	
	/**
	 * Maps the VBO target.
	 * @param target Target - ELEMENT_ or DATA_ARRAY.
	 * @param access Access - READ, WRITE or both.
	 * @return Pointer to mapped range.
	 */
	inline void * mapBuffer(short unsigned _t, unsigned _a) { return __gpu.mapBuffer(*this, _t, _a); }
	
	/**
	 * Unmaps the VBO target.
	 * @param target Target - ELEMENT_ or DATA_ARRAY.
	 * @return False if OpenGL sais so.
	 */
	inline bool unmapBuffer(short unsigned _t) { return __gpu.unmapBuffer(*this, _t); }
	
	/**
	 * Binds specified VBO object.
	 * @param target Buffer to bind.
	 */
	inline void bind(short unsigned _t) { __gpu.bind(*this, _t); }
	
	/**
	 * Unbinds specified VBO object.
	 * @param target Buffer to unbind.
	 */
	inline void unbind(short unsigned _t) { __gpu.unbind(*this, _t); }
	
	
	
	/* OpenGL VAO buffer pointer */
	GLuint	vaoID;
	
	vboData	vboID[2];
	
private:
	
	/* GPUMemory instance */
	GPUMemory& __gpu;
	
};

#endif // BUFFEROBJECT_H
