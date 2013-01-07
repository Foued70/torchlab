#ifndef GPUMEMORY_H
#define GPUMEMORY_H

#include <vector>

#include "Singleton.h"

class BufferObject;
struct vboData;

enum vboUsage {
	STATIC_DRAW
};

enum {
	READ = 1,
	WRITE = 2
};


class GPUMemory : public Singleton < GPUMemory > {
	
	/*
	 * This is class that handles VAOs and VBOs.
	 * It provides the basic in-out data interface.
	 */
	
public:
	
	/**
	 * Default constructor just sends some info to stdout.
	 */
	GPUMemory();
	
	/**
	 * Frees all the memory.
	 */
	~GPUMemory();
	
	/**
	 * Prepares the room in GPU's buffer.
	 * @param buffer Gets the params - elementsCount and dataSize,
	 * 		sends back vao and vbos IDs.
	 * @param usage Usage - GL VBO usage.
	 */
	void prepareRoom(BufferObject&, vboUsage = STATIC_DRAW);
	
	/**
	 * Sends data to VBOs.
	 * @param buffer Buffer param.
	 * @param target 0 or 1.
	 * @param data Elements pointer.
	 * @return False if something went wrong - see log.
	 */
	void sendData(const BufferObject&, short unsigned, void*);
	
	/**
	 * Queries OpenGL to get the amount of used memory.
	 * @param buffer Buffer to be queried.
	 * @return Size of buffer.
	 */
	unsigned getBufferSize(const BufferObject&);
	
	/**
	 * Deletes the buffers.
	 * @param bufferParams Which buffer to delete.
	 */
	void deleteBuffers(BufferObject&);
	
	/**
	 * Maps the VBO target.
	 * @param buffer Buffer to be mapped.
	 * @param target Target - ELEMENT_ or DATA_ARRAY.
	 * @param access Access - READ, WRITE or both.
	 * @return Pointer to mapped range.
	 */
	void * mapBuffer(BufferObject&, short unsigned, unsigned);
	
	/**
	 * Unmaps the VBO target.
	 * @param buffer Buffer to be unmapped.
	 * @param target Target - ELEMENT_ or DATA_ARRAY.
	 * @return False if OpenGL sais so.
	 */
	bool unmapBuffer(BufferObject&, short unsigned);
	
	/**
	 * Binds specified VBO object.
	 * @param buffer VAO.
	 * @param target Buffer to bind.
	 */
	void bind(BufferObject&, short unsigned);
	
	/**
	 * Unbinds specified VBO object.
	 * @param buffer VAO.
	 * @param target Buffer to unbind.
	 */
	void unbind(BufferObject&, short unsigned);
	
	size_t getVBOUsage() { return __vboUsage; }
	
private:
	/* Buffers vectors */
	std::vector< BufferObject* > __buffers;
	
	size_t __vboUsage;
	
};

#endif // GPUMEMORY_H
