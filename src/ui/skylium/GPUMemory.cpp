/*
     ___ ___ _   _ __  __                                       
    / __| _ \ | | |  \/  |___ _ __  ___ _ _ _  _   __ _ __ _ __ 
   | (_ |  _/ |_| | |\/| / -_) '  \/ _ \ '_| || |_/ _| '_ \ '_ \
    \___|_|  \___/|_|  |_\___|_|_|_\___/_|  \_, (_)__| .__/ .__/
                                            |__/     |_|  |_|   
   
    Copyright (C) 2011  Michał Garapich garrappachc@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>

#include "GPUMemory.h"

#include "BufferObject.h"

#include "defines.h"
#include "config.h"
#include "utils.h"

using namespace std;

GPUMemory::GPUMemory() : __vboUsage(0) {
	log(CONSTRUCTOR, "GPUMemory constructed.");
}

GPUMemory::~GPUMemory() {
	log(DESTRUCTOR, "GPUMemory destructed.");
}

void
GPUMemory::prepareRoom(BufferObject& _buffer, vboUsage _usage) {
	glGenVertexArrays(1, &_buffer.vaoID);
	checkGLErrors(AT);
	glGenBuffers(1, &_buffer.vboID[ELEMENTS_ARRAY].vboID);
	checkGLErrors(AT);
	glGenBuffers(1, &_buffer.vboID[DATA_ARRAY].vboID);
	checkGLErrors(AT);
	
	glBindVertexArray(_buffer.vaoID);
	checkGLErrors(AT);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _buffer.vboID[ELEMENTS_ARRAY].vboID);
	checkGLErrors(AT);
	glBindBuffer(GL_ARRAY_BUFFER, _buffer.vboID[DATA_ARRAY].vboID);
	checkGLErrors(AT);
	
	GLenum usage;
	switch (_usage) {
		case STATIC_DRAW:
			usage = GL_STATIC_DRAW;
			break;
	}
	
	glBufferData(
			GL_ELEMENT_ARRAY_BUFFER,
			_buffer.vboID[ELEMENTS_ARRAY].dataSize,
			NULL, usage
		);
	glBufferData(
			GL_ARRAY_BUFFER,
			_buffer.vboID[DATA_ARRAY].dataSize,
			NULL, usage
		);
	
	checkGLErrors(AT);
	
	// restore previous VAO state
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	checkGLErrors(AT);
	
	__buffers.push_back(&_buffer);
}

void
GPUMemory::sendData(const BufferObject& _buffer, short unsigned _target, void* _data) {
	glBindVertexArray(_buffer.vaoID);
	checkGLErrors(AT);
	
	if (!_buffer.vboID[_target].bound) {
		glBindBuffer(_buffer.vboID[_target].target, _buffer.vboID[_target].vboID);
		checkGLErrors(AT);
	}
	
	glBufferSubData(_buffer.vboID[_target].target, 0,
			_buffer.vboID[_target].dataSize,
			_data
		);
	checkGLErrors(AT);

	// restore previous VAO state
	if (!_buffer.vboID[_target].bound) {
		glBindBuffer(_buffer.vboID[_target].target, 0);
		checkGLErrors(AT);
	}
	
	glBindVertexArray(0);
	
	__vboUsage += _buffer.vboID[_target].dataSize;
}

unsigned
GPUMemory::getBufferSize(const BufferObject& _buffer) {
	int elemsSize, dataSize;
	
	glBindVertexArray(_buffer.vaoID);
	checkGLErrors(AT);
	
	if (!_buffer.vboID[ELEMENTS_ARRAY].bound)
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _buffer.vboID[ELEMENTS_ARRAY].vboID);
	glGetBufferParameteriv(GL_ELEMENT_ARRAY_BUFFER, GL_BUFFER_SIZE, &elemsSize);
	checkGLErrors(AT);
	
	if (!_buffer.vboID[DATA_ARRAY].bound)
		glBindBuffer(GL_ARRAY_BUFFER, _buffer.vboID[DATA_ARRAY].vboID);
	glGetBufferParameteriv(GL_ARRAY_BUFFER, GL_BUFFER_SIZE, &dataSize);
	checkGLErrors(AT);
	
	if (!_buffer.vboID[ELEMENTS_ARRAY].bound)
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	if (!_buffer.vboID[DATA_ARRAY].bound)
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	
	glBindVertexArray(0);
	
	return elemsSize + dataSize;
}

void
GPUMemory::deleteBuffers(BufferObject& _buffer) {
	glDeleteBuffers(1, &_buffer.vboID[ELEMENTS_ARRAY].vboID);
	checkGLErrors(AT);
	_buffer.vboID[ELEMENTS_ARRAY].bound = false;
	_buffer.vboID[ELEMENTS_ARRAY].vboID = 0;
	
	glDeleteBuffers(1, &_buffer.vboID[DATA_ARRAY].vboID);
	checkGLErrors(AT);
	_buffer.vboID[DATA_ARRAY].bound = false;
	_buffer.vboID[DATA_ARRAY].vboID = 0;
	
	glDeleteVertexArrays(1, &_buffer.vaoID);
	checkGLErrors(AT);
	_buffer.vaoID = 0;
	
	__vboUsage -= _buffer.vboID[ELEMENTS_ARRAY].dataSize;
	__vboUsage -= _buffer.vboID[DATA_ARRAY].dataSize;
}

void *
GPUMemory::mapBuffer(BufferObject& _buffer, short unsigned _target, unsigned _acs) {
	GLenum access;
	if ((_acs & (READ | WRITE)) == (READ | WRITE))
		access = GL_READ_WRITE;
	else if (_acs & READ)
		access = GL_READ_ONLY;
	else if (_acs & WRITE)
		access = GL_WRITE_ONLY;
	else
		return NULL;
	
	glBindVertexArray(_buffer.vaoID);
	checkGLErrors(AT);
	
	if (!_buffer.vboID[_target].bound) {
		glBindBuffer(_buffer.vboID[_target].target, _buffer.vboID[_target].vboID);
		checkGLErrors(AT);
	}
	
	void* data = glMapBuffer(_buffer.vboID[_target].target, access);
	checkGLErrors(AT);
	
	return data;
}

bool
GPUMemory::unmapBuffer(BufferObject& _buffer, short unsigned _target) {
	GLboolean result = glUnmapBuffer(_buffer.vboID[_target].target);
	checkGLErrors(AT);
	
	if (!_buffer.vboID[_target].bound) {
		glBindBuffer(_buffer.vboID[_target].target, 0);
		checkGLErrors(AT);
	}
	
	glBindVertexArray(0);
	
	return result;
}

void
GPUMemory::bind(BufferObject& _buffer, short unsigned _target) {
	glBindVertexArray(_buffer.vaoID);
	checkGLErrors(AT);
	
	glBindBuffer(_buffer.vboID[_target].target, _buffer.vboID[_target].vboID);
	checkGLErrors(AT);
	
	_buffer.vboID[_target].bound = true;
	
	glBindVertexArray(0);
}

void
GPUMemory::unbind(BufferObject& _buffer, short unsigned _target) {
	glBindVertexArray(_buffer.vaoID);
	checkGLErrors(AT);
	
	glBindBuffer(_buffer.vboID[_target].target,0);
	checkGLErrors(AT);
	
	_buffer.vboID[_target].bound = false;
	
	glBindVertexArray(0);
}