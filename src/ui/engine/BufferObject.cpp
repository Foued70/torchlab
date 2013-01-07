#include "BufferObject.h"

using namespace std;

BufferObject::BufferObject() :
		vaoID(0),
		vboID({{0, GL_ELEMENT_ARRAY_BUFFER, 0, 0, false}, {0, GL_ARRAY_BUFFER, 0, 0, false}}),
		__gpu(GPUMemory::GetSingleton()) {}

