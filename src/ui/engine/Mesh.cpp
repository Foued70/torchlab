#include <iostream>

#include "Mesh.h"
#include "Shader.h"
#include "utils.h"

// makro to count the position
#define BUFFER_OFFSET(i) ((char *)NULL + (i))

using namespace std;

Mesh::Mesh(const string &_name) :
		name(_name),
		__vertices(0),
		__indices(0),
		__materials(1),
		__smooth(false),
		__usage(STATIC_DRAW),
		__mode(GL_TRIANGLES),
		__isShown(true) {
	
	__materials[0].begin = 0;
	
	log(CONSTRUCTOR, "Mesh (\"%s\") constructed.", name.c_str());
}

Mesh::~Mesh() {
	__buffer.deleteBuffers();
	checkGLErrors(AT);
  __clearAllTriangles();
	log(DESTRUCTOR, "Mesh (\"%s\") destructed.", name.c_str());
}

Triangle* 
Mesh::getTriangleByID(unsigned int _id) {
  if ((_id*3) >= __indices.size()) {
    log(WARN, "Requested a triangle id of %d where the total number of triangles is %d and total verts is %d", _id, __indices.size()/3, __vertices.size());
    return NULL;
  }
  auto it = __triangles.find(_id);
  if ( it != __triangles.end() ) {
    return it->second;
  }
  Vertex* v1 = &__vertices[__indices[(_id*3)]];
  Vertex* v2 = &__vertices[__indices[(_id*3)+1]];
  Vertex* v3 = &__vertices[__indices[(_id*3)+2]];
  Triangle* triangle = new Triangle(TriangleID(0, 0, _id), v1, v2, v3);
  __triangles.insert(make_pair(_id, triangle));
  return triangle;
}

void
Mesh::show() {
	if (!__isShown)
		return;
	// there we go!
	glBindVertexArray(__buffer.vaoID);
	checkGLErrors(AT);
	
	for (MeshRange& m: __materials) {
		if (m.begin == m.end)
			continue;
		
		if (m.material)
			m.material -> setMaterial();
    
		glDrawElements(
				__mode,
				m.end,
				GL_UNSIGNED_INT,
				BUFFER_OFFSET(sizeof(IndicesType) * m.begin)
			);
		checkGLErrors(AT);
		
		if (m.material)
			m.material -> unsetTextures();
	
	}
	glBindVertexArray(0);
}

void
Mesh::loadIntoVbo() {
  log(PARAM, "Loading Mesh into VBO...");
	__buffer.vboID[ELEMENTS_ARRAY].dataCount = __indices.size();
  log(PARAM, "__indices.size() in vbo: %d", __indices.size());
  
	__buffer.vboID[ELEMENTS_ARRAY].dataSize = sizeof(IndicesType) * __indices.size();
	
	__buffer.vboID[DATA_ARRAY].dataCount = __vertices.size();
  log(PARAM, "__vertices.size() in vbo: %d", __vertices.size());
	__buffer.vboID[DATA_ARRAY].dataSize = sizeof(Vertex) * __vertices.size();
  
  for(unsigned int v = 0; v < __vertices.size(); v++) {
    log(PARAM, "Vertex %d= %f %f %f", v, __vertices[v].vertexPosition.x, __vertices[v].vertexPosition.y, __vertices[v].vertexPosition.z);
  }

	
	__buffer.prepareRoom();
	__buffer.sendData(ELEMENTS_ARRAY, &__indices[0]);
	__buffer.sendData(DATA_ARRAY, &__vertices[0]);
	
	log(BUFFER, "Mesh (\"%s\") in the VBO. Size: %u B.", name.c_str(), __buffer.getBufferSize());
	
	__buffer.bind(ELEMENTS_ARRAY);
	__buffer.bind(DATA_ARRAY);
	
	glBindVertexArray(__buffer.vaoID);
	checkGLErrors(AT);
	
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), BUFFER_OFFSET(0)); // vertex
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), BUFFER_OFFSET(16)); // texture
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), BUFFER_OFFSET(24)); // normal
	checkGLErrors(AT);

	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(2);
	checkGLErrors(AT);
	
	glBindVertexArray(0);
}

void
Mesh::useMtl(Material *_mtl) {
	if (!__indices.empty()) {
		__materials[__materials.size() - 1].end = __indices.size() - __materials[__materials.size() - 1].begin;
		__materials.push_back(MeshRange(__indices.size()));
	}
	__materials[__materials.size() - 1].material = _mtl;
}

void
Mesh::smooth(bool _s) {
	__smooth = _s;
}

unsigned
Mesh::push_back(const Vertex &_v) {
	__vertices.push_back(_v);
	return __vertices.size() - 1;
}

void
Mesh::addNewIdx(int _idx) {
	__indices.push_back(_idx);
}

void
Mesh::addThreeIdxs(int _idx) {
	__indices.push_back(__indices[__indices.size() - 3]);
	__indices.push_back(__indices[__indices.size() - 2]);
	__indices.push_back(_idx);
}

void
Mesh::closeMesh(Material* _mat) {
	if ((__materials[__materials.size() - 1].material == NULL) && (_mat != NULL))
		__materials[__materials.size() - 1].material = _mat;
	__materials[__materials.size() - 1].end = __indices.size() - __materials[__materials.size() - 1].begin;
	
	loadIntoVbo();
}

void 
Mesh::__clearAllTriangles() {
  auto it = __triangles.begin();
  while(it != __triangles.end()) {
    auto eraseIter = it;
    ++it;
    delete eraseIter->second;
    __triangles.erase(eraseIter);
  }
}

