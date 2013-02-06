#ifndef MESH_H
#define MESH_H

#include <vector>

#include "Vertex.h"
#include "Material.h"
#include "GPUMemory.h"
#include "BufferObject.h"

struct MeshRange {
	
	size_t		begin;
	size_t		end;
	Material*		material;
	
	MeshRange() : begin(0), end(0), material(NULL) {}
	MeshRange(size_t _b) : begin(_b), end(_b), material(NULL) {}
	
};

typedef GLuint IndicesType;

class Mesh {
	
public:
	
	/**
	 * Default ctor, gets only name.
	 * @param name Mesh's name.
	 */
	Mesh(const std::string& = "");
	
	/**
	 * Empties the VBO (if used), some output.
	 */
	virtual ~Mesh();
	
	/**
	 * Renders the mesh.
	 */
	void show();
	
	/**
	 * Sends all of the vertices to GPU's buffer.
	 */
	void loadIntoVbo();
	
	/**
	 * Sets active material to this given.
	 */
	void useMtl(Material*);
	void useMtl(Material*, size_t begin, size_t end);
	
	/**
	 * If true, the smooth shading is set on.
	 */
	void smooth(bool = true);
	
	/**
	 * Puts into the vector the next Vertex and returns its index.
	 * @param v Vertex to be put.
	 * @return The new vertex's index.
	 */
	unsigned push_back(const Vertex&);
	
	/**
	 * Puts the new index to the vector.
	 */
	void addNewIdx(int);
	
	/**
	 * This is used to add these faces, which have 4 indices - it adds
	 * the first, third and the last indices.
	 */
	void addThreeIdxs(int);
	
	/**
	 * Indicates that the mesh creating has just finished.
	 */
	void closeMesh(Material*);
	
	bool empty() { return __vertices.empty() && __indices.empty(); }
	
	bool hasAnyMaterials() { return __materials.size() > 1; }
	
	void rendering(bool _s = true) { __isShown = _s; }
	
	std::string name;
	
private:
	
	BufferObject __buffer;
	
	/* Vertices' vector */
	std::vector< Vertex > __vertices;
	
	/* Vertices' index */
	std::vector< IndicesType > __indices;
	
	/* Materials that will be used */
	std::vector < MeshRange > __materials;
	
	/* If true, then glShadeModel(GL_SMOOTH) */
	bool __smooth;
	
	/* http://www.opengl.org/sdk/docs/man/xhtml/glBufferData.xml */
	vboUsage __usage;
	
	/* http://www.opengl.org/sdk/docs/man/xhtml/glDrawElements.xml */
	GLenum __mode;
	
	bool __isShown;

};

#endif // MESH_H
