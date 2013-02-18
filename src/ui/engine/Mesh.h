#ifndef MESH_H
#define MESH_H

#include <vector>
#include <map>

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

struct TriangleID {
  unsigned int objectID;
  unsigned int meshID;
  unsigned int primitiveID;
  
  TriangleID() {
    objectID = 0;
    meshID = 0;
    primitiveID = 0;
  }
  TriangleID( unsigned int _objectID,
              unsigned int _meshID,
              unsigned int _primitiveID ) 
  {
    objectID = _objectID;
    meshID = _meshID;
    primitiveID = _primitiveID;
  }
};

struct Triangle {
  TriangleID ID;
  Vertex* vertices[3];
  
  Triangle() {
    ID.objectID = 0;
    ID.meshID = 0;
    ID.primitiveID = 0;
    vertices[0] = NULL;
    vertices[1] = NULL;
    vertices[2] = NULL;
  }
  Triangle(Vertex* _v1, Vertex* _v2, Vertex* _v3) {
    ID.objectID = 0;
    ID.meshID = 0;
    ID.primitiveID = 0;
    vertices[0] = _v1;
    vertices[1] = _v2;
    vertices[2] = _v3;
  }
  Triangle(const TriangleID& _ID, Vertex* _v1, Vertex* _v2, Vertex* _v3) {
    ID = _ID;
    vertices[0] = _v1;
    vertices[1] = _v2;
    vertices[2] = _v3;
  }
};

typedef GLuint IndicesType;
typedef std::map< unsigned int, Triangle* > triangleMap;

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
	 * Look for the triangle.
	 * @param id ID of the triangle that is looked for. 
   * ID's for triangles are local to the mesh that owns them. To get a unique triangle, you need to know the object's ID, the meshes's ID, as well as the triangle ID
	 * @return Pointer to the found Triangle, or NULL if nothing was found.
	 */ 
  Triangle* getTriangleByID(unsigned int);
	
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
	void useMtl(Material*, size_t begin, size_t length);
	
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
  
  void logMeshData() { log(PARAM, "__vertices.size()=%d __indices.size()=%d __materials.size()=%d", __vertices.size(), __indices.size(), __materials.size());}
  
private:
  void __clearAllTriangles();
	
private:
	
	BufferObject __buffer;
	
	/* Vertices' vector */
	std::vector< Vertex > __vertices;
	
	/* Vertices' index */
	std::vector< IndicesType > __indices;
	
	/* Materials that will be used */
	std::vector < MeshRange > __materials;
  
  triangleMap __triangles;
	
	/* If true, then glShadeModel(GL_SMOOTH) */
	bool __smooth;
	
	/* http://www.opengl.org/sdk/docs/man/xhtml/glBufferData.xml */
	vboUsage __usage;
	
	/* http://www.opengl.org/sdk/docs/man/xhtml/glDrawElements.xml */
	GLenum __mode;
	
	bool __isShown;

};

#endif // MESH_H
