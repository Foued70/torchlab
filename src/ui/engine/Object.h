#ifndef OBJECT_H
#define OBJECT_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <map>

#include "Vectors.h"
#include "Material.h"
#include "Mesh.h"
#include "LuaObject.h"

class Shader;
class MatricesManager;
class ShaderDataHandler;
struct Position;
struct TexCoords;
struct Normal;
struct Index;
struct HashMyIndex;

enum {
	TEXTURE = 1,
	NORMALS = 2,
	NORMAL_MAP = 4,
	SPECULAR_MAP = 8
};

enum {
	INVERT_X = 1,
	INVERT_Y = 2
};

class Object;

typedef std::unordered_map< Index, long, HashMyIndex > indicesMap;
typedef std::map< std::string, Mesh* > meshesMap;
typedef std::unordered_map< std::string, Material* > materialsMap;
typedef std::vector< GLfloat > s3DVector;

class Object {
	
	friend class Shader;

public:
	
	/**
	 * Default ctor, gets the name.
	 */
	Object(const std::string& = "");
	
	/**
	 * Destroys children, meshes and materials.
	 */
	virtual ~Object();
	
	/**
	 * Transforms and renders.
	 */
	virtual void show();
	
	/**
	 * Moves the object.
	 * @param x factor;
	 * @param y factor;
	 * @param z factor.
	 */
	void move(GLfloat, GLfloat, GLfloat);
  
	/**
	 * Moves the object to an absolute position in world space.
	 * @param x factor;
	 * @param y factor;
	 * @param z factor.
	 */ 
  void setPosition(GLfloat, GLfloat, GLfloat);
	
	/**
	 * Scales the object.
	 * @param x;
	 * @param y;
	 * @param z.
	 */
	void scale(GLfloat, GLfloat, GLfloat);
	
	/**
	 * Rotates the object.
	 * @param rotX X angle;
	 * @param rotY Y angle;
	 * @param rotZ Z angle.
	 */
	void rotate(GLfloat, GLfloat, GLfloat);
	
	/**
	 * Sets the object general colour.
	 * @param R Red;
	 * @param G Green;
	 * @param B Blue;
	 * @param A Alpha.
	 * @return False if one of the values given is not between [0; 1].
	 */
	bool setColor(GLfloat, GLfloat, GLfloat, GLfloat = 1.0);
	
	/**
	 * Sets the object general colour.
	 * @param R Red;
	 * @param G Green;
	 * @param B Blue
	 * @param A Alpha.
	 * @return False if one of the values given is not between [0; 255] or alpha is not between [0; 1].
	 */
	bool setColor(int, int, int, GLfloat = 1.0);
	
	/**
	 * Loads the object form .obj and .mtl.
	 * @param objFile .obj file localization.
	 * @return False if something went wrong.
	 */
	bool loadFromObj(const std::string&, unsigned = 0);
	
	bool loadFrom(LuaObject* obj);
	
	/**
	 * Loads the whole object into the VBO.
	 */
	virtual void loadIntoVBO();
	
	/**
	 * Gives us the particular mesh.
	 * @param name Mesh's name.
	 * @return Mesh.
	 */
	inline Mesh * getMeshByName(const std::string& _name) { return __meshes[_name]; }
   
	/**
	 * Look for the mesh.
	 * @param id ID of the mesh that is looked for. 
   * ID's for meshes are local to the object that owns them. To get a unique mesh, you need to know the object's ID, as well as the mesh ID
	 * @return Pointer to the found Mesh, or NULL if nothing was found.
	 */ 
  Mesh* getMeshByID(unsigned int);
  
	
	inline Material * getMaterialByName(const std::string& _name) { return __materials[_name]; }
	
	/**
	 * Adds the pointer to the children's vector.
	 * @param childPtr Child's pointer.
	 */
	void addChild(Object*);
	
	/**
	 * @return True, if the object was already rendered.
	 */
	bool wasShown() { return __wasShown; }
	
	/**
	 * Called by the Scene at the end of each frame.
	 */
	void endFrame() { __wasShown = false; }
	
	/**
	 * @return True if object has any texture.
	 */
	bool isTextured() { return (__content & TEXTURE); }
	
	std::string name;
  
  void select();
  
  /* A method that returns the object's id */
  unsigned int getID();
	
protected:
	
	/* Genera colour of the object.
	 * Does not have to be set, if texture enabled. */
	sColor __defColor;
	
	/* Vectors that the ModelView Matrix will be
	 * multiplied by in MatricesManager::translate, ::rotate
	 * and ::scale. */
	Vector3 __mov;
	Vector3 __rot;
	Vector3 __scale;
	
	/* A pointer to object's shader */
	Shader * __shader;
	
	/* To indicate if the object was already rendered in the current frame. */
	bool __wasShown;
	
private:
	
	/**
	 * Parses the .obj file and put results in current
	 * object's instance.
	 * @param fileName Name of the .obj file.
	 */
	void __parseObj(const std::string&, unsigned);
	
	/**
	 * Helpful function to shorten a bit parsing the obj file.
	 * It just parses the one face's line.
	 */
	void __parseFace(std::istringstream&, Mesh*&,
			std::vector< Position >&, std::vector< TexCoords >&,
			std::vector< Normal >&, indicesMap&,
			unsigned, unsigned, unsigned,
			long&, unsigned);
	
	/**
	 * Parses the .mtl file.
	 */
	void __parseMtl(const std::string&);
	
	/**
	 * @param fileName Name of the file to be checked.
	 * @return True, if the file exists.
	 */
	bool __fileExists(const std::string&);
	
	/**
	 * According to what object has in __content,
	 * binds the appropriate shader. Default shaders'
	 * instances can be found in global instance.
	 */
	void __bindAppropriateShader();
  
  /* A simple method to hand out unique ids to objects on creation */
  unsigned int __getNewID();

  /* Object's unique id */
  unsigned int __id;
  
  static unsigned int nextAvailableID;
  
  
	/* Vector of children's pointers. */
	std::vector< Object* > __children;
	
	meshesMap __meshes;
	
	materialsMap __materials;
	
	short unsigned __content;
	
	MatricesManager& __matrices;
	
	ShaderDataHandler& __shaders;
	
};

#endif // OBJECT_H