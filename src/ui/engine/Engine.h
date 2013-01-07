#ifndef ENGINE_H
#define ENGINE_H

#include "Singleton.h"
#include "TextureManager.h"
#include "Scene.h"
#include "Shader.h"
#include "MatricesManager.h"
#include "ShaderDataHandler.h"
#include "GPUMemory.h"

class Engine : public Singleton < Engine > {

public:
	
	/**
	 * Creates the singleton's instances and reads
	 * the config file.
	 */
	Engine();
	
	virtual ~Engine();
	
	/**
	 * Sets rendering context and does some standard jobs.
	 * @param windowName Window name;
	 * @return False if something went wrong.
	 */
	bool init();
	
	/**
	 * Creates the new scene.
	 * @param name Name of the new scene - must be unique!
	 * @return Pointer to the newly created scene.
	 */
	Scene * createScene(const std::string&);
	
	/**
	 * Creates the new shader.
	 * @param sourceFiles Source file of both - vertex and fragment shaders (without extensions).
	 */
	Shader * createShader(const std::string&);
   
  void render(Scene* scene);
	
	/* Access to managers from outside the class */
	TextureManager*& Textures;
	MatricesManager*& Matrices;
	ShaderDataHandler*& Shaders;
	GPUMemory*& VBOManagement;
	
	Shader* identityShader;
	Shader* shadingShader;
	Shader* texturedShadingShader;
	Shader* normalMapShader;
	
private:
	
	/* TextureManager instance */
	TextureManager * __textureManagement;
	
	/* MatricesManager instance */
	MatricesManager * __matricesManagement;
	
	/* ShaderDataHandler instance */
	ShaderDataHandler * __shaderDataHandling;
	
	/* GPUMemory instance */
	GPUMemory * __vboManagement;
	
	std::vector< Shader* > __shaderList;
	
 	/* All scenes */
 	std::vector< Scene* > __sceneList;
	
	/* Vector with available extensions */
	std::vector< std::string* > __extensions;
	
};

#endif // ENGINE_H
