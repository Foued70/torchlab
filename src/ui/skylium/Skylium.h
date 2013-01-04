#ifndef SKYLIUM_H
#define SKYLIUM_H

#include "Singleton.h"
#include "SceneManager.h"
#include "TextureManager.h"
#include "Scene.h"
#include "Shader.h"
#include "MatricesManager.h"
#include "ShaderDataHandler.h"
#include "GPUMemory.h"

class Skylium : public Singleton < Skylium > {

public:
	
	/**
	 * Creates the singleton's instances and reads
	 * the config file.
	 */
	Skylium();
	
	virtual ~Skylium();
	
	/**
	 * Sets rendering context and does some standard jobs.
	 * @param windowName Window name;
	 * @return False if something went wrong.
	 */
	bool init();
	
	/**
	 * Catches events from the queue and renders activeScene.
	 */
	void execute();
	
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
	
	/* Access to managers from outside the class */
	SceneManager*& Scenes;
	TextureManager*& Textures;
	MatricesManager*& Matrices;
	ShaderDataHandler*& Shaders;
	GPUMemory*& VBOManagement;
	
	Shader* identityShader;
	Shader* shadingShader;
	Shader* texturedShadingShader;
	Shader* normalMapShader;
	
private:
	
	/**
	 * Renders the scene.
	 */
	void __render();
	
	/**
	 * @param fileName Name of the file.
	 * @return False if file could not be found, otherwise true.
	 */
	bool __fileExists(const std::string&);
	
	/* SceneManager instance */
	SceneManager * __sceneManagement;
	
	/* TextureManager instance */
	TextureManager * __textureManagement;
	
	/* MatricesManager instance */
	MatricesManager * __matricesManagement;
	
	/* ShaderDataHandler instance */
	ShaderDataHandler * __shaderDataHandling;
	
	/* GPUMemory instance */
	GPUMemory * __vboManagement;
	
	std::vector< Shader* > __shaderList;
	
	/* Vector with available extensions */
	std::vector< std::string* > __extensions;
	
};

#endif // SKYLIUM_H
