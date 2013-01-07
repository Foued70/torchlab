#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
     
#include <sys/stat.h>
#include <unistd.h>

#include "Engine.h"

#include "config.h"
#include "utils.h"

using namespace std;


Engine::Engine() : 
		Textures(__textureManagement),
      Matrices(__matricesManagement),
		Shaders(__shaderDataHandling),
		VBOManagement(__vboManagement),
		identityShader(NULL),
		shadingShader(NULL),
		texturedShadingShader(NULL),
		normalMapShader(NULL),
		__textureManagement(NULL),
		__matricesManagement(NULL),
		__shaderDataHandling(NULL),
    __shaderList(0),
    __sceneList(0) {
			
	/* Create instances of singletons */
	__textureManagement = new TextureManager();
	__matricesManagement = new MatricesManager();
	__shaderDataHandling = new ShaderDataHandler();
	__vboManagement = new GPUMemory();
	
	log(CONSTRUCTOR, "Engine constructed.");
}

Engine::~Engine() {
	delete __textureManagement;
	delete __matricesManagement;
	delete __shaderDataHandling;
	delete __vboManagement;
	
	while (!__shaderList.empty()) delete  __shaderList.back(), __shaderList.pop_back();
	while (!__extensions.empty()) delete __extensions.back(), __extensions.pop_back();
	while (!__sceneList.empty()) delete   __sceneList.back(),  __sceneList.pop_back();
  
	log(DESTRUCTOR, "Engine destructed.");
}

bool
Engine::init() {
	/* Construct and compile default shaders. */
	identityShader = createShader("identity");
	shadingShader = createShader("shadow");
	texturedShadingShader = createShader("textured");
	normalMapShader = createShader("normalmap");
	
	return true;
	
}

Scene *
Engine::createScene(const string &_sceneName) {
	Scene *newScene = new Scene(_sceneName);
	__sceneList.push_back(newScene);
	return newScene;
}

Shader*
Engine::createShader(const string& _fileName) {
	Shader* newShader = new Shader(_fileName);
	__shaderList.push_back(newShader);
	return newShader;
}


void Engine::render(Scene* scene) {
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	glClearColor(1.0, 1.0, 1.0, 1.0);
	checkGLErrors(AT);
	
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	checkGLErrors(AT);
	
  glEnable(GL_DEPTH_TEST);
  checkGLErrors(AT);
  
  // glEnable(GL_POLYGON_SMOOTH);
  // checkGLErrors(AT);
  // 
  //   glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
  // checkGLErrors(AT);
  //   glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
  //   checkGLErrors(AT);
  
  glEnable(GL_MULTISAMPLE_ARB);
  checkGLErrors(AT);
	
  scene -> show();
  
	glDisable(GL_BLEND);
	glDisable(GL_DEPTH_TEST);
	
	checkGLErrors(AT);
}

