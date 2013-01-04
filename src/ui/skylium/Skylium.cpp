#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
     
#include <sys/stat.h>
#include <unistd.h>

#include "Skylium.h"

#include "config.h"
#include "utils.h"

using namespace std;


Skylium::Skylium() : 
		Scenes(__sceneManagement),
		Textures(__textureManagement),
    // TheHud(__hud),
		Matrices(__matricesManagement),
		Shaders(__shaderDataHandling),
		VBOManagement(__vboManagement),
		identityShader(NULL),
		shadingShader(NULL),
		texturedShadingShader(NULL),
		normalMapShader(NULL),
		__sceneManagement(NULL),
		__textureManagement(NULL),
		__matricesManagement(NULL),
		__shaderDataHandling(NULL),
    __shaderList(0) {
			
	/* Create instances of singletons */
	__sceneManagement = new SceneManager();
	__textureManagement = new TextureManager();
	__matricesManagement = new MatricesManager();
	__shaderDataHandling = new ShaderDataHandler();
	__vboManagement = new GPUMemory();
	
	log(CONSTRUCTOR, "Skylium constructed.");
}

Skylium::~Skylium() {
	delete __textureManagement;
	delete __sceneManagement;
  // delete __hud;
	delete __matricesManagement;
	delete __shaderDataHandling;
	delete __vboManagement;
	
	while (!__shaderList.empty())
		delete __shaderList.back(), __shaderList.pop_back();
	
	while (!__extensions.empty())
		delete __extensions.back(), __extensions.pop_back();
	
	log(DESTRUCTOR, "Skylium destructed.");
}

bool
Skylium::init() {
	/* Construct and compile default shaders. */
	identityShader = createShader("identity");
	shadingShader = createShader("shadow");
	texturedShadingShader = createShader("textured");
	normalMapShader = createShader("normalmap");
	
	return true;
	
}

void
Skylium::execute() {
  // log(PARAM, "Skylium::execute");
	__render();
}

Scene *
Skylium::createScene(const string &_sceneName) {
	Scene *newScene = __sceneManagement -> createScene(_sceneName);
	return newScene;
}

Shader*
Skylium::createShader(const string& _fileName) {
	Shader* newShader = new Shader(_fileName);
	__shaderList.push_back(newShader);
	return newShader;
}

void
Skylium::__render() {
	__sceneManagement -> displayActiveScene();
}

bool
Skylium::__fileExists(const string &_fileName) {
	struct stat buf;
	if (stat(_fileName.c_str(), &buf) == 0)
		return 1;
	else
		return 0;
}
