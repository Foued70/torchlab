#include "opengl.h"
#include "SceneManager.h"

#include "Skylium.h"

#include "config.h"
#include "utils.h"

using namespace std;

SceneManager::SceneManager() :
		__sceneList(0),
		__activeScene(NULL) {
	log(CONSTRUCTOR, "SceneManager constructed.");
}

SceneManager::~SceneManager() {
	while (!__sceneList.empty())
		delete __sceneList.back(), __sceneList.pop_back();
	
	log(DESTRUCTOR, "SceneManager destructed.");
}

Scene *
SceneManager::createScene(const string &_name) {
	Scene *newScene = new Scene(_name);
	__sceneList.push_back(newScene);
	if (!__activeScene)
		__activeScene = newScene;
	return newScene;
}

void
SceneManager::displayActiveScene() {
  // log(PARAM, "SceneManager::displayActiveScene");
	
	__setRenderingOptions();
	
	if (__activeScene) {
		__activeScene -> show();	
	}
	
	__unsetRenderingOptions();
}

bool
SceneManager::setActive(const Scene *_toSet) {
	if (__sceneList.size() <= 0) {
		log(WARN, "There are no scenes! Active scene was NOT set.");
		return false;
	}
	
	for (unsigned i = 0; i < __sceneList.size(); i++) {
		if (__sceneList[i] == _toSet) {
			__activeScene = __sceneList[i];
			return true;
		}
	}
	log(WARN, "There is no such scene in the vector! Remember to create new scenes via SceneManager::createScene().");
	return false;
}

void
SceneManager::__setRenderingOptions() {
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
	
}

void
SceneManager::__unsetRenderingOptions() {
	glDisable(GL_BLEND);
	glDisable(GL_DEPTH_TEST);
	
	checkGLErrors(AT);
}