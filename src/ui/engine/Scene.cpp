#include <vector>

#include "Scene.h"
#include "ShaderDataHandler.h"
#include "utils.h"

using namespace std;

Scene::Scene(const string &_name) :
		name(_name),
		__objectList(0),
		__objectIterator(),
		__cameraList(0),
		__activeCamera(NULL),
		__lightList(0),
		__lightIterator(),
		__lightModelAmbient({0.2, 0.2, 0.2, 1.0}),
		__shaders(ShaderDataHandler::GetSingleton()) {
	log(CONSTRUCTOR, "Scene (\"%s\") constructed.", name.c_str());
}

Scene::~Scene() {
	while (!__objectList.empty())
		delete __objectList.back(), __objectList.pop_back();
  
	while (!__cameraList.empty())
		delete __cameraList.back(), __cameraList.pop_back();
	while (!__lightList.empty())
		delete __lightList.back(), __lightList.pop_back();
	
	log(DESTRUCTOR, "Scene (\"%s\") destructed.", name.c_str());
}

void
Scene::show() {
  // log(PARAM, "Scene::show");
	if (__activeCamera) {
		__activeCamera -> setView();
  }
	
	__setLights();
	
	__shaders.updateData("sLightModel.ambient", __lightModelAmbient);
	
	__setObjects();
	
	__endFrame();
}

Object *
Scene::createObject(const string &_name, Object *_parent) {
  if ( getObjectByName(_name) ) {
    log(WARN, "Object %s already exists! Cannot create a new object of the same name.", _name.c_str());
    return NULL;
  }
	Object *newObject;
	newObject = new Object(_name);
  
	if (_parent != NULL)
		_parent -> addChild(newObject);
	
	__objectList.push_back(newObject);
	log(PARAM, "Gonna return new object");
	return newObject;
}

bool 
Scene::deleteObject(const std::string& _name) {
  auto it = __objectList.begin();
  while ( it != __objectList.end() ) {
     if( (*it)->name == _name ) {
       delete *it;
       it = __objectList.erase(it);
       return true;
     }
     else {
        ++it;
     }
  }
  return false;
}

Object *
Scene::getObjectByName(const string &_name) {
  for ( int object = 0; object < __objectList.size(); object++ ) {
    if (__objectList[object]->name == _name) {
      return __objectList[object];
    }
  }
  return NULL;
}

Object * 
Scene::getObjectByID(unsigned int _id) {
  for ( int object = 0; object < __objectList.size(); object++ ) {
    if (__objectList[object]->getID() == _id) {
      return __objectList[object];
    }
  }
  return NULL;
}

Camera *
Scene::createCamera(GLfloat _x, GLfloat _y, GLfloat _z) {
	Camera *newCamera = new Camera(_x, _y, _z);
	if (!__activeCamera) {
		__activeCamera = newCamera;
		__activeCamera -> setProjection(); // sets "seeing" parameters
	}
	__cameraList.push_back(newCamera);
	return newCamera;
}

bool
Scene::setActiveCamera(Camera *_camera, bool _checking) {
	if (_checking) {
		for (unsigned i = 0; i < __cameraList.size(); i++) {
			if (__cameraList[i] == _camera) {
				__activeCamera = _camera;
				return true;
			}
		}
		return false;
	} else {
		__activeCamera = _camera;
		return true;
	}
}

Light *
Scene::createLight(GLfloat _x, GLfloat _y, GLfloat _z) {
	Light* newLight = new Light(Vector3({_x, _y, _z}));
	__lightList.push_back(newLight);
	return newLight;
}

void
Scene::removeLight(Light* _light) {
	for (auto it = __lightList.begin(); it != __lightList.end(); ++it) {
		if ((*it) == _light) {
			__lightList.erase(it);
			delete (*it);
			return;
		}
	}
}

void
Scene::__setObjects() {
	if (__objectList.empty())
		return;
	
	__objectIterator = __objectList.begin();
	while(__objectIterator != __objectList.end()) {
		// log(PARAM, "Scene::__setObjects %s", (*__objectIterator)->name.c_str());
		if (!(*__objectIterator) -> wasShown())
			(*__objectIterator) -> show();
		++__objectIterator;
	}
}

void
Scene::__setLights() {
	unsigned i = 0;
	for (Light*& l: __lightList) {
		l -> makeLight(i);
		++i;
	}
	/*
	 * TODO: Tell shader how many lights we have.
	 */
}

void
Scene::__endFrame() {
	__objectIterator = __objectList.begin();
	while (__objectIterator != __objectList.end())
		(*__objectIterator) -> endFrame(), __objectIterator++;
}