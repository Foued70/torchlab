#ifndef SCENEMANAGER_H
#define SCENEMANAGER_H

#include <vector>

#include "Singleton.h"
#include "Scene.h"

class SceneManager : public Singleton< SceneManager > {
	
public:
	
	/**
	 * Default ctor - no scenes, no active scene.
	 */
	SceneManager();
	
	/**
	 * Deletes the __sceneList vector.
	 */
	virtual ~SceneManager();
	
	/**
	 * Creates new scene with the name given and return the pointer.
	 * If there's no active scene, this scene will be activated.
	 * @param name Name of the new scene.
	 * @return Pointer to the newly creates scene.
	 */
	Scene * createScene(const std::string&);
	
	/**
	 * Renders active scene.
	 */
	void displayActiveScene();
	
	/**
	 * Sets activeScene pointer to this given in argument.
	 * @param toSet Pointer to scene that has to be set as an active.
	 * @return False if something went wrong; see log.
	 */
	bool setActive(const Scene*);
	
	/**
	 * @return Pointer to an active scene.
	 */
	Scene * getActiveScene() { return __activeScene; }
	
private:
	
	void __setRenderingOptions();
	
	void __unsetRenderingOptions();
	
	/* All scenes */
	std::vector< Scene* > __sceneList;
	
	/* Pointer to the active scene */
	Scene * __activeScene;
	
};

#endif // SCENEMANAGER_H
