/*
     ___                  __  __                              _    
    / __| __ ___ _ _  ___|  \/  |__ _ _ _  __ _ __ _ ___ _ _ | |_  
    \__ \/ _/ -_) ' \/ -_) |\/| / _` | ' \/ _` / _` / -_) '_|| ' \ 
    |___/\__\___|_||_\___|_|  |_\__,_|_||_\__,_\__, \___|_|(_)_||_|
                                               |___/    

    Copyright (C) 2011  Michał Garapich garrappachc@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


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
