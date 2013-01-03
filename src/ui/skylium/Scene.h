/*
     ___                    _    
    / __| __ ___ _ _  ___  | |_  
    \__ \/ _/ -_) ' \/ -_)_| ' \ 
    |___/\__\___|_||_\___(_)_||_|
                  
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


#ifndef SCENE_H
#define SCENE_H

#include <vector>

#include "Object.h"
#include "Camera.h"
#include "Light.h"
#include "Vectors.h"

class ShaderDataHandler;

class Scene {
public:
	
	/**
	 * Default ctor.
	 * Empty vectors, empty pointers.
	 * @param name Name of the new scene.
	 */
	Scene(const std::string& = "");
	
	/**
	 * Destroys objects, cams and lights.
	 */
	virtual ~Scene();
	
	/**
	 * Renders the scene.
	 * Active camera is used.
	 */
	void show();
	
//============ OBJECTS ============//
	
	/**
	 * Creates new object with the name given.
	 * @param name Name of the new object.
	 * @param parent Pointer to the object, which has to be parent (default NULL).
	 * @return Pointer to the newly created object.
	 */
	Object * createObject(const std::string&, Object* = (Object*)NULL);
	
	/**
	 * Look for the object.
	 * @param name Name of the object that is looked for.
	 * @return Pointer to the found object, or NULL if nothing was found.
	 */
	Object * getObjectByName(const std::string&);
	
//============ CAMERAS ============//
	
	/**
	 * Creates the new camera. If this is the first camera, it is set as the main.
	 * @param x Camera's position X coord;
	 * @param y Y coord;
	 * @param z Z coord.
	 * @return Pointer to the newly created camera.
	 */
	Camera * createCamera(GLfloat, GLfloat, GLfloat, const cType& = FPP);
	
	/**
	 * Sets the active camera pointer to the camera given as an argument.
	 * @param camera Pointer to the camera that has to be set as an active.
	 * @param checking If true, looks if the pointer exists in the vector. To disable, 
	 * 		and improve the speed (in case of camera switching really often), set this false.
	 * @return False if checking = true and camera was not found in the cameras vector.
	 */
	bool setActiveCamera(Camera*, bool = true);
	
	/**
	 * @return Pointer to the active camera.
	 */
	Camera * getActiveCamera() { return __activeCamera; }
	
//============ LIGHTS ============//
	
	/**
	 * Creates the new light. Parameters define the light source position, (0,0,0) by default.
	 * Light are indexed - as OpenGL defines them - from 0 to 7.
	 * This function looks for the first index available. If we have, for example, lights 1, 2 and 3,
	 * and then we call removeLight(1) and then we create the new light, the function will return 1
	 * as the new light's index.
	 * @param x X coord of the light's source;
	 * @param y Y coord;
	 * @param z Z coord.
	 * @return Index of the new light.
	 */
	Light * createLight(GLfloat = 0, GLfloat = 0, GLfloat = 0);
	
	/**
	 * Removes light from the scene.
	 * @param id Light's ID. By default it is the last known light.
	 */
	void removeLight(Light*);
	
	/* Scene's name */
	std::string name;
	
private:
	/* Shows all objects */
	void __setObjects();
	
	/* Sets all lights on the scene. 
	 * http://www.opengl.org/sdk/docs/man/xhtml/glLight.xml */
	void __setLights();
	
	/* Ends rendering the frame, sets wasShown = false
	 * for all objects */
	void __endFrame();
	
	/* Vector of objects */
	std::vector< Object* > __objectList;
	
	/* Objects' iterator */
	std::vector< Object* >::const_iterator __objectIterator;
	
	/* Cameras' iterator */
	std::vector< Camera* > __cameraList;
	
	/* Pointer to the active camera */
	Camera * __activeCamera;
	
	/* Lights' vector */
	std::vector< Light* > __lightList;
	
	/* Light's iterator.
	 * Patrz metoda __setLights(). */
	std::vector< Light* >::const_iterator __lightIterator;
	
	sColor __lightModelAmbient;
	
	ShaderDataHandler& __shaders;

};

#endif // SCENE_H
