#ifndef ENGINE_H
#define ENGINE_H

#include "Singleton.h"
#include "TextureManager.h"
#include "Scene.h"
#include "Shader.h"
#include "MatricesManager.h"
#include "ShaderDataHandler.h"
#include "GPUMemory.h"
#include "TimeManager.h"
#include "FrameBuffer.h"
#include "ControllerManager.h"

#include <string>
#include <map>

class ScanWidget;
struct TriangleID;

enum RENDER_FLAGS {
  RENDER_TO_WINDOW = 1,
  RENDER_TO_FRAMEBUFFER = 2
};

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
    
  void render(const std::string& _sceneName, unsigned int _renderMode = (RENDER_TO_WINDOW | RENDER_TO_FRAMEBUFFER));
  
  void multiPassRender();
  
  /**
   * Creates a new frame buffer and deletes the old one. Necessary if the window is resized as the frame buffer is resolution dependant.
   */
  void regenerateFrameBuffer();
  
  /**
   * Ticks all dynamics / animations and exits once all simulations are complete(at a state of rest)
   */
  void simulateDynamics(ScanWidget* _scanWidget);
  
  /**
   * Returns the intersection point between a ray and polygon in the scene.
   * @param source The start point of the ray
   * @param direction the direction vector of the ray
   * @param outHitLocation position in world space of the first intersection.
   * @param return Weather or not the ray intersected a polygon
   */
  bool raycast(const Vector3& _source, const Vector3& _direction, Vector3& _outHitLocation);
  
  Scene* getScene(const string& _sceneName);
  
  Triangle* getTriangleByID(const TriangleID& _id);
	
	/* Access to managers from outside the class */
	TextureManager*& Textures;
	MatricesManager*& Matrices;
	ShaderDataHandler*& Shaders;
	GPUMemory*& VBOManagement;
  TimeManager*& Timer;
  FrameBuffer*& FBO;
  ControllerManager*& ControllerManagment;
	
	Shader* identityShader;
	Shader* shadingShader;
	Shader* texturedShadingShader;
	Shader* normalMapShader;
  Shader* wireframeShader;
  Shader* vertexHighlightShader;
	
private:
	
	/* TextureManager instance */
	TextureManager * __textureManagement;
	
	/* MatricesManager instance */
	MatricesManager * __matricesManagement;
	
	/* ShaderDataHandler instance */
	ShaderDataHandler * __shaderDataHandling;
	
	/* GPUMemory instance */
	GPUMemory * __vboManagement;
  
  /* TimeManager instance */
  TimeManager * __timeManagement;
  
  /* FrameBuffer instance */
  FrameBuffer * __frameBuffer;
  
  /* ControllerManager instance */
  ControllerManager * __controllerManagement;
  
	
	std::vector< Shader* > __shaderList;
	
 	/* All scenes */
 	std::map<string, Scene*> __sceneMap;
	
	/* Vector with available extensions */
	std::vector< std::string* > __extensions;
	
};

#endif // ENGINE_H
