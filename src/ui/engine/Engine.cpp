#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
     
#include <sys/stat.h>
#include <unistd.h>

#include <QtOpenGL/QGLWidget>
#include <QtOpenGL/QGLBuffer>
#include <QtOpenGL/QGLShaderProgram>

#include "Engine.h"
#include "utils.h"
#include "../ScanWidget.h"

using namespace std;


Engine::Engine() : 
		Textures(__textureManagement),
    Matrices(__matricesManagement),
		Shaders(__shaderDataHandling),
		VBOManagement(__vboManagement),
    Timer(__timeManagement),
    FBO(__frameBuffer),
    ControllerManagment(__controllerManagement),
		identityShader(NULL),
		shadingShader(NULL),
		texturedShadingShader(NULL),
		normalMapShader(NULL),
		__textureManagement(NULL),
		__matricesManagement(NULL),
		__shaderDataHandling(NULL),
    __timeManagement(NULL),
    __frameBuffer(NULL),
    __controllerManagement(NULL)
{
			
	/* Create instances of singletons */
	__textureManagement = new TextureManager();
	__matricesManagement = new MatricesManager();
	__shaderDataHandling = new ShaderDataHandler();
	__vboManagement = new GPUMemory();
  __timeManagement = new TimeManager();
  //__frameBuffer = new FrameBuffer();
  __controllerManagement = new ControllerManager();
	
	log(CONSTRUCTOR, "Engine constructed.");
}

Engine::~Engine() {
	delete __textureManagement;
	delete __matricesManagement;
	delete __shaderDataHandling;
	delete __vboManagement;
  delete __timeManagement;
  delete __frameBuffer;
  delete __controllerManagement;
	
	while (!__shaderList.empty()) delete  __shaderList.back(), __shaderList.pop_back();
	while (!__extensions.empty()) delete __extensions.back(), __extensions.pop_back();
  
  auto it = __sceneMap.begin();
  while (it != __sceneMap.end()) {
    delete it->second;
  }
  __sceneMap.clear();
  
	log(DESTRUCTOR, "Engine destructed.");
}

bool
Engine::init() {
	/* Construct and compile default shaders. */
	identityShader = createShader("identity");
	shadingShader = createShader("shadow");
	texturedShadingShader = createShader("textured");
	normalMapShader = createShader("normalmap");
  wireframeShader = createShader("wireframe");
  vertexHighlightShader = createShader("vertexHighlight");
  	
	return true;	
}

Scene *
Engine::createScene(const string &_sceneName) {
  Scene* scene;
  auto it = __sceneMap.find(_sceneName);
  if ( it == __sceneMap.end() ) {
    scene = new Scene(_sceneName);
    __sceneMap.insert( make_pair(_sceneName, scene) );
    return scene;
  }

  log(WARN, "Scene %s already exists! No scene created.", _sceneName.c_str());
  return NULL;
}

Scene* 
Engine::getScene(const string& _sceneName) {
  auto it = __sceneMap.find(_sceneName);
  if (it == __sceneMap.end()) {
    log(WARN, "Scene %s does not exist!", _sceneName.c_str());
    return NULL;
  }
  return it->second;
}

Shader*
Engine::createShader(const string& _fileName) {
	Shader* newShader = new Shader(_fileName);
	__shaderList.push_back(newShader);
	return newShader;
}

void 
Engine::render(const std::string& _sceneName, unsigned int _renderMode) {
  //Render scene to our framebuffer
  Scene* currentScene = getScene(_sceneName);
  if (!currentScene) {
    log(WARN, "Cannot render scene. No scene %s found.", _sceneName.c_str());
    return;
  }
  
  if (_renderMode & RENDER_TO_FRAMEBUFFER) {
    __frameBuffer -> renderToTexture();
  }
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	glClearColor(0.0, 0.0, 0.0, 0.0);
	checkGLErrors(AT);
	
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	checkGLErrors(AT);
	
  glEnable(GL_DEPTH_TEST);
  checkGLErrors(AT);
  
  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);
  checkGLErrors(AT);
  
  // glEnable(GL_POLYGON_SMOOTH);
  // checkGLErrors(AT);
  // 
  //   glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
  // checkGLErrors(AT);
  //   glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
  //   checkGLErrors(AT);
  
  glEnable(GL_MULTISAMPLE);
  checkGLErrors(AT);
	
  currentScene -> show();
  
	glDisable(GL_BLEND);
	glDisable(GL_DEPTH_TEST);
	
	checkGLErrors(AT);
  
  if (_renderMode & RENDER_TO_FRAMEBUFFER) {
    __frameBuffer -> unbind();
  }
  //display rendered color data to screen
  if ((_renderMode & RENDER_TO_FRAMEBUFFER) &&
      (_renderMode & RENDER_TO_WINDOW)) {
    __frameBuffer -> displayToWindow();
  }
  checkGLErrors(AT);
}

void 
Engine::regenerateFrameBuffer() {
  if( __frameBuffer != NULL ) {
    delete __frameBuffer;
  }
  __frameBuffer = new FrameBuffer();
  __frameBuffer -> initialize();
}

void 
Engine::simulateDynamics(ScanWidget* _scanWidget) {
  while(__controllerManagement->needsUpdate()) {
    __timeManagement->tick();
    __controllerManagement->update();
    _scanWidget->refresh();
    log(PARAM, "End of simulation tick.");
  }
  log(PARAM, "Exiting simulation. All dynamics idle.");
}

bool 
Engine::raycast(const Vector3& _source, const Vector3& _direction, Vector3& _outHitLocation) {
  Vector3 currentEye;
  Vector3 currentCenter;
  Scene* currentScene = getScene("MainScene");
  if(!currentScene) {
    log(WARN, "Cannot raycast. No scene set.");
    return false;
  }
  Camera* camera = currentScene->getActiveCamera();
  currentEye = camera->getEye();
  currentCenter = camera->getCenter();
  camera->setEyePosition(_source);
  camera->setCenterPosition(_direction);
  render("MainScene", RENDER_TO_FRAMEBUFFER);
  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);   
  float centerX = viewport[2] * 0.5f;
  float centerY = viewport[3] * 0.5f;
  _outHitLocation = camera->cameraToWorld(centerX, centerY);
  
  //Set the camera back to it's normal position, and do a render to refresh all frame buffer data to correct values.
  camera->setEyePosition(currentEye);
  camera->setCenterPosition(currentCenter);
  render("MainScene", RENDER_TO_WINDOW | RENDER_TO_FRAMEBUFFER);
  
  Vector3 rayOffset = _outHitLocation - _source;
  if(rayOffset.magnitude() >= camera->getFarPlane()) {
    return false;
  }
  return true;
}

Triangle* 
Engine::getTriangleByID(const TriangleID& _id) {
  Triangle* triangle =  getScene("MainScene")->
                        getObjectByID(_id.objectID)->
                        getMeshByID(_id.meshID)->
                        getTriangleByID(_id.primitiveID);
  
  triangle->ID = _id; 
  return triangle;
}

Vertex* 
Engine::pickVertex(float _x, float _y, unsigned int _searchDistance, float _foundThreshold) {
  Vertex* selectedVertex = NULL;
      
  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);
  float windowWidth = viewport[2];
	float windowHeight = viewport[3];
      
  std::vector<Triangle*> foundTriangles;
      
  for ( unsigned int x = (((int)_x-_searchDistance) >= 0) ? ((int)_x-_searchDistance) : 0; 
        x < ((int)_x+_searchDistance) && x < windowWidth;
        x++ )
  {
    for ( unsigned int y = (((int)_y-_searchDistance) >= 0) ? ((int)_y-_searchDistance) : 0;
          y < ((int)_y+_searchDistance) && y < windowHeight;
          y++ )
    {
      TriangleID pickingData = __frameBuffer->pickTriangle((GLuint)x, (GLuint)y);
      if (pickingData.objectID != 0) {
        Triangle* tempTriangle = getTriangleByID(pickingData);
        bool triangleExists = false;
        for (unsigned int t = 0; t < foundTriangles.size(); t++) {
          if (foundTriangles[t] == tempTriangle) {
            triangleExists = true;
            break;
          }
        }
        if (!triangleExists) {
          foundTriangles.push_back(tempTriangle); 
        }
      } 
    }
  }
  //Find the closest vertex to click
  Vector3 clickPosition = getScene("MainScene")->getActiveCamera()->cameraToWorld(_x, _y);
  for (unsigned int t = 0; t < foundTriangles.size(); t++) {
    for (unsigned int v = 0; v < 3; v++) {
      Vertex* tempVertex = foundTriangles[t]->vertices[v];
                                              
      if (t == 0 && v == 0) {
        selectedVertex = tempVertex;
        continue;
      }
      if (  distanceSquared(clickPosition, Vector3({tempVertex->vertexPosition.x, tempVertex->vertexPosition.y, tempVertex->vertexPosition.z})) 
            < distanceSquared(clickPosition, Vector3({selectedVertex->vertexPosition.x, selectedVertex->vertexPosition.y, selectedVertex->vertexPosition.z})) ) {
        selectedVertex = tempVertex;
      }                          
    }
  }
  foundTriangles.clear();
  if ( distanceSquared( clickPosition, 
                        Vector3({selectedVertex->vertexPosition.x, selectedVertex->vertexPosition.y, selectedVertex->vertexPosition.z}))
        < (_foundThreshold*_foundThreshold))
  {
    log(PARAM, "Vertex Selected. Distance from mouse: %f", distance(clickPosition, Vector3{selectedVertex->vertexPosition.x, selectedVertex->vertexPosition.y, selectedVertex->vertexPosition.z}));
    return selectedVertex;
  }
  log(PARAM, "No vertex found.");
  return NULL;
}

