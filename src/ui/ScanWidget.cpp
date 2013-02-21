#include "Core3_2_context.h"
#include "ScanWidget.h"
#include "utils.h"
#include "engine/Engine.h"
#include "engine/FrameBuffer.h"
#include "engine/CameraController.h"
#include "engine/Surface.h"
#include "engine/Texture.h"

#include <QtGui/QMouseEvent>
#include <QDebug>
#include <QKeyEvent>
#include <QFile>
#include <QString>

#include <iostream>
#include <string>
#include <QAbstractEventDispatcher>

ScanWidget::ScanWidget(const QGLFormat& format, QWidget* parent ) : QGLWidget( new Core3_2_context(format), parent ),
    vertexBuffer( QGLBuffer::VertexBuffer ),
    polyBuffer(QGLBuffer::VertexBuffer),
    clickTimerRMB_Start(-1.0),
    clickTimerRMB_Current(0.0),
    rotateMode(true),
    __vertexPointMode(false),
    __picturePointMode(false),
    __modelFile(""),
    __photoFile(""),
    __outputPath(""),
    __outputIndex(1)
{
  setMouseTracking(false);
  engine = Engine::GetSingletonPtr();
}

ScanWidget::~ScanWidget() {
}

void 
ScanWidget::refresh() {
  QAbstractEventDispatcher::instance(0)->processEvents(QEventLoop::AllEvents);
  updateGL();
}

void 
ScanWidget::initializeGL() {
  printf("OpenGL %s GLSL %s\n", glGetString(GL_VERSION), glGetString(GL_SHADING_LANGUAGE_VERSION));
  
  setupSceneFromFile("config/poseAlignConfig.txt");
  
  engine->init();
  mainScene = engine->createScene("MainScene");
  scene = mainScene;
  postScene = engine->createScene("PostScene");
  
  mainModel = mainScene->createObject("mainModel");
  if (!mainModel -> loadFromObj(__modelFile.c_str())) exit(1);
  
  Camera *sphereCamera = mainScene->createCamera(-20, 0, 7);
  sphereCamera -> lookAt(0, 0, 7);
  
  sphereCameraController = ControllerManager::GetSingleton().createCameraController(sphereCamera);
  sphereCameraController->setZoom(10.0f);
        
  /* Light on (7, 3, 0) position */
  Light* light = mainScene->createLight(5, 15, -10);
  /* Set ambient light */
  light -> setAmbient(sColor( {0.8, 0.8, 0.8, 1.0} ));
  
  TimeManager& timer = TimeManager::GetSingleton();
  TimeManager::logTime();
  timer.logDeltaTime();
}

void ScanWidget::resizeGL(int w, int h) {
  // printf("glViewport(%d, %d)\n", w, h);
  
  //Lock width to aspect ration of photograph
  //TO DO: Stop this from being hard coded. Only apply this lock when registering 3D to background photographs
  float windowWidth = 400.0;
  float windowHeight = 600.0;
  glViewport(0, 0, windowWidth, windowHeight);
  
  mainScene->getActiveCamera()->setProjection();
  engine->regenerateFrameBuffer();
  
  postScene->deleteObject("screenPlane");
  postScene->deleteObject("mainModelVerts");
  
  Object* mainModelVerts = postScene->createObject("mainModelVerts");
  if (!mainModelVerts -> createVertexCloud(mainModel)) exit(1);
  
  std::vector<Texture*> texturesVector; 
  texturesVector.push_back(new Texture(__photoFile.c_str(), MODE_INDEXED_MAP));
  //texturesVector.push_back(FrameBuffer::GetSingleton().getTexture(COLOR_PASS));
  texturesVector.push_back(FrameBuffer::GetSingleton().getTexture(PICKING_PASS));
  //texturesVector.push_back(FrameBuffer::GetSingleton().getTexture(DEPTH_PASS));
      
  Object* plane = postScene->createObject("screenPlane");
  if (!plane-> explicitLoad(      "objects/planeNormalized.obj",
                                  0, //Dont flip the uvs
                                  "wireframeMat",
                                  Engine::GetSingleton().wireframeShader, 
                                  1.0f, 
                                  0.0f, 
                                  0.0f,
                                  &texturesVector[0],
                                  2) ) exit(1);
  texturesVector.clear();
}

void ScanWidget::paintGL() {
	engine->render("MainScene", RENDER_TO_FRAMEBUFFER);
  engine->render("PostScene", RENDER_TO_WINDOW);
}

void ScanWidget::mousePressEvent(QMouseEvent* event) {
  dragStartX = event->globalX();
  dragStartY = event->globalY();
  
  if (event->button() == Qt::RightButton) {
    clickTimerRMB_Start = TimeManager::GetSingleton().getTimeSeconds();
    clickTimerRMB_Current = 0.0f;
  }  
}

void ScanWidget::mouseReleaseEvent(QMouseEvent* event) {
  float mouseX = event->x();
  float mouseY = event->y();
  
  if (event->button() == Qt::LeftButton) {
    if (__vertexPointMode) {
      __selectedVertex = Vector3({0.0, 0.0, 0.0});
      
      GLint viewport[4];
      glGetIntegerv(GL_VIEWPORT, viewport);
      float windowWidth = viewport[2];
    	float windowHeight = viewport[3];
      
      unsigned int searchDistance = 2;
      
      std::vector<Triangle*> foundTriangles;
      
      for ( unsigned int x = (((int)mouseX-searchDistance) >= 0) ? ((int)mouseX-searchDistance) : 0; 
            x < ((int)mouseX+searchDistance) && x < windowWidth;
            x++ )
      {
        for ( unsigned int y = (((int)mouseY-searchDistance) >= 0) ? ((int)mouseY-searchDistance) : 0;
              y < ((int)mouseY+searchDistance) && y < windowHeight;
              y++ )
        {
          TriangleID pickingData = FrameBuffer::GetSingleton().pickTriangle((GLuint)x, (GLuint)y);
          if (pickingData.objectID != 0) {
            Triangle* tempTriangle = engine->getTriangleByID(pickingData);
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
      Vector3 clickPosition = mainScene->getActiveCamera()->cameraToWorld(mouseX, mouseY);
      for (unsigned int t = 0; t < foundTriangles.size(); t++) {
        for (unsigned int v = 0; v < 3; v++) {
          Vector3 vertexPosition = Vector3({  foundTriangles[t]->vertices[v]->vertexPosition.x,
                                              foundTriangles[t]->vertices[v]->vertexPosition.y,
                                              foundTriangles[t]->vertices[v]->vertexPosition.z  });
                                              
          if (t == 0 && v == 0) {
            __selectedVertex = vertexPosition;
            continue;
          }
          if ( distanceSquared(clickPosition, vertexPosition) < distanceSquared(clickPosition, __selectedVertex) ) {
            __selectedVertex = vertexPosition;
          }                          
        }
      }
      log(PARAM, "Vertex Selected! Distance from mouse: %f", distance(clickPosition, __selectedVertex));
      foundTriangles.clear();   
    }
    else if (__picturePointMode) {
      GLint viewport[4];
      glGetIntegerv(GL_VIEWPORT, viewport);
      float windowWidth = viewport[2];
    	float windowHeight = viewport[3];
      __selectedPicturePoint = Vector2({mouseX/windowWidth, mouseY/windowHeight});
      __poseAlignmentData.insert(make_pair(__selectedVertex,__selectedPicturePoint));
    }
    else {
      TriangleID pickingData = FrameBuffer::GetSingleton().pickTriangle((GLuint)mouseX, (GLuint)mouseY);
      if (pickingData.objectID != 0) {
        Triangle* selectedTriangle = engine->getTriangleByID(pickingData);
        log(PARAM, "Picked Object %d, Mesh %d, Triangle %d", pickingData.objectID, pickingData.meshID, pickingData.primitiveID);
        if(selectedTriangle) {
          Surface* surface = Surface::GenSurfaceFromTriangle(selectedTriangle);
          sphereCameraController->selectSurface(surface, mainScene->getActiveCamera()->cameraToWorld(mouseX, mouseY));
          engine->simulateDynamics(this);
        }
      }
      setCursor(Qt::PointingHandCursor);
    }
  }
  else if (event->button() == Qt::RightButton) {
    clickTimerRMB_Current = TimeManager::GetSingleton().getTimeSeconds() - clickTimerRMB_Start;
    if ( clickTimerRMB_Current > 0.15f ) {
      //Rotate Behavior
      rotateMode = true;
      setCursor(Qt::PointingHandCursor);
      QAbstractEventDispatcher::instance(0)->processEvents(QEventLoop::AllEvents);
    }
    else {
      rotateMode = false;
      //FlyTo Behavior    
      Camera* currentCamera = mainScene->getActiveCamera();
      Vector3 selectedPosition = currentCamera->cameraToWorld(mouseX, mouseY);
      float farPlane = currentCamera->getFarPlane();
      if(distanceSquared(selectedPosition, currentCamera->getEye()) < (farPlane*farPlane)) {
        sphereCameraController->flyTo(selectedPosition);
        engine->simulateDynamics(this);
      }
      rotateMode = true;
      setCursor(Qt::PointingHandCursor);
    }
  }
}

void ScanWidget::mouseMoveEvent(QMouseEvent* event) {
  float dX = event->globalX() - dragStartX;
  float dY = event->globalY() - dragStartY;
  
  if (event->buttons() & Qt::RightButton) {
    if (rotateMode) {
      setCursor(Qt::ClosedHandCursor);
      sphereCameraController->rotate(dX, dY);
      dragStartX = event->globalX();
      dragStartY = event->globalY();
      updateGL();
    }
  }
  else if (event->buttons() & Qt::MiddleButton) {
    mainScene->getActiveCamera()->moveEye(-dX/10.0, dY/10.0, 0);
    QCursor::setPos(dragStartX, dragStartY);
    updateGL();
  }
  
}

void ScanWidget::wheelEvent(QWheelEvent* event) {
  float moveUnits = (float)event->delta() / 120.0f;
  //sphereCameraController->setZoom(sphereCameraController->getZoom() + moveUnits);
  updateGL();
}
  
void ScanWidget::keyPressEvent(QKeyEvent* event) {
  switch(event->key()) {
    case Qt::Key_Escape:
      // close();
      break;
    case Qt::Key_Left:
      break;
    case Qt::Key_Right:
      break;
    case Qt::Key_Up:
      break;
    case Qt::Key_Down:
      break;
    case Qt::Key_V: {
      if (!__vertexPointMode)
        log(PARAM, "Entering Vertex Selection Mode");
      __vertexPointMode = true;
      break;
    }
    case Qt::Key_P: {
      if (!__picturePointMode)
        log(PARAM, "Entering Picture Point Selection Mode");
      __picturePointMode = true;
    }
    default:
      event->ignore();
      break;
  }

  updateGL();
}

void ScanWidget::keyReleaseEvent(QKeyEvent* event) {
  switch(event->key()) {
    case Qt::Key_V: {
      log(PARAM, "Exiting Vertex Selection Mode");
      __vertexPointMode = false;
      break;
    }
    case Qt::Key_P: {
      log(PARAM, "Exiting Picture Point Selection Mode");
      __picturePointMode = false;
      break;
    }
    case Qt::Key_S: {
      string outputFile;
      outputFile = __outputPath + string("poseAlignData.txt");
      savePoseAlignmentData(outputFile.c_str());
    }
    default:
      event->ignore();
      break;
  }
  
  updateGL();
}

void 
ScanWidget::setupSceneFromFile(const string& _filename) {
  fstream configFile(_filename.c_str(), ios::in);
  string buffer;
  
  while (!configFile.eof()) {
    getline (configFile, buffer);
    
		if (buffer[0] == '#')
			continue;	// comment
    
    if (buffer.substr(0, 6) == "Model=") {
			__modelFile = buffer.substr(6, buffer.length()-6);
      log(PARAM, "Read model file: %s", __modelFile.c_str());	
      continue;
    }
    
    if (buffer.substr(0, 6) == "Photo=") {
			__photoFile = buffer.substr(6, buffer.length()-6);
      log(PARAM, "Read photo file: %s", __photoFile.c_str());	
      continue;
    }
    
    if (buffer.substr(0, 7) == "Output=") {
			__outputPath = buffer.substr(7, buffer.length()-7);
      log(PARAM, "Read output path: %s", __outputPath.c_str());	
      continue;
    }
    
    if (__modelFile.length() && __photoFile.length() && __outputPath.length()) {
      break;
    }
  }
}

void
ScanWidget::savePoseAlignmentData(const std::string& _filename) {
  log(PARAM, "Saving pose alignment data to %s", _filename.c_str());
  
  ofstream dataFile;
  dataFile.open (_filename.c_str(), std::ios::out | std::ios::app);
  if( !dataFile.is_open() ) {
    dataFile.open(_filename.c_str(), std::ios::in | std::ios::out | std::ios::trunc);
  }
  
  for(auto it = __poseAlignmentData.begin(); it != __poseAlignmentData.end(); it++) {
    dataFile << "Model=" << __modelFile.c_str() << "\n";
    dataFile << "Photo=" << __photoFile.c_str() << "\n";
    dataFile << "Vertex=" << it->first.x << " " << it->first.y << " " << it->first.z << "\n";
    dataFile << "Picture Coordinate=" << it->second.x << " " << it->second.y << "\n";
    dataFile << "\n\n";
  }
  
  dataFile.close();
}


