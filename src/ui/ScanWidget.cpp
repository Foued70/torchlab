#include "Core3_2_context.h"
#include "ScanWidget.h"
#include "utils.h"
#include "engine/Engine.h"
#include "engine/FrameBuffer.h"
#include "engine/CameraController.h"
#include "engine/Surface.h"
#include "engine/Texture.h"
#include "engine/PoseRefinementDataHandler.h"

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
    __picturePointMode(false)
{
  setMouseTracking(false);
  engine = Engine::GetSingletonPtr();
}

ScanWidget::~ScanWidget() {
  delete poseRefinementDataHandler;
}

void 
ScanWidget::refresh() {
  QAbstractEventDispatcher::instance(0)->processEvents(QEventLoop::AllEvents);
  updateGL();
}

void 
ScanWidget::initializeGL() {
  printf("OpenGL %s GLSL %s\n", glGetString(GL_VERSION), glGetString(GL_SHADING_LANGUAGE_VERSION));
  
  engine->init();
  mainScene = engine->createScene("MainScene");
  scene = mainScene;
  postScene = engine->createScene("PostScene");
  
  Camera *sphereCamera = mainScene->createCamera(-20, 0, 7);
  sphereCamera -> lookAt(0, 0, 7);
  
  sphereCameraController = ControllerManager::GetSingleton().createCameraController(sphereCamera);
  sphereCameraController->setZoom(10.0f);
  
  poseRefinementDataHandler = new PoseRefinementDataHandler("config/poseAlignConfig.txt");
  
  sphereCameraController->setPositionAndRotation( poseRefinementDataHandler->getStartPosition(),
                                                  Vector3({0.0f, 0.0f, 0.0f}) );                                                
                                                  
  mainModel = mainScene->createObject("mainModel");
  log(PARAM, "modelPath=%s", poseRefinementDataHandler->getModelPath().c_str());
  if (!mainModel -> loadFromObj(poseRefinementDataHandler->getModelPath().c_str())) exit(1);
  
        
  /* Light on (7, 3, 0) position */
  Light* light = mainScene->createLight(5, 15, -10);
  /* Set ambient light */
  light -> setAmbient(sColor( {0.8, 0.8, 0.8, 1.0} ));
  
  TimeManager& timer = TimeManager::GetSingleton();
  TimeManager::logTime();
  timer.logDeltaTime();

  log(PARAM, "Init complete");
}

void ScanWidget::resizeGL(int w, int h) {
  // printf("glViewport(%d, %d)\n", w, h);
  
  //Lock width to aspect ration of photograph
  //TO DO: Stop this from being hard coded. Only apply this lock when registering 3D to background photographs
  log(PARAM, "resizing");
  float windowWidth = 400.0;
  float windowHeight = 600.0;
  glViewport(0, 0, windowWidth, windowHeight);
  
  mainScene->getActiveCamera()->setProjection();
  engine->regenerateFrameBuffer();
  log(PARAM, "regenerated framebuffer");
  postScene->deleteObject("screenPlane");
  postScene->deleteObject("mainModelVerts");
  log(PARAM, "deleting objects");
  Object* mainModelVerts = postScene->createObject("mainModelVerts");
  log(PARAM, "created mainModelVerts");
  if (!mainModelVerts -> createVertexCloud(mainModel)) {
    log(PARAM, "well shit"); 
    exit(1);
  }
  log(PARAM, "didn't exit");  
  poseRefinementDataHandler->loadPhotoPlane();
  log(PARAM, "finished resizing");
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
      Vertex* selectedVertex = engine->pickVertex(mouseX, mouseY, 2, 0.3f);
      if (selectedVertex) {
        poseRefinementDataHandler->selectVertex(Vector3({ selectedVertex->vertexPosition.x, 
                                                          selectedVertex->vertexPosition.y, 
                                                          selectedVertex->vertexPosition.z})); 
      }
    }
    else if (__picturePointMode) {
      GLint viewport[4];
      glGetIntegerv(GL_VIEWPORT, viewport);
      float windowWidth = viewport[2];
    	float windowHeight = viewport[3];
      log(PARAM, "Photo point selected.");
      poseRefinementDataHandler->selectPhotoPoint(Vector2({mouseX/windowWidth, mouseY/windowHeight}));
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
      //poseRefinementDataHandler->save("PoseRefinmentCalibrationData.txt");
    }
    default:
      event->ignore();
      break;
  }
  
  updateGL();
}


