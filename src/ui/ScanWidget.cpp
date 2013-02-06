#include "Core3_2_context.h"
#include "ScanWidget.h"
#include "utils.h"
#include "engine/Engine.h"
#include "engine/FrameBuffer.h"
#include "engine/CameraController.h"

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
    rotateMode(true)
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
  
  engine->init();

  scene = engine->createScene("SceneName");
  
  /*  
  // create second object and load its data from file 
  Object* crate = scene -> createObject("crate");
  if (!crate -> loadFromObj("objects/crate.obj")) exit(1);
  crate -> scale(4, 4, 4);
        
     
  Object* monkey = scene -> createObject("monkey"); // monkey
  if (!monkey -> loadFromObj("objects/monkey.obj")) exit(1);
  monkey -> move(0, 0, 7);
  monkey -> zd
  monkey -> rotate(90, 0, 0);
  monkey -> setColor(80, 24, 25);
  */
  
  Object* dryDragon = scene -> createObject("dryDragon");
  if (!dryDragon -> loadFromObj("objects/dryDragon.obj", (INVERT_X | INVERT_Y))) exit(1);
  
  selectionModel = scene -> createObject("crate");
  if (!selectionModel -> loadFromObj("objects/crate.obj", (INVERT_X | INVERT_Y))) exit(1);
  selectionModel -> scale(0.0625f, 0.0625f, 0.0625f);
     
  Camera *sphereCamera = scene -> createCamera(-20, 0, 7);
  sphereCamera -> lookAt(0, 0, 7);
  
  sphereCameraController = ControllerManager::GetSingleton().createCameraController(sphereCamera);
        
  /* Light on (7, 3, 0) position */
  Light* light = scene -> createLight(5, 15, -10);
  /* Set ambient light */
  light -> setAmbient(sColor( {0.8, 0.8, 0.8, 1.0} ));
  
  TimeManager& timer = TimeManager::GetSingleton();
  TimeManager::logTime();
  timer.logDeltaTime();
}

void ScanWidget::resizeGL(int w, int h) {
  // printf("glViewport(%d, %d)\n", w, h);
  glViewport(0, 0, w, h);
  
  scene -> getActiveCamera() -> setProjection();
  engine -> regenerateFrameBuffer();
}

void ScanWidget::paintGL() {
	engine -> render(scene, (RENDER_TO_WINDOW | RENDER_TO_FRAMEBUFFER));
}

void ScanWidget::mousePressEvent(QMouseEvent* event) {
  // log(PARAM, "mousePressEvent %d %d (%d, %d)", event->button(), (int)event->buttons(), event->globalX(), event->globalY());
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
  
  if (event->button() == Qt::RightButton) {
    clickTimerRMB_Current = TimeManager::GetSingleton().getTimeSeconds() - clickTimerRMB_Start;
    if ( clickTimerRMB_Current > 0.1 ) {
      //Rotate Behavior
      rotateMode = true;
      QAbstractEventDispatcher::instance(0)->processEvents(QEventLoop::AllEvents);
    }
    else {
      rotateMode = false;
      //FlyTo Behavior
      log(PARAM, "Face ID at %d, %d is: %d" , (int)mouseX, (int)mouseY, (int)FrameBuffer::GetSingleton().readPixel((GLuint)mouseX, (GLuint)mouseY, (GLuint)0) - 1);
    
      unsigned int selectedObjectID = FrameBuffer::GetSingleton().readPixel((GLuint)mouseX, (GLuint)mouseY, (GLuint)1);
      log(PARAM, "Object ID at %d, %d is: %d" , (int)mouseX, (int)mouseY, (int)selectedObjectID);
    
      unsigned int selectedMeshID = FrameBuffer::GetSingleton().readPixel((GLuint)mouseX, (GLuint)mouseY, (GLuint)2);
      log(PARAM, "Mesh ID at %d, %d is: %d" , (int)mouseX, (int)mouseY, (int)selectedMeshID);
      
      if (selectedObjectID) {
        Object* selectedObject = scene->getObjectByID(selectedObjectID);
        selectedObject -> select();
      }
    
      //For debugging: Place crate model at desired fly to position
      Vector3 selectedPosition = scene->getActiveCamera()->cameraToWorld(mouseX, mouseY);
      selectionModel -> setPosition(selectedPosition.x, selectedPosition.y, selectedPosition.z);
      sphereCameraController->flyTo(selectedPosition);
      engine->simulateDynamics(this);
      rotateMode = true;
    }
  }
}

void ScanWidget::mouseMoveEvent(QMouseEvent* event) {
  // log(PARAM, "mouseMoveEvent %d", (int)event->buttons());
  float dX = event->globalX() - dragStartX;
  float dY = event->globalY() - dragStartY;
  
  if (event->buttons() & Qt::RightButton) {
    if (rotateMode) {
      scene->getActiveCamera()->rotateCenterAroundCamera(dX, dY);
      QCursor::setPos(dragStartX, dragStartY);
      updateGL();
    }
  }
  else if (event->buttons() & Qt::MiddleButton) {
    scene->getActiveCamera()->moveEye(-dX/10.0, dY/10.0, 0);
    QCursor::setPos(dragStartX, dragStartY);
    updateGL();
  }
  
}

void ScanWidget::wheelEvent(QWheelEvent* event) {
  // log(PARAM, "wheelEvent %d %d", event->delta());
  float moveUnits = (float)event->delta() / 120.0f;
  scene->getActiveCamera()->moveEye(0, 0, moveUnits);
  updateGL();
}
  
void ScanWidget::keyPressEvent(QKeyEvent* event) {
  // printf("%d\n", event->key());
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
      
    default:
      event->ignore();
      break;
  }

  updateGL();
}


