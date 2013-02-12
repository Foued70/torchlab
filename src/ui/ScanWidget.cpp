#include "Core3_2_context.h"
#include "ScanWidget.h"
#include "utils.h"
#include "engine/Engine.h"
#include "engine/FrameBuffer.h"
#include "engine/CameraController.h"
#include "engine/Surface.h"

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
  engine->createScene("SceneName");
  
  Object* dryDragon = engine->getCurrentScene()->createObject("dryDragon");
  if (!dryDragon -> loadFromObj("objects/dryDragon.obj", (INVERT_X | INVERT_Y))) exit(1);
     
  Camera *sphereCamera = engine->getCurrentScene()->createCamera(-20, 0, 7);
  sphereCamera -> lookAt(0, 0, 7);
  
  sphereCameraController = ControllerManager::GetSingleton().createCameraController(sphereCamera);
  sphereCameraController->setZoom(10.0f);
        
  /* Light on (7, 3, 0) position */
  Light* light = engine->getCurrentScene()->createLight(5, 15, -10);
  /* Set ambient light */
  light -> setAmbient(sColor( {0.8, 0.8, 0.8, 1.0} ));
  
  TimeManager& timer = TimeManager::GetSingleton();
  TimeManager::logTime();
  timer.logDeltaTime();
}

void ScanWidget::resizeGL(int w, int h) {
  // printf("glViewport(%d, %d)\n", w, h);
  glViewport(0, 0, w, h);
  
  engine->getCurrentScene()->getActiveCamera()->setProjection();
  engine->regenerateFrameBuffer();
}

void ScanWidget::paintGL() {
	engine->render(RENDER_TO_WINDOW | RENDER_TO_FRAMEBUFFER);
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
    TriangleID pickingData = FrameBuffer::GetSingleton().pickTriangle((GLuint)mouseX, (GLuint)mouseY);
    if (pickingData.objectID != 0) {
      Triangle* selectedTriangle = engine->getTriangleByID(pickingData);
      if(selectedTriangle) {
        Surface* surface = Surface::GenSurfaceFromTriangle(selectedTriangle);
        sphereCameraController->selectSurface(surface, engine->getCurrentScene()->getActiveCamera()->cameraToWorld(mouseX, mouseY));
        engine->simulateDynamics(this);
      }
    }
    setCursor(Qt::PointingHandCursor);
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
      Camera* currentCamera = engine->getCurrentScene()->getActiveCamera();
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
    engine->getCurrentScene()->getActiveCamera()->moveEye(-dX/10.0, dY/10.0, 0);
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
    case Qt::Key_Tab:
      break;
    default:
      event->ignore();
      break;
  }

  updateGL();
}


