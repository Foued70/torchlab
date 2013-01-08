#include "Core3_2_context.h"
#include <QtGui/QMouseEvent>
#include "ScanWidget.h"
#include "engine/Engine.h"
#include <QDebug>
#include <QKeyEvent>
#include <QFile>
#include <QString>

#include <iostream>
#include <string>

ScanWidget::ScanWidget(const QGLFormat& format, QWidget* parent ) : QGLWidget( new Core3_2_context(format), parent ),
      vertexBuffer( QGLBuffer::VertexBuffer ),
      polyBuffer(QGLBuffer::VertexBuffer)
{
  setMouseTracking(false);
  camera_x = 4; camera_y = 4; camera_z = 4;
  engine = Engine::GetSingletonPtr();
}

ScanWidget::~ScanWidget() {
}



void ScanWidget::initializeGL() {
  printf("OpenGL %s GLSL %s\n", glGetString(GL_VERSION), glGetString(GL_SHADING_LANGUAGE_VERSION));
  
  engine->init();

  scene = engine->createScene("SceneName");
        
  /* create second object and load its data from file */
  Object* crate = scene -> createObject("crate");
  if (!crate -> loadFromObj("objects/crate.obj")) exit(1);
  crate -> scale(4, 4, 4);
        
  Object* monkey = scene -> createObject("monkey"); // monkey
  if (!monkey -> loadFromObj("objects/monkey.obj")) exit(1);
  monkey -> move(0, 7, 0);
  monkey -> scale(3, 3, 3);
  monkey -> rotate(0, -45, 35);
  monkey -> setColor(80, 24, 25);
        
  /* Camera of (0, 4, -20) position, looking at (0, 7, 20) */
  // Camera* fppCamera = scene -> createCamera(0, 4, -20, FPP);
  // fppCamera -> lookAt(-0.1, 9.7, -19);
  // fppCamera -> lookAt(0.0, 7.0, 20.0);
        
  Camera *sphereCamera = scene -> createCamera(0, 7, -20, SPHERICAL);
  sphereCamera -> lookAt(0, 7, 0);
        
  /* Light on (7, 3, 0) position */
  Light* light = scene -> createLight(5, 15, -10);
  /* Set ambient light */
  light -> setAmbient(sColor( {0.8, 0.8, 0.8, 1.0} ));
}

void ScanWidget::resizeGL(int w, int h) {
  printf("glViewport(%d, %d)\n", w, h);
  glViewport(0, 0, w, h);
  
  scene -> getActiveCamera() -> setProjection();
}

void ScanWidget::paintGL() {
  engine -> render(scene);
}

void ScanWidget::mousePressEvent(QMouseEvent *event) {

}
void ScanWidget::mouseMoveEvent(QMouseEvent *event) {
    // printf("%d, %d\n", event->x(), event->y());
}

void ScanWidget::keyPressEvent(QKeyEvent* event) {
  // printf("%d\n", event->key());
  switch(event->key()) {
    case Qt::Key_Escape:
      // close();
      break;
      
    case Qt::Key_Left:
      scene -> getActiveCamera() -> rotateCamera(20.0, 0.0, 0.0);
      break;
    case Qt::Key_Right:
      scene -> getActiveCamera() -> rotateCamera(-20.0, 0.0, 0.0);
      break;
    case Qt::Key_Up:
      scene -> getActiveCamera() -> rotateCamera(0.0, 20.0, 0.0);
      break;
    case Qt::Key_Down:
      scene -> getActiveCamera() -> rotateCamera(0.0, -20.0, 0.0);
      break;
      
    case Qt::Key_S:
      scene -> getActiveCamera() -> moveCamera(0.0, 0.0, -0.1);
      break;
    case Qt::Key_W:
      scene -> getActiveCamera() -> moveCamera(0.0, 0.0, 0.1);
      break;
    case Qt::Key_D:
      scene -> getActiveCamera() -> moveCamera(0.1, 0.0, 0.0);
      break;
    case Qt::Key_A:
      scene -> getActiveCamera() -> moveCamera(-0.1, 0.0, 0.0);
      break;
    case Qt::Key_Z:
      scene -> getActiveCamera() -> moveCamera(0.0, -0.7, 0.0);
      break;
    case Qt::Key_X:
      scene -> getActiveCamera() -> moveCamera(0.0, 0.7, 0.0);
      break;
                
    case Qt::Key_Q:
      scene -> getActiveCamera() -> setRange((scene -> getActiveCamera() -> getRange()) + 0.5);
      break;
    case Qt::Key_E:
      scene -> getActiveCamera() -> setRange((scene -> getActiveCamera() -> getRange()) - 0.5);
      break;

    default:
      event->ignore();
      break;
  }

  updateGL();
}


