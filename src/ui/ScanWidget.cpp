#include "Core3_2_context.h"
#include "ScanWidget.h"
#include "utils.h"
#include "engine/Engine.h"
#include "engine/FrameBuffer.h"

#include <QtGui/QMouseEvent>
#include <QDebug>
#include <QKeyEvent>
#include <QFile>
#include <QString>

#include <iostream>
#include <string>

ScanWidget::ScanWidget(const QGLFormat& format, QWidget* parent ) : QGLWidget( new Core3_2_context(format), parent ),
    vertexBuffer( QGLBuffer::VertexBuffer ),
    polyBuffer(QGLBuffer::VertexBuffer),
    framebuffer(NULL),
    takeScreenShot(false)
{
  setMouseTracking(false);
  engine = Engine::GetSingletonPtr();
}

ScanWidget::~ScanWidget() {
	delete framebuffer;
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
  monkey -> move(0, 0, 7);
  monkey -> scale(3, 3, 3);
  monkey -> rotate(90, 0, 0);
  monkey -> setColor(80, 24, 25);
  
        
  Camera *sphereCamera = scene -> createCamera(-20, 0, 7);
  sphereCamera -> lookAt(0, 0, 7);
        
  /* Light on (7, 3, 0) position */
  Light* light = scene -> createLight(5, 15, -10);
  /* Set ambient light */
  light -> setAmbient(sColor( {0.8, 0.8, 0.8, 1.0} ));
}

void ScanWidget::resizeGL(int w, int h) {
  // printf("glViewport(%d, %d)\n", w, h);
  glViewport(0, 0, w, h);
  
  scene -> getActiveCamera() -> setProjection();
  
  if( framebuffer != NULL ) {
    delete framebuffer;
  }
  framebuffer = new FrameBuffer();
}

void ScanWidget::paintGL() {
  
  //Render scene to our framebuffer
  framebuffer -> renderToTexture();  
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	glClearColor(0.0, 0.0, 0.0, 0.0);
	engine -> render(scene);
  framebuffer -> unbind();
  
	if( takeScreenShot ) {
		takeScreenShot = false;
     framebuffer -> saveToFile("frameBufferImage");
	}
  
  //Render scene to default framebuffer (render to window)
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	glClearColor(1.0, 1.0, 1.0, 1.0);
	engine -> render(scene);
}

void ScanWidget::mousePressEvent(QMouseEvent* event) {
  // log(PARAM, "mousePressEvent %d %d (%d, %d)", event->button(), (int)event->buttons(), event->globalX(), event->globalY());
  dragStartX = event->globalX();
  dragStartY = event->globalY();  
}

void ScanWidget::mouseReleaseEvent(QMouseEvent* event) {
  float mouseX = event->x();
  float mouseY = event->y();
  
  if (event->button() == Qt::LeftButton) {
    log(PARAM, "Face ID at %d, %d is: %d" , (int)mouseX, (int)mouseY, (int)framebuffer -> readPixel((GLint)mouseX, (GLint)mouseY) - 1);
  }
}

void ScanWidget::mouseMoveEvent(QMouseEvent* event) {
  // log(PARAM, "mouseMoveEvent %d", (int)event->buttons());
  float dX = event->globalX() - dragStartX;
  float dY = event->globalY() - dragStartY;
  
  if (event->buttons() & Qt::RightButton) {
    scene->getActiveCamera()->rotateAroundCenter(-dX, dY);
    QCursor::setPos(dragStartX, dragStartY);
    updateGL();
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
    case Qt::Key_Tab:
		takeScreenShot = true;
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


