#ifndef _SCAN_WIDGET_H
#define _SCAN_WIDGET_H

#include <QtOpenGL/QGLWidget>
#include <QtOpenGL/QGLBuffer>
#include <QtOpenGL/QGLShaderProgram>

#include "engine/Engine.h"
#include "engine/Scene.h"


class FrameBuffer;

class ScanWidget : public QGLWidget {
  Q_OBJECT // must include this if you use Qt signals/slots

public:
  ScanWidget(const QGLFormat& format, QWidget *parent = NULL);
  ~ScanWidget();

protected:
  void initializeGL();
  void resizeGL(int w, int h);
  void paintGL();
  void mousePressEvent(QMouseEvent *event);
  void mouseMoveEvent(QMouseEvent *event);
  void wheelEvent(QWheelEvent* event);
  void keyPressEvent(QKeyEvent *event);
  
private:
  bool prepareShaderProgram( const QString& vertexShaderPath, const QString& fragmentShaderPath );
  GLuint prepShaderProgram( const QString& vertexShaderPath, const QString& fragmentShaderPath );

  QGLShaderProgram shader;
  QGLBuffer vertexBuffer;
  QGLBuffer polyBuffer;

  GLuint m_vertexBuffer;
  GLuint m_shader;

  Engine* engine;
  Scene* scene;
  FrameBuffer* framebuffer;
  bool takeScreenShot;
  
  int dragStartX;
  int dragStartY;
};

#endif  /* _SCAN_WIDGET_H */