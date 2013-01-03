#ifndef _SCAN_WIDGET_H
#define _SCAN_WIDGET_H

#include <QtOpenGL/QGLWidget>
#include <QtOpenGL/QGLBuffer>
#include <QtOpenGL/QGLShaderProgram>

#include "skylium/Skylium.h"
#include "skylium/Scene.h"

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
  void keyPressEvent(QKeyEvent *event);
  
private:
  bool prepareShaderProgram( const QString& vertexShaderPath, const QString& fragmentShaderPath );
  GLuint prepShaderProgram( const QString& vertexShaderPath, const QString& fragmentShaderPath );

  QGLShaderProgram shader;
  QGLBuffer vertexBuffer;
  QGLBuffer polyBuffer;

  GLuint m_vertexBuffer;
  GLuint m_shader;

  float camera_x, camera_y, camera_z;
  
  Skylium* skylium;
  Scene* scene;
};

#endif  /* _SCAN_WIDGET_H */