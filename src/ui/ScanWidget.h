#ifndef _SCAN_WIDGET_H
#define _SCAN_WIDGET_H

#include <QtOpenGL/QGLWidget>
#include <QtOpenGL/QGLBuffer>
#include <QtOpenGL/QGLShaderProgram>
#include "engine/Engine.h"
#include "engine/Scene.h"

class FrameBuffer;
class CameraController;
class Object;

class ScanWidget : public QGLWidget {
  Q_OBJECT // must include this if you use Qt signals/slots

public:
  ScanWidget(const QGLFormat& format, QWidget *parent = NULL);
  ~ScanWidget();
  
  void refresh();

  Engine* engine;
  Scene* scene;
  Scene* mainScene;
  Scene* postScene;
  Object* mainModel;

protected:
  void initializeGL();
  void resizeGL(int w, int h);
  void paintGL();
  void mousePressEvent(QMouseEvent *event);
  void mouseReleaseEvent(QMouseEvent* event);
  void mouseMoveEvent(QMouseEvent *event);
  void wheelEvent(QWheelEvent* event);
  void keyPressEvent(QKeyEvent *event);
  void keyReleaseEvent(QKeyEvent *event);
  
  void setupSceneFromFile(const std::string& _filename);
  void savePoseAlignmentData(const std::string& _filename);
  
private:
  bool prepareShaderProgram( const QString& vertexShaderPath, const QString& fragmentShaderPath );
  GLuint prepShaderProgram( const QString& vertexShaderPath, const QString& fragmentShaderPath );

  QGLShaderProgram shader;
  QGLBuffer vertexBuffer;
  QGLBuffer polyBuffer;

  GLuint m_vertexBuffer;
  GLuint m_shader;

  CameraController* sphereCameraController;

  int dragStartX;
  int dragStartY;
  double clickTimerRMB_Start;
  double clickTimerRMB_Current;
  bool rotateMode;
  
  bool __vertexPointMode;
  bool __picturePointMode;
  
  string __modelFile;
  string __photoFile;
  string __outputPath;
  unsigned int __outputIndex;
  
  std::map<Vector3, Vector2> __poseAlignmentData;
  
  Vector3 __selectedVertex;
  Vector2 __selectedPicturePoint;
};

#endif  /* _SCAN_WIDGET_H */