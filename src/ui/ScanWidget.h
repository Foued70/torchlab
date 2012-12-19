#ifndef _SCAN_WIDGET_H
#define _SCAN_WIDGET_H

#include <QtOpenGL/QGLWidget>
#include <QtOpenGL/QGLBuffer>

class ScanWidget : public QGLWidget {
  Q_OBJECT // must include this if you use Qt signals/slots

  float camera_x, camera_y, camera_z;
  float camera_rot_h, camera_rot_v;
  
  QGLBuffer* vertexBuffer;
  QGLBuffer* polyBuffer;
  
  
  void camera();
  
public:
  ScanWidget(QWidget *parent = NULL);
  ~ScanWidget();

protected:
  void initializeGL();
  void resizeGL(int w, int h);
  void paintGL();
  void mousePressEvent(QMouseEvent *event);
  void mouseMoveEvent(QMouseEvent *event);
  void keyPressEvent(QKeyEvent *event);
};

#endif  /* _SCAN_WIDGET_H */