#ifndef _GL_WIDGET_H
#define _GL_WIDGET_H

#include <QtOpenGL/QGLWidget>

extern "C" {
  #include "lua.h"
  #include <lauxlib.h>
  #include <lualib.h>
}

extern char* PACKAGE_PATH;

class GLFormat : public QGLFormat {
public:
  inline GLFormat() {
    setVersion( 3, 2 );
    setProfile( QGLFormat::CompatibilityProfile ); // Requires >=Qt-4.8.0
    setSampleBuffers( true );
  }
};

class GLWidget : public QGLWidget {
  Q_OBJECT // must include this if you use Qt signals/slots

public:
  GLWidget(lua_State* L, int luaWidgetRef, QWidget* parent = NULL);
  // GLWidget(QWidget* parent = NULL);
  ~GLWidget();
  
protected:
  void initializeGL();
  void resizeGL(int w, int h);
  void paintGL();
  void mousePressEvent(QMouseEvent *event);
  void mouseReleaseEvent(QMouseEvent* event);
  void mouseMoveEvent(QMouseEvent *event);
  void wheelEvent(QWheelEvent* event);
  void keyPressEvent(QKeyEvent *event);
  
private:
  lua_State* L;
  int luaWidgetRef;

  int callLua(int inCount, int outCount);
  void selfFunction(const char* functionName);
  void setTableInt(const char* name, int value);
  void pushMouseEvent(QMouseEvent* event);
};

#endif  /* _GL_WIDGET_H */