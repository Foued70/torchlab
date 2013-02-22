extern "C" {
#include <TH.h>
#include <luaT.h>
}

#include <QtGui/QApplication>
#include <QtOpenGL/QGLWidget>
#include "LuaObject.h"
#include "ScanWidget.h"
#include "engine/Engine.h"
#include "utils.h"


ScanWidget* scanWidget;

// gui component holder which will be moved to main thread
class gui_launcher : public QObject
{
  public:
  virtual bool event( QEvent *ev ) {   
    // printf("event %d\n", ev->type());

    if( ev->type() == QEvent::User ) {
      /* create the main Engine class instance */
      Engine *s_main = new Engine();
        
      QGLFormat glFormat;
      glFormat.setVersion( 3, 2 );
      glFormat.setProfile( QGLFormat::CompatibilityProfile ); // Requires >=Qt-4.8.0
      glFormat.setSampleBuffers( true );

      // Create a GLWidget requesting our format
      scanWidget = new ScanWidget(glFormat);
      scanWidget->resize(400,600);
      scanWidget->show();
      return true;
    }
    return false;
  }
};

int libui_display(lua_State *L) {
  // create holder
  gui_launcher* gl = new gui_launcher;
  // move it to main thread
  gl->moveToThread( QApplication::instance()->thread() );
  // send it event which will be posted from main thread
  QCoreApplication::postEvent( gl, new QEvent( QEvent::User ) );

  return 0;
}


int libui_create_object(lua_State *L) {
  LuaObject obj = LuaObject(L, 1);

  scanWidget->makeCurrent();
  Scene* scene = scanWidget->scene;
  Object* object = scene->createObject("space");
  printf("load\n");
  object->loadFrom(&obj);
  // object->scale(-1, 1, 1);
  // object -> move(0, 0, 0);
  // object -> rotate(0, 0, 0);
  // object -> setColor(80, 24, 25);

  scanWidget->updateGL();

  return 0;
}



static const luaL_reg libui_init[] =
{
  {"display", libui_display},
  {"create_object", libui_create_object},
  {NULL, NULL}
};

LUA_EXTERNC DLL_EXPORT int luaopen_libui(lua_State *L)
{
  luaL_register(L, "libui", libui_init);
  return 1;
}
