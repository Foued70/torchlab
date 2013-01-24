extern "C" {
#include <TH.h>
#include <luaT.h>
}

#include <QtGui/QApplication>
#include <QtOpenGL/QGLWidget>
#include "ScanWidget.h"
#include "engine/Engine.h"

// gui component holder which will be moved to main thread
class gui_launcher : public QObject
{
  ScanWidget* window;

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
      window = new ScanWidget(glFormat);
      window->resize(800,600);
      window->show();
      return true;
    }
    return false;
  }
};

int libui_display(lua_State *L) {
  THDoubleTensor* objData = NULL;
  printf("%d\n", lua_gettop(L));
  if(lua_gettop(L) == 1) {
    objData = (THDoubleTensor*)luaT_toudata(L, 1, "torch.DoubleTensor");
  }
  // create holder
  gui_launcher* gl = new gui_launcher;
  // move it to main thread
  gl->moveToThread( QApplication::instance()->thread() );
  // send it event which will be posted from main thread
  QCoreApplication::postEvent( gl, new QEvent( QEvent::User ) );

  return 0;
}


static const luaL_reg libui_init[] =
{
  {"display", libui_display},
  {NULL, NULL}
};

LUA_EXTERNC DLL_EXPORT int luaopen_libui(lua_State *L)
{
  luaL_register(L, "libui", libui_init);
  return 1;
}

