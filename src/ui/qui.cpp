extern "C" {
#include <TH.h>
#include <luaT.h>
}

#include <sstream>
#include <unordered_map>
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


typedef std::unordered_map<const char*, void*> Map;

Map* lua_table_to_map(lua_State *L, int index) {
  Map* map = new Map();
  lua_pushnil(L);
  while (lua_next(L, index) != 0) {
    /* uses 'key' (at index -2) and 'value' (at index -1) */
    const char* name;
    if (lua_isnumber(L, -2)) {
      lua_Number j = luaL_checknumber(L, -2);
      // convert ot int handling float rounding issue
      int k = (int)(j+(j<0?-0.5:0.5));
      std::stringstream ss;
      ss << k;
      name = ss.str().c_str();
    }
    else if (lua_isstring(L, -2)) {
      name = luaL_checkstring(L, -2);
    }
    

    printf("%s: %s\n", name, lua_typename(L, lua_type(L, -1)));
    if (name[0] != '_') {
      void* value = NULL;
      if (lua_isuserdata(L, -1)) {
        value = lua_touserdata(L, -1);
      }
      else if (lua_istable(L, -1)) {
        value = (void*)lua_table_to_map(L, lua_gettop(L));
      }

      (*map)[name] = value;
    }

    /* removes 'value'; keeps 'key' for next iteration */
    lua_pop(L, 1);
  }


  return map;
}


int libui_create_object(lua_State *L) {
  Map* map = lua_table_to_map(L, 1);



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
