extern "C" {
#include <TH.h>
#include <luaT.h>
}

#include <QtGui/QApplication>
#include <QtOpenGL/QGLWidget>
#include "GLWidget.h"

// gui component holder which will be moved to main thread
class gui_launcher : public QObject
{
  public:
    lua_State* L;
    int luaWidgetRef;

  virtual bool event( QEvent *ev ) {
    if( ev->type() == QEvent::User ) {
      // Create a GLWidget requesting our format
      GLWidget* glWidget = new GLWidget(L, luaWidgetRef);
      return true;
    }
    return false;
  }
};

// int libui2_display(lua_State *L) {
//   if (!PACKAGE_PATH) {
//     lua_getglobal(L, "package");
//     lua_getfield(L, -1, "path");
//     PACKAGE_PATH = (char*)luaL_checkstring(L, -1);
//     lua_pop(L, 2);
//   }

//   // create holder
//   gui_launcher* gl = new gui_launcher;
//   // move it to main thread
//   gl->moveToThread( QApplication::instance()->thread() );
//   // send it event which will be posted from main thread
//   QCoreApplication::postEvent( gl, new QEvent( QEvent::User ) );

//   return 0;
// }


int libui2_attach_qt(lua_State* L) {
  gui_launcher* gl = new gui_launcher;
  gl->L = L;
  lua_pushvalue(L, -1);
  gl->luaWidgetRef = luaL_ref(L, LUA_REGISTRYINDEX);
  // move it to main thread
  gl->moveToThread( QApplication::instance()->thread() );
  // send it event which will be posted from main thread
  QCoreApplication::postEvent( gl, new QEvent( QEvent::User ) );

  return 0;
}

int libui2_int_storage_info(lua_State* L) {
  THIntTensor* tensor = (THIntTensor*)luaT_checkudata(L, 1, "torch.IntTensor");
  lua_pushlightuserdata(L, tensor->storage->data);
  lua_pushinteger(L, tensor->storage->size * sizeof(int));
  return 2;
}

int libui2_double_storage_info(lua_State* L) {
  THDoubleTensor* tensor = (THDoubleTensor*)luaT_checkudata(L, 1, "torch.DoubleTensor");
  lua_pushlightuserdata(L, tensor->storage->data);
  lua_pushinteger(L, tensor->storage->size * sizeof(double));
  return 2;
}


static const luaL_reg libui2_init[] =
{
  // {"display", libui2_display},
  {"attach_qt", libui2_attach_qt},
  {"int_storage_info", libui2_int_storage_info},
  {"double_storage_info", libui2_double_storage_info},
  {NULL, NULL}
};

LUA_EXTERNC DLL_EXPORT int luaopen_libui2(lua_State *L)
{
  luaL_register(L, "libui2", libui2_init);
  return 1;
}
