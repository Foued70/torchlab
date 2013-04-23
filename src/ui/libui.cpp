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

      delete this;
      
      return true;
    }
    return false;
  }
};


int libui_attach_qt(lua_State* L) {
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

int libui_update_gl(lua_State* L) {
  GLWidget* glWidget = (GLWidget*)lua_touserdata(L, 1);
  glWidget->update();

  return 0;
}

int libui_widget_resize(lua_State* L) {
  GLWidget* glWidget = (GLWidget*)lua_touserdata(L, 1);
  int w = luaL_checkinteger(L, 2);
  int h = luaL_checkinteger(L, 3);
  glWidget->resize(w, h);

  return 0;
}

int libui_make_current(lua_State* L) {
  GLWidget* glWidget = (GLWidget*)lua_touserdata(L, 1);
  glWidget->makeCurrent();

  return 0;
}

int libui_hide_widget(lua_State* L) {
  GLWidget* glWidget = (GLWidget*)lua_touserdata(L, 1);
  glWidget->hide();

  return 0;
}

int libui_int_storage_info(lua_State* L) {
  THIntTensor* tensor = (THIntTensor*)luaT_checkudata(L, 1, "torch.IntTensor");
  lua_pushlightuserdata(L, tensor->storage->data);
  lua_pushinteger(L, tensor->storage->size * sizeof(int));
  return 2;
}

int libui_byte_storage_info(lua_State* L) {
  THByteTensor* tensor = (THByteTensor*)luaT_checkudata(L, 1, "torch.ByteTensor");
  lua_pushlightuserdata(L, tensor->storage->data);
  lua_pushinteger(L, tensor->storage->size * 1);
  return 2;
}

int libui_double_storage_info(lua_State* L) {
  THDoubleTensor* tensor = (THDoubleTensor*)luaT_checkudata(L, 1, "torch.DoubleTensor");
  lua_pushlightuserdata(L, tensor->storage->data);
  lua_pushinteger(L, tensor->storage->size * sizeof(double));
  return 2;
}

int libui_float_storage_info(lua_State* L) {
  THFloatTensor* tensor = (THFloatTensor*)luaT_checkudata(L, 1, "torch.FloatTensor");
  lua_pushlightuserdata(L, tensor->storage->data);
  lua_pushinteger(L, tensor->storage->size * sizeof(float));
  return 2;
}


static const luaL_reg libui_init[] =
{
  // {"display", libui_display},
  {"attach_qt", libui_attach_qt},
  {"update_gl", libui_update_gl},
  {"widget_resize", libui_widget_resize},
  {"make_current", libui_make_current},
  {"hide_widget", libui_hide_widget},
  {"int_storage_info", libui_int_storage_info},
  {"byte_storage_info", libui_byte_storage_info},
  {"double_storage_info", libui_double_storage_info},
  {"float_storage_info", libui_float_storage_info},
  {NULL, NULL}
};

LUA_EXTERNC DLL_EXPORT int luaopen_libui(lua_State *L)
{
  luaL_register(L, "libui", libui_init);
  return 1;
}
