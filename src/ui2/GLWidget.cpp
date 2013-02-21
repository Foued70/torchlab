#include "GLWidget.h"
#include "Core3_2_context.h"

#include <QtGui/QMouseEvent>
#include <iostream>
using namespace std;

Core3_2_context* CORE_CONTEXT = new Core3_2_context(GLFormat());

GLWidget::GLWidget(lua_State* L, int luaWidgetRef, QWidget* parent) : QGLWidget( CORE_CONTEXT, parent ), 
  L(L),
  luaWidgetRef(luaWidgetRef)
{
  resize(800,600);
  show();
}


GLWidget::~GLWidget() {
}

int 
GLWidget::callLua(int inCount, int outCount) {
  int err = lua_pcall(L, inCount, outCount, 0);
  if (err) {
    cout << luaL_checkstring(L, -1) << '\n';
    lua_pop(L, 1);
  }

  return err;
}

void 
GLWidget::selfFunction(const char* functionName) {
  lua_rawgeti(L, LUA_REGISTRYINDEX, luaWidgetRef); // get the GLWidget instance from the registry
  lua_getfield(L, -1, functionName); // push the fucntion name
  lua_pushvalue(L, -2); // push self

  lua_remove(L, -3); // remove the original instance
  // lua_pushvalue(L, -2);
}

void
GLWidget::setTableInt(const char* name, int value) {
  lua_pushinteger(L, value);
  lua_setfield(L, -2, name);
}

void
GLWidget::pushMouseEvent(QMouseEvent* event) {
  lua_createtable(L, 0, 8);
  setTableInt("x", event->x());
  setTableInt("y", event->y());
  setTableInt("global_x", event->globalX());
  setTableInt("global_y", event->globalY());
  setTableInt("button", event->button());
}

void 
GLWidget::initializeGL() {
  selfFunction("init");
  lua_pushlightuserdata(L, this);
  callLua(2, 0);
}

void GLWidget::resizeGL(int width, int height) {
  selfFunction("resize");
  lua_pushinteger(L, width);
  lua_pushinteger(L, height);
  callLua(3, 0);
}

void GLWidget::paintGL() {
  selfFunction("paint");
  callLua(1, 0);
}

void GLWidget::mousePressEvent(QMouseEvent* event) {
  selfFunction("mouse_press");
  pushMouseEvent(event);
  callLua(2, 0);
}

void GLWidget::mouseReleaseEvent(QMouseEvent* event) {
  selfFunction("mouse_release");
  pushMouseEvent(event);
  callLua(2, 0);
}

void GLWidget::mouseMoveEvent(QMouseEvent* event) {
  selfFunction("mouse_move");
  pushMouseEvent(event);
  callLua(2, 0);
}

void GLWidget::wheelEvent(QWheelEvent* event) {
  selfFunction("mouse_wheel");
  lua_createtable(L, 0, 8);
  setTableInt("delta", event->delta());
  callLua(2, 0);
}
  
void GLWidget::keyPressEvent(QKeyEvent* event) {
  selfFunction("key_press");
  lua_createtable(L, 0, 8);
  setTableInt("key", event->key());
  callLua(2, 0);
}


