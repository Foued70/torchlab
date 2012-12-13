extern "C" {
#include <TH.h>
#include <luaT.h>
}

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif

void display(void) 
{ 
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
  glutSwapBuffers(); 
} 

void reshape(int width, int height) 
{ 
  glViewport(0, 0, width, height); 
} 

void idle(void) 
{ 
  glutPostRedisplay(); 
} 

int libui_display(lua_State *L) 
{ 
  // need to call glutInit() so we create args from a main function
  int argc = 1;
  char* argv[] ={"program"};
  glutInit(&argc, argv);

  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH); 
  glutInitWindowSize(640, 480); 

  
  glutDisplayFunc(display); 
  int win = glutCreateWindow("Floored Viewer"); 
  glutDisplayFunc(display); 
  //glutReshapeFunc(reshape); 
  //glutIdleFunc(idle); 


  glutMainLoop(); 
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

