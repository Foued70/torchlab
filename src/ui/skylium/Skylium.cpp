 /*
     ___ _        _ _                           
    / __| |___  _| (_)_  _ _ __    __ _ __ _ __ 
    \__ \ / / || | | | || | '  \ _/ _| '_ \ '_ \
    |___/_\_\\_, |_|_|\_,_|_|_|_(_)__| .__/ .__/
             |__/                    |_|  |_|   

    Copyright (C) 2011  Michał Garapich garrappachc@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
     
#include <sys/stat.h>
// #include <X11/Xlib.h>
// #include <X11/Xutil.h>
// #include <GL/glx.h>
#include <unistd.h>

#include "Skylium.h"

#include "defines.h"
#include "config.h"
#include "utils.h"

using namespace std;


Skylium::Skylium() : 
		Scenes(__sceneManagement),
		Textures(__textureManagement),
    // TheHud(__hud),
		GlobalTimer(__timer),
		Matrices(__matricesManagement),
		Shaders(__shaderDataHandling),
		VBOManagement(__vboManagement),
		identityShader(NULL),
		shadingShader(NULL),
		texturedShadingShader(NULL),
		normalMapShader(NULL),
		__sceneManagement(NULL),
		__textureManagement(NULL),
		__matricesManagement(NULL),
		__shaderDataHandling(NULL),
    // __pendingKeys(KEY_NOKEY),
		__isMouseMotionEnabled(false),
		__lastMousePositionX(0),
		__lastMousePositionY(0),
		__windowWidth(__GLXContext.winWidth),
		__windowHeight(__GLXContext.winHeight),
		__shaderList(0),
    // __hud(NULL),
		__timer(NULL) {
			
	__loadConfig("skylium.cfg");
	
	/* Create instances of singletons */
	__sceneManagement = new SceneManager();
	__textureManagement = new TextureManager();
	__matricesManagement = new MatricesManager();
	__shaderDataHandling = new ShaderDataHandler();
	__vboManagement = new GPUMemory();
	
	__timer = new Timer();
	
	log(CONSTRUCTOR, "Skylium constructed.");
}

Skylium::~Skylium() {
	delete __textureManagement;
	delete __sceneManagement;
  // delete __hud;
	delete __matricesManagement;
	delete __shaderDataHandling;
	delete __vboManagement;
	
	delete __timer;
	
	while (!__shaderList.empty())
		delete __shaderList.back(), __shaderList.pop_back();
	
	__destroyContextAndWindow();
	
	while (!__extensions.empty())
		delete __extensions.back(), __extensions.pop_back();
	
	log(DESTRUCTOR, "Skylium destructed.");
}

bool
Skylium::init() {
	
  // __earlyInitGLXFnPointers();
	
	__GLXContext.mousePosX = 0;
	__GLXContext.mousePosY = 0;
	
  // XVisualInfo *visualInfo = nullptr;
  // GLXFBConfig bestFbConfig;
	
  // if (!__openXServerConnection())
  //   return false;
  // 
  // if (!__getGLXVersion())
  //   return false;
  // 
  // if (!__getBestConfig(visualInfo, bestFbConfig))
  //   return false;
  //  
  // if (!__openXWindow(visualInfo, _windowName))
  //   return false;
  // 
  // if (!__createGLXContext(bestFbConfig))
  //   return false;
	
  // __getExtensionList();
  //   cout << "__getExtensionList\n";
  // 
  // __checkSupportedExtensions();
  //   cout << "__checkSupportedExtensions\n";
	
  // if (!sGlobalConfig::MOUSE_VISIBLE)
  //   __hideMousePointer();
	
  // glinitGLExtensionsPointers();
	
  // glViewport(0, 0, __GLXContext.winWidth, __GLXContext.winHeight); checkGLErrors(AT);
  // 
  // glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
  // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  // glEnable(GL_DEPTH_TEST);
  checkGLErrors(AT);
	
	
  // if (sGlobalConfig::HUD_EXISTS)
  //   TheHud -> prepare();
	
	/* Construct and compile default shaders. */
	identityShader = createShader("identity");
	shadingShader = createShader("shadow");
	texturedShadingShader = createShader("textured");
	normalMapShader = createShader("normalmap");
	
	return true;
	
}

void
Skylium::execute() {
  // log(PARAM, "Skylium::execute");
	__catchEvents();
	__render();
  // if (TheHud -> visible())
  //   TheHud -> draw();
}

Scene *
Skylium::createScene(const string &_sceneName) {
	Scene *newScene = __sceneManagement -> createScene(_sceneName);
	return newScene;
}

Shader*
Skylium::createShader(const string& _fileName) {
	Shader* newShader = new Shader(_fileName);
	__shaderList.push_back(newShader);
	return newShader;
}

bool
Skylium::isSupported(const string &_ext) {
	// lambda expression
	auto comparator = [](const string *a, const string *b) -> bool {
		return (*a) < (*b);
	};
	
	return binary_search(__extensions.begin(), __extensions.end(), &_ext, comparator);
}

void
Skylium::__render() {
	__sceneManagement -> displayActiveScene();
}

void
Skylium::__catchEvents() {
  // __pendingKeys = KEY_NOKEY;
  // 
  // XEvent xev;
  // 
  // while(XPending(__GLXContext.display)) {
  //   XNextEvent(__GLXContext.display, &xev);
  //   
  //   switch (xev.type) {
  //     case  KeyPress:
  //       doYourJob(XLookupKeysym(&xev.xkey, 0), __pendingKeys);
  //       break;
  //     case MotionNotify:
  //       if (__isMouseMotionEnabled) {
  //         int x = xev.xmotion.x;
  //         int y = xev.xmotion.y;
  //         const GLfloat moveX = (__lastMousePositionX - x);
  //         const GLfloat moveY = (__lastMousePositionY - y);
  //         __sceneManagement->getActiveScene()->getActiveCamera()->rotateCamera(moveX, moveY, 0.0);
  //         __lastMousePositionX = x;
  //         __lastMousePositionY = y;
  //       }
  //       break;
  //     case ConfigureNotify:
  //       XWindowAttributes winData;
  //       XGetWindowAttributes(__GLXContext.display, __GLXContext.window, &winData);
  //       __GLXContext.winWidth = winData.width;
  //       __GLXContext.winHeight = winData.height;
  //       __sceneManagement->getActiveScene()->getActiveCamera()->setProjection();
  //       break;
  //   }
  // }
	
}

void
Skylium::__loadConfig(const string& _fileName) {
	
	if (!__fileExists(_fileName)) {
		log(WARN, "No config file found. Default values in use.");
    return;
  }
	
	fstream configFile(_fileName.c_str(), ios::in);
	
	string buffer, param, value;
	
	while (!configFile.eof()) {
		param = "";
		getline(configFile, buffer);
		
		if (buffer[0] == '#')
			continue;
		
		istringstream line(buffer);
		
		line >> param >> value;
		
		if (param == "fullscreen") {
			if (value == "false" || value == "0")
				sGlobalConfig::FULLSCREEN_RENDERING = false;
			else if (value == "true" || value == "1")
				sGlobalConfig::FULLSCREEN_RENDERING = true;
			else
				cout << LOG_ERROR << "Value " << value <<
					" invalid. Possible values for \"fullscreen\" are 0, 1, false or true.";
		} else if (param == "hud_exists") {
			if (value == "false" || value == "0")
				sGlobalConfig::HUD_EXISTS = false;
			else if (value == "true" || value == "1")
				sGlobalConfig::HUD_EXISTS = true;
			else
				cout << LOG_ERROR << "Value " << value <<
					" invalid. Possible valuse for \"hud_exists\" are 0, 1, false or true.";
		} else if (param == "mouse_visible") {
			if (value == "false" || value == "0")
				sGlobalConfig::MOUSE_VISIBLE = false;
			else if (value == "true" || value == "1")
				sGlobalConfig::MOUSE_VISIBLE = true;
			else
				cout << LOG_ERROR << "Value " << value <<
					" invalid. Possible values for \"" << param << "\" are 0, 1, false or true.";
		} else if (param == "gl_red_size") {
			sGlobalConfig::GL_RED_SIZE = string2T< short >(value);
			if (sGlobalConfig::GL_RED_SIZE < 0)
				cout << LOG_WARN << "Buffer size is less than 0! It is not going to work as expected.";
		} else if (param == "gl_blue_size") {
			sGlobalConfig::GL_BLUE_SIZE = string2T< short >(value);
			if (sGlobalConfig::GL_BLUE_SIZE < 0)
				cout << LOG_WARN << "Buffer size is less than 0! It is not going to work as expected.";
		} else if (param == "gl_green_size") {
			sGlobalConfig::GL_GREEN_SIZE = string2T< short >(value);
			if (sGlobalConfig::GL_GREEN_SIZE < 0)
				cout << LOG_WARN << "Buffer size is less than 0! It is not going to work as expected.";
		} else if (param == "gl_depth_size") {
			sGlobalConfig::GL_DEPTH_SIZE = string2T< short >(value);
			if (sGlobalConfig::GL_DEPTH_SIZE < 0)
				cout << LOG_WARN << "Buffer size is less than 0! It is not going to work as expected.";
		} else if (param == "create_mipmaps") {
			if (value == "false" || value == "0")
				sGlobalConfig::CREATE_MIPMAPS = false;
			else if (value == "true" || value == "1")
				sGlobalConfig::CREATE_MIPMAPS = true;
			else
				cout << LOG_ERROR << "Value " << value <<
					" invalid. Possible values for \"" << param << "\" are 0, 1, false or true.";
		} else if (param == "tellmeabout") {
			sGlobalConfig::DEBUGGING = D_NOTHING;
			
			vector< string > flags;
			
			explode(value, ',', flags);
			
			for (vector< string >::const_iterator it = flags.begin(); it != flags.end(); it++) {
				if (*it == "nothing") {
					sGlobalConfig::DEBUGGING = D_NOTHING;
					break;
				} else if (*it == "constructors")
					sGlobalConfig::DEBUGGING |= D_CONSTRUCTORS;
				else if (*it == "all_constructors")
					sGlobalConfig::DEBUGGING |= D_ALL_CONSTRUCTORS;
				else if (*it == "destructors")
					sGlobalConfig::DEBUGGING |= D_DESTRUCTORS;
				else if (*it == "params")
					sGlobalConfig::DEBUGGING |= D_PARAMS;
				else if (*it == "all_params")
					sGlobalConfig::DEBUGGING |= D_ALL_PARAMS;
				else if (*it == "shaders")
					sGlobalConfig::DEBUGGING |= D_SHADERS;
				else if (*it == "buffer")
					sGlobalConfig::DEBUGGING |= D_BUFFER;
				else if (*it == "warnings")
					sGlobalConfig::DEBUGGING |= D_WARNINGS;
				else if (*it == "errors")
					sGlobalConfig::DEBUGGING |= D_ERRORS;
				else if (*it == "everything") {
					sGlobalConfig::DEBUGGING = D_CONSTRUCTORS 
								|	D_ALL_CONSTRUCTORS
								|	D_DESTRUCTORS
								|	D_PARAMS
								|	D_ALL_PARAMS
								|	D_SHADERS
								|	D_BUFFER
								|	D_WARNINGS
								|	D_ERRORS;
					break;
				} else {
					cout << LOG_WARN << "ConfigParser: unrecognized option: " << *it;
				}
			}
		} else if (param != "") {
			cout << LOG_WARN << "ConfigParser: unrecognized option: \"" << param << "\"";
		}
	}
	configFile.close();
}

// void
// Skylium::__earlyInitGLXFnPointers() {
//   glXCreateContextAttribsARB = getProcAddr< decltype(glXCreateContextAttribsARB) >("glXCreateContextAttribsARB");
//   glXChooseFBConfig = getProcAddr< decltype(glXChooseFBConfig) >("glXChooseFBConfig");
//   glXGetVisualFromFBConfig = getProcAddr< decltype(glXGetVisualFromFBConfig) >("glXGetVisualFromFBConfig");
//   glXGetFBConfigAttrib = getProcAddr< decltype(glXGetFBConfigAttrib) >("glXGetFBConfigAttrib");
// }

// void
// Skylium::__getExtensionList() {
//   int n;
//   glGetIntegerv(GL_NUM_EXTENSIONS, &n);
//   checkGLErrors(AT);
//   cout << n << "\n";
// 
//   for (int i = 0; i < n; i++) {
//     const char *temp = (const char*)glGetStringi(GL_EXTENSIONS, i);
//     __extensions.push_back(new string(temp));
//     log(LOW_PARAM, "Found an extension: %s", temp);
//   }
//   checkGLErrors(AT);
// 
//   
//   auto comparator = [](string *a, string *b) -> bool {
//     return (*a) < (*b);
//   };
//   
//   sort(__extensions.begin(), __extensions.end(), comparator);
// }

// void
// Skylium::__checkSupportedExtensions() {
//   static const string extensions[] = {
//     "GL_ARB_vertex_buffer_object",
//     "GL_ARB_vertex_array_object",
//     "GL_ARB_fragment_program",
//     "GL_ARB_fragment_shader",
//     "GL_ARB_vertex_shader",
//     "GL_ARB_vertex_program"
//   };
//   
//   for (const string& ext: extensions)
//     if (!isSupported(ext))
//       log(ERROR, "Extension %s is not supported on your system. Sorry, we have to terminate.", ext.c_str());
// }

// bool
// Skylium::__openXServerConnection() {
//   /* Open connection with the X server */
//   __GLXContext.display = XOpenDisplay(NULL);
//   if (__GLXContext.display == NULL)
//     log(ERROR, "Failed to connect with the X server.");
//   return true;
// }
// 
// bool
// Skylium::__getGLXVersion() {
//   /* Get the GLX version */
//   glXQueryVersion(__GLXContext.display, &__GLXContext.GLXVersionMajor, &__GLXContext.GLXVersionMinor);
//   log(PARAM, "GLX version: %i.%i", __GLXContext.GLXVersionMajor, __GLXContext.GLXVersionMinor);
//   
//   if (__GLXContext.GLXVersionMajor <= 1 && __GLXContext.GLXVersionMinor < 3)
//     log(ERROR, "GLX version too old. Exiting.");
//   return true;
// }

// bool
// Skylium::__getBestConfig(XVisualInfo*& _visualInfo, GLXFBConfig& _bestFbConfig) {
//   /* Look for the best configuration */
//   GLXFBConfig *fbConfigs;
//   int numConfigs = 0;
//   static int fbAttribs[] = {
//     GLX_RENDER_TYPE,  GLX_RGBA_BIT,
//     GLX_X_RENDERABLE,  True,
//     GLX_DRAWABLE_TYPE,  GLX_WINDOW_BIT,
//     GLX_DOUBLEBUFFER,  True,
//     GLX_X_VISUAL_TYPE,  GLX_TRUE_COLOR,
//     GLX_RED_SIZE,    sGlobalConfig::GL_RED_SIZE,
//     GLX_BLUE_SIZE,    sGlobalConfig::GL_BLUE_SIZE,
//     GLX_GREEN_SIZE,  sGlobalConfig::GL_GREEN_SIZE,
//     GLX_DEPTH_SIZE,  sGlobalConfig::GL_DEPTH_SIZE,
//     GLX_STENCIL_SIZE,  8,
//     GLX_ALPHA_SIZE,  8,
//     0 };
//   /* Gets new configuration */
//   fbConfigs = glXChooseFBConfig(__GLXContext.display, DefaultScreen(__GLXContext.display), fbAttribs, &numConfigs);
//   if (!fbConfigs)
//     log(ERROR, "Frame buffer config failed to be fetched. Exiting.");
//   
//   /* Choose the best configuration */
//   int bestFbConfigNum = -1, worstFbConfigNum = -1, bestNumSamp = -1, worstNumSamp = 999;
//   for (int i = 0; i < numConfigs; i++) {
//     XVisualInfo *vi = glXGetVisualFromFBConfig(__GLXContext.display, fbConfigs[i]);
//     if (vi) {
//       int sampBuf, samples;
//       glXGetFBConfigAttrib(__GLXContext.display, fbConfigs[i], GLX_SAMPLE_BUFFERS, &sampBuf);
//       glXGetFBConfigAttrib(__GLXContext.display, fbConfigs[i], GLX_SAMPLES, &samples);
//       
//       if ((bestFbConfigNum < 0 || sampBuf) && samples > bestNumSamp)
//         bestFbConfigNum = i, bestNumSamp = samples;
//       if (worstFbConfigNum < 0 || !sampBuf || samples < worstNumSamp)
//         worstFbConfigNum = i, worstNumSamp = samples;
//     }
//     XFree(vi);
//   }
//   
//   _bestFbConfig = fbConfigs[bestFbConfigNum];
//   XFree(fbConfigs);
//   
//   _visualInfo = glXGetVisualFromFBConfig(__GLXContext.display, _bestFbConfig);
//   
//   __GLXContext.winAttribs.event_mask =  ExposureMask
//               |  VisibilityChangeMask
//               |  KeyPressMask
//               |  PointerMotionMask
//               |  StructureNotifyMask
//               ;
//   __GLXContext.winAttribs.border_pixel = 0;
//   __GLXContext.winAttribs.background_pixmap = None;
//   __GLXContext.winAttribs.bit_gravity = StaticGravity;
//   __GLXContext.winAttribs.colormap = XCreateColormap(__GLXContext.display,
//                     RootWindow(__GLXContext.display, _visualInfo->screen),
//                     _visualInfo->visual,
//                     AllocNone
//                 );
//   return true;
// }
// 
// bool
// Skylium::__openXWindow(XVisualInfo* _visualInfo, const string& _windowName) {
//   GLint winmask = CWBorderPixel | CWBitGravity | CWEventMask | CWColormap;  
//   
//   __GLXContext.winWidth = DisplayWidth(__GLXContext.display,  DefaultScreen(__GLXContext.display));
//   __GLXContext.winHeight = DisplayHeight(__GLXContext.display,  DefaultScreen(__GLXContext.display));
//   
//   if (!sGlobalConfig::FULLSCREEN_RENDERING) {
//     __GLXContext.winWidth -= 50;
//     __GLXContext.winHeight -= 100;
//   }
//   
//   /* Create the new window */
//   __GLXContext.window = XCreateWindow(__GLXContext.display,
//                DefaultRootWindow(__GLXContext.display),
//                20, 20,
//                __GLXContext.winWidth, __GLXContext.winHeight, 0,
//                _visualInfo -> depth, InputOutput,
//                _visualInfo -> visual, winmask,
//                &__GLXContext.winAttribs
//           );
//   
//   if (sGlobalConfig::FULLSCREEN_RENDERING) {
//     XGrabKeyboard(__GLXContext.display, __GLXContext.window, True, GrabModeAsync, GrabModeAsync, CurrentTime);                                     
//     XGrabPointer(__GLXContext.display, __GLXContext.window, True, ButtonPressMask, GrabModeAsync,
//            GrabModeAsync, __GLXContext.window, None, CurrentTime); 
//   }
//   
//   XStoreName(__GLXContext.display, __GLXContext.window, _windowName.c_str());
//   XMapWindow(__GLXContext.display, __GLXContext.window);
//   
//   return true;
// }
// 
// bool
// Skylium::__createGLXContext(GLXFBConfig& _bestFbConfig) {
//   /* Create the new OpenGL context */;
//   log(PARAM, "Initializing OpenGL version %i.%i...", OPENGL_VERSION_MAJOR, OPENGL_VERSION_MINOR);
//   
//   if (glXCreateContextAttribsARB) {
//     GLint attribs[] = {
//       GLX_CONTEXT_MAJOR_VERSION_ARB,  OPENGL_VERSION_MAJOR,
//       GLX_CONTEXT_MINOR_VERSION_ARB,  OPENGL_VERSION_MINOR,
//       //GLX_CONTEXT_FLAGS_ARB,      GLX_CONTEXT_FORWARD_COMPATIBLE_BIT_ARB,
//       0 };
//   
//     __GLXContext.context = glXCreateContextAttribsARB(__GLXContext.display, _bestFbConfig,
//                     0, True, attribs);  
//     XSync(__GLXContext.display, False);
//     checkGLErrors(AT);
//   } else {
//     log(WARN, "glXCreateContextAttribsARB() not found. Older function in use.");
//     __GLXContext.context = glXCreateNewContext(__GLXContext.display, _bestFbConfig, GLX_RGBA_TYPE, 0, True);
//   }
//   
//   glXMakeCurrent(__GLXContext.display, __GLXContext.window, __GLXContext.context);
//   
//   return true;
// }

void
Skylium::__hideMousePointer() {
	// hides the mouse pointer
  // Cursor invisibleCursor;
  // Pixmap bitmapEmpty;
  // XColor black;
  // static char noData[] = { 0,0,0,0,0,0,0,0 };
  // black.red = black.green = black.blue = 0;
  // 
  // bitmapEmpty = XCreateBitmapFromData(__GLXContext.display, __GLXContext.window, noData, 8, 8);
  // invisibleCursor = XCreatePixmapCursor(__GLXContext.display, bitmapEmpty, bitmapEmpty, &black, &black, 0, 0);
  // XDefineCursor(__GLXContext.display, __GLXContext.window, invisibleCursor);
  // 
  // XFreeCursor(__GLXContext.display, invisibleCursor);
}

void
Skylium::__destroyContextAndWindow() {
  // glXMakeCurrent(__GLXContext.display, None, NULL);
  // glXDestroyContext(__GLXContext.display, __GLXContext.context);
  // __GLXContext.context = NULL;
  // 
  // XDestroyWindow(__GLXContext.display, __GLXContext.window);
  // __GLXContext.window = (Window)NULL;
  // 
  // XCloseDisplay(__GLXContext.display);
  // __GLXContext.display = 0;
}

bool
Skylium::__fileExists(const string &_fileName) {
	struct stat buf;
	if (stat(_fileName.c_str(), &buf) == 0)
		return 1;
	else
		return 0;
}
