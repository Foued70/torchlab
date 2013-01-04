#include "config.h"

unsigned	sGlobalConfig::DEBUGGING				= D_ERRORS | D_WARNINGS | D_EVERYTHING;
bool		sGlobalConfig::FULLSCREEN_RENDERING	= false;
bool		sGlobalConfig::HUD_EXISTS			= true;
bool		sGlobalConfig::MOUSE_VISIBLE			= true;
short	sGlobalConfig::GL_RED_SIZE			= 5;
short	sGlobalConfig::GL_GREEN_SIZE			= 5;
short	sGlobalConfig::GL_BLUE_SIZE			= 5;
short	sGlobalConfig::GL_DEPTH_SIZE			= 16;
bool		sGlobalConfig::CREATE_MIPMAPS			= true;