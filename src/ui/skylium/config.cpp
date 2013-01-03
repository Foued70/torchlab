/*
                  __ _                     
     __ ___ _ _  / _(_)__ _   __ _ __ _ __ 
    / _/ _ \ ' \|  _| / _` |_/ _| '_ \ '_ \
    \__\___/_||_|_| |_\__, (_)__| .__/ .__/
                      |___/     |_|  |_|   
    
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