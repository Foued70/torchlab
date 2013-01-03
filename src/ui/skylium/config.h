/*
                  __ _        _    
     __ ___ _ _  / _(_)__ _  | |_  
    / _/ _ \ ' \|  _| / _` |_| ' \ 
    \__\___/_||_|_| |_\__, (_)_||_|
                      |___/  
    
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

#ifndef CONFIG_H
#define CONFIG_H

enum {
	D_NOTHING			= 0,
	D_CONSTRUCTORS		= 1,
	D_ALL_CONSTRUCTORS	= 2,
	D_DESTRUCTORS		= 4,
	D_PARAMS			= 8,
	D_ALL_PARAMS		= 16,
	D_SHADERS			= 32,
	D_BUFFER			= 64,
	D_WARNINGS		= 128,
	D_ERRORS			= 256,
	D_EVERYTHING		= 511
};

/* For more info, see ConfigFile wiki page. */
struct sGlobalConfig {
	
	/* Configures the output verbosity. */
	static unsigned	DEBUGGING;
	
	/* If true, Skylium renders on the fullscreen. */
	static bool		FULLSCREEN_RENDERING;
	
	/* If, false, hud is not shown */
	static bool		HUD_EXISTS;
	
	/* Mouse visibility, true or false. */
	static bool		MOUSE_VISIBLE;
	
	/* Minimum bytes for the Skylium to be started. */
	static short		GL_RED_SIZE;
	static short		GL_GREEN_SIZE;
	static short		GL_BLUE_SIZE;
	static short		GL_DEPTH_SIZE;
	
	/* If true, Skylium will generate mipmaps. */
	static bool		CREATE_MIPMAPS;
	
	
};

#endif