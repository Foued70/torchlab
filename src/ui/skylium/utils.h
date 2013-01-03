/*
          _   _ _      _    
     _  _| |_(_) |___ | |_  
    | || |  _| | (_-<_| ' \ 
     \_,_|\__|_|_/__(_)_||_|
                        

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


#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <cstdarg>


#include "opengl.h"
#include "config.h"
#include "defines.h"

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define AT __FILE__ " : " TOSTRING(__LINE__)

enum LogType {
	CONSTRUCTOR,
	DESTRUCTOR,
	PARAM,
	LOW_PARAM,
	SHADER,
	BUFFER,
	WARN,
	ERROR
};

/* Some helpful functions */


template < typename T >
inline T string2T(const std::string &_s) {
	T temp;
	std::istringstream ss(_s);
	ss >> temp;
	return temp;
}

template < typename T >
inline std::string T2String(const T &_orig) {
	std::stringstream ss;
	std::string output;
	ss << _orig;
	ss >> output;
	return output;
}

// template< typename T >
// inline T getProcAddr(const std::string &_procName) {
//   T temp = (T)glXGetProcAddress((GLubyte*)_procName.c_str());
//   if (temp == (T)NULL) {
//     if ((sGlobalConfig::DEBUGGING & D_WARNINGS) == D_WARNINGS)
//       std::cout << LOG_WARN << "Error getting the \"" << _procName << "\" function pointer." <<
//         LOG_WARN << "Segfault is expected.";
//   }
//   return temp;    
// }

void explode(const std::string&, char, std::vector< std::string >&);

void checkGLErrors(const std::string&);

std::string getErrorString(GLenum);

void log(LogType, const std::string&, ...);

#endif // UTILS_H
