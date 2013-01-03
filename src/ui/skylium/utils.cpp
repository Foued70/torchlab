/*
          _   _ _                   
     _  _| |_(_) |___  __ _ __ _ __ 
    | || |  _| | (_-<_/ _| '_ \ '_ \
     \_,_|\__|_|_/__(_)__| .__/ .__/
                         |_|  |_|  
                     
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

#include "utils.h"
#include "config.h"

using namespace std;

void explode(const std::string &_text, char _delim, std::vector< std::string > &_dest) {
	_dest.clear();
	string temp = "";
	
	for (unsigned i = 0; i < _text.length(); i++) {
		if (_text[i] == _delim) {
			_dest.push_back(temp);
			temp = "";
		} else {
			temp += _text[i];
		}
	}
	if (temp != "")
		_dest.push_back(temp);
}

void checkGLErrors(const string &_at) {
	GLenum err = glGetError(); // fetch errors
	while (err != GL_NO_ERROR) {
		if ((sGlobalConfig::DEBUGGING & D_WARNINGS) == D_WARNINGS) {
			cout << "\n\e[31m(EE)\e[0m " << "OpenGL error (" << err << "): " << getErrorString(err)
				<< "\n\tAt: " << _at << "\n";
			cout.flush();
		}
		err = glGetError();
	}
}

string getErrorString(GLenum _err) {
	switch (_err) {
		case 0x0500:
			return "GL_INVALID_ENUM";
		case 0x0501:
			return "GL_INVALID_VALUE";
		case 0x0502:
			return "GL_INVALID_OPERATION";
		case 0x0503:
			return "GL_OUT_OF_MEMORY";
		case 0x0506:
			return "GL_INVALID_FRAMEBUFFER_OPERATION​";
		default:
			return "unknown error code";
	}
}

void log(LogType _type, const std::string& _out, ...) {
	va_list ap;
	va_start(ap, _out);
	
	string msg;
	
	for (unsigned i = 0; i < _out.length(); ++i) {
		switch (_out[i]) {
			case '%':
				switch (_out[i + 1]) {
					case 'i':
					case 'd':
						msg += T2String< int >(va_arg(ap, int));
						break;
					case 'f':
						msg += T2String< double >(va_arg(ap, double));
						break;
					case 's':
						msg += va_arg(ap, char*);
						break;
					case 'u':
						msg += T2String< unsigned >(va_arg(ap, unsigned));
						break;
				}
				i += 1;
				break;
			default:
				msg += _out[i];
		}
		
	}
	
	va_end(ap);
	
	switch (_type) {
		case CONSTRUCTOR:
			if (sGlobalConfig::DEBUGGING & D_CONSTRUCTORS)
				std::cout << LOG_INFO << msg << std::endl;
			break;
		case DESTRUCTOR:
			if (sGlobalConfig::DEBUGGING & D_DESTRUCTORS)
				std::cout << LOG_INFO << msg << std::endl;
			break;
		case PARAM:
			if (sGlobalConfig::DEBUGGING & D_PARAMS)
				std::cout << LOG_INFO << msg << std::endl;
			break;
		case LOW_PARAM:
			if (sGlobalConfig::DEBUGGING & D_ALL_PARAMS)
				std::cout << LOG_INFO << msg << std::endl;
			break;
		case SHADER:
			if (sGlobalConfig::DEBUGGING & D_SHADERS)
				std::cout << LOG_INFO << msg << std::endl;
			break;
		case BUFFER:
			if (sGlobalConfig::DEBUGGING & D_BUFFER)
				std::cout << LOG_INFO << msg << std::endl;
			break;
		case WARN:
			if (sGlobalConfig::DEBUGGING & D_WARNINGS)
				std::cout << LOG_WARN << msg << std::endl;
			break;
		case ERROR:
			if (sGlobalConfig::DEBUGGING & D_ERRORS)
				std::cout << LOG_ERROR << msg << std::endl;
			exit(1);
	}
}