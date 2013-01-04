#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <cstdarg>


#include "opengl.h"
#include "config.h"

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

void explode(const std::string&, char, std::vector< std::string >&);

void checkGLErrors(const std::string&);

std::string getErrorString(GLenum);

void log(LogType, const std::string&, ...);

#endif // UTILS_H
