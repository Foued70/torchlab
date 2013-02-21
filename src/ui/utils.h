#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <cstdarg>

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
  static bool CREATE_MIPMAPS;
};

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

void log(LogType, const std::string&, ...);

#endif // UTILS_H
