#include "utils.h"
#include "opengl.h"

#define LOG_INFO "\e[36m(II)\e[0m "
#define LOG_ERROR "\e[31m(EE)\e[0m "
#define LOG_WARN "\e[33m(WW)\e[0m "

using namespace std;

unsigned	sGlobalConfig::DEBUGGING = D_ERRORS | D_WARNINGS | D_EVERYTHING;
bool sGlobalConfig::CREATE_MIPMAPS = true;


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

void checkGLErrors(const string &_at) {
	GLenum err = glGetError(); // fetch errors
	while (err != GL_NO_ERROR) {
		if ((sGlobalConfig::DEBUGGING & D_WARNINGS) == D_WARNINGS) {
			cout << LOG_ERROR << "OpenGL error (" << err << "): " << getErrorString(err)
				<< "\n\tAt: " << _at << "\n";
			cout.flush();
		}
		err = glGetError();
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

	cout.flush();
}

/*
std::string 
readResourceText(const std::string& _filename) {
  std::ifstream textFile;
  textFile.open(_filename.c_str(), ifstream::in);
  std::stringstream buffer;
  buffer << textFile.rdbuf();
  textFile.close();
  return buffer.str();
}
*/




