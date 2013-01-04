#include <iostream>
#include <string>
#include <fstream>

#include <sys/stat.h>

#include "Shader.h"

#include "Vectors.h"
#include "Skylium.h"

#include "config.h"
#include "utils.h"
#include "../qutil.h"

using namespace std;

static const unsigned int MAX_LOG_SIZE = 4096;


Shader::Shader(const string& _fileName) : 
    __name(_fileName),
		__vertFile(":/shaders/" + _fileName + ".vert"),
		__fragFile(":/shaders/" + _fileName + ".frag"),
		__vertCode(""),
		__fragCode(""),
		__isRunning(false),
		__isCompiled(false) {
	
	__vertexShader = glCreateShader(GL_VERTEX_SHADER);
	__fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	
	log(CONSTRUCTOR, "Shader (\"%s\", \"%s\") constructed.", __vertFile.c_str(), __fragFile.c_str());
}

Shader::Shader(const string& _vertexCode, const string& _fragmentCode) :
    __name("anon"),
		__vertFile(""),
		__fragFile(""),
		__vertCode(_vertexCode),
		__fragCode(_fragmentCode),
		__isRunning(false),
		__isCompiled(false) {
			
	__vertexShader = glCreateShader(GL_VERTEX_SHADER);
	__fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	
	log(CONSTRUCTOR, "Shader constructed (from source code).");
}

Shader::~Shader() {
	glDetachShader(__shaderProgram, __vertexShader);
	glDetachShader(__shaderProgram, __fragmentShader);

	glDeleteProgram(__shaderProgram);

	glDeleteShader(__vertexShader);
	glDeleteShader(__fragmentShader);
	checkGLErrors(AT);
	
	log(DESTRUCTOR, "Shader (\"%s\", \"%s\") destructed.", __vertFile.c_str(), __fragFile.c_str());
}

bool
Shader::make(GLuint _var1, const string& _param1,
		   GLuint _var2, const string& _param2,
		   GLuint _var3, const string& _param3) {
	const char *vert;
	const char *frag;
	
	static const string HEADER = 
		"#version 150\n"
		"struct sMaterialParams {"
		" vec4 emission;"
		" vec4 ambient;"
		" vec4 diffuse;"
		" vec4 specular;"
		" float shininess;"
		"};"
		
		"struct sLightParams {"
		" vec4 ambient;"
		" vec4 diffuse;"
		" vec4 specular;"
		" vec4 position;"
		" float constantAttenuation;"
		" float linearAttenuation;"
		" float quadraticAttenuation;"
		"};"
		
		"struct sLightModelParameters {"
		" vec4 ambient;"
		"};"
		
		"uniform vec4 sDefColor;"
		"uniform mat4 sModelViewMatrix;"
		"uniform mat4 sProjectionMatrix;"
		"uniform mat4 sModelViewProjectionMatrix;"
		"uniform mat3 sNormalMatrix;"
		"uniform sMaterialParams sFrontMaterial;"
		"uniform sLightParams sLightSource[7];"
		"uniform sLightModelParameters sLightModel;\n";
	
	static const string VERTEX_HEADER =
		"in vec4 sVertex;"
		"in vec3 sNormal;"
		"in vec2 sTexCoords;"
		"smooth out vec2 sVaryingTexCoords;\n";
	
	static const string FRAGMENT_HEADER =
		"out vec4 sFragColor;"
		"smooth in vec2 sVaryingTexCoords;"
		"uniform sampler2D textureUnit;"
		"uniform sampler2D normalMap;"
		"uniform sampler2D specularMap;";
	
	if (!__vertFile.empty() && !__fragFile.empty()) {
    __vertCode = HEADER + VERTEX_HEADER + readResourceText(__vertFile.c_str());
    __fragCode = HEADER + FRAGMENT_HEADER + readResourceText(__fragFile.c_str());
	} else if (!__vertCode.empty() && !__fragCode.empty()) {
		__vertCode = HEADER + VERTEX_HEADER + __vertCode;
		__fragCode = HEADER + FRAGMENT_HEADER + __fragCode;
	} else {
		log(ERROR, "Shader source could not be obtained!");
	}

	vert = __vertCode.c_str();
	frag = __fragCode.c_str();

	GLint vlength = __vertCode.length();
	GLint flength = __fragCode.length();
	
	glShaderSource(__vertexShader, 1, (const GLchar**)&vert, &vlength);
	checkGLErrors(AT);
	glShaderSource(__fragmentShader, 1, (const GLchar**)&frag, &flength);
	checkGLErrors(AT);

	int result;

	glCompileShader(__vertexShader);
	checkGLErrors(AT);
	glGetShaderiv(__vertexShader, GL_COMPILE_STATUS, &result);
	checkGLErrors(AT);
	if (!result) {
		char msg[MAX_LOG_SIZE];
		glGetShaderInfoLog(__vertexShader, MAX_LOG_SIZE, NULL, msg);
		
		log(ERROR, "Error compiling vertex shader! Compilation log:\n%s", msg);
	}

	glCompileShader(__fragmentShader);
	checkGLErrors(AT);
	glGetShaderiv(__fragmentShader, GL_COMPILE_STATUS, &result);
	checkGLErrors(AT);
	if (!result) {
		char msg[MAX_LOG_SIZE];
		glGetShaderInfoLog(__fragmentShader, MAX_LOG_SIZE, NULL, msg);
		
		log(ERROR, "Error compiling fragment shader! Compilation log:\n%s", msg);
	}

	__shaderProgram = glCreateProgram();
	checkGLErrors(AT);
	if (!__shaderProgram)
		log(ERROR, "Error creating shader program!");
	
	glAttachShader(__shaderProgram, __vertexShader);
	checkGLErrors(AT);
	glAttachShader(__shaderProgram, __fragmentShader);
	checkGLErrors(AT);
	
	if (!_param1.empty())
		glBindAttribLocation(__shaderProgram, _var1, _param1.c_str());
	if (!_param2.empty())
		glBindAttribLocation(__shaderProgram, _var2, _param2.c_str());
	if (!_param3.empty())
		glBindAttribLocation(__shaderProgram, _var3, _param3.c_str());

	glLinkProgram(__shaderProgram);
	checkGLErrors(AT);

	glGetProgramiv(__shaderProgram, GL_LINK_STATUS, &result);
	checkGLErrors(AT);
	if (!result) {
		char msg[MAX_LOG_SIZE];
		glGetProgramInfoLog(__shaderProgram, MAX_LOG_SIZE, NULL, msg);
		
		log(ERROR, "Error linking shader program! Linking log:\n%s", msg);
	}
	checkGLErrors(AT);
	
	if (glIsProgram(__shaderProgram) == GL_FALSE)
		log(ERROR, "Error creating shader program!");

	log(SHADER, "Shader compiled. No errors reported.");
	
	__isCompiled = true;
	
	return true;

}

void
Shader::toggle() {
	if (!__isRunning) {
		glUseProgram(__shaderProgram);
		checkGLErrors(AT);
		__isRunning = true;
	} else {
		glUseProgram(0);
		checkGLErrors(AT);
		__isRunning = false;
	}
}

void
Shader::bind(Object *_dest) {
	if (!__isCompiled)
		make();
	
	_dest -> __shader = this;
	
	log(SHADER, "Shader %s bound to %s.", __name.c_str(), _dest -> name.c_str());
}

void
Shader::unbind(Object *_dest) {
	_dest -> __shader = NULL;
}

bool
Shader::isBound(Object *_dest) {
	if (_dest -> __shader == NULL)
		return false;
	else
		return true;
}

void
Shader::setUniformFloat(const string& _name, const sVectorBase< GLfloat >& _params) const {
	GLint location = glGetUniformLocation(__shaderProgram, _name.c_str());
	checkGLErrors(AT);
	
	switch (_params.size()) {
		case 2:
			glUniform2f(location, _params[0], _params[1]);
		case 3:
			glUniform3f(location, _params[0], _params[1], _params[2]);
		case 4:
			glUniform4f(location, _params[0], _params[1], _params[2], _params[3]);
	}
	checkGLErrors(AT);
}

void
Shader::setUniformFloat(const string& _name, GLfloat _param) const {
	glUniform1f(glGetUniformLocation(__shaderProgram, _name.c_str()), _param);
	checkGLErrors(AT);
}

void
Shader::setUniformInt(const string& _name, GLint _value) const {
	glUniform1i(glGetUniformLocation(__shaderProgram, _name.c_str()), _value);
	checkGLErrors(AT);
}

void
Shader::setMatrixFloat(const string& _name, const sMat16& _matrix) const {
  // log(PARAM, "setMatrix %s", _name.c_str());
  // cout << _matrix;
	glUniformMatrix4fv(glGetUniformLocation(__shaderProgram, _name.c_str()), 1, GL_FALSE, _matrix);
	checkGLErrors(AT);
}

void
Shader::setMatrixFloat(const string& _name, const sMat9& _matrix) const {
  // log(PARAM, "setMatrix %s", _name.c_str());
  // cout << _matrix;
	glUniformMatrix3fv(glGetUniformLocation(__shaderProgram, _name.c_str()), 1, GL_FALSE, _matrix);
	checkGLErrors(AT);
}

bool
Shader::__fileExists(const std::string &_fileName) {
	struct stat buf;
	if (stat(_fileName.c_str(), &buf) == 0)
		return 1;
	else
		return 0;
}