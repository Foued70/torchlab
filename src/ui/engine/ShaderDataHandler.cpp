#include "ShaderDataHandler.h"

#include "MatricesManager.h"
#include "Shader.h"

#include "config.h"
#include "utils.h"

using namespace std;

ShaderDataHandler::ShaderDataHandler() :
		__matrices(MatricesManager::GetSingleton()),
		__isStreamOpen(false) {
	log(CONSTRUCTOR, "ShaderDataHandler constructed.");
}

void
ShaderDataHandler::updateData(const string& _uniformName, const sVector2D& _value) {
	if (__isStreamOpen) {
		__activeShader->setUniformFloat(_uniformName, _value);
	} else {
		auto result = __2Dvectors.find(_uniformName);
		if (result == __2Dvectors.end())
			__2Dvectors.insert(make_pair(_uniformName, _value));
		else
			result -> second = _value;
	}
}

void
ShaderDataHandler::updateData(const string& _uniformName, const sVector3D& _value) {
	if (__isStreamOpen) {
		__activeShader->setUniformFloat(_uniformName, _value);
	} else {
		auto result = __3Dvectors.find(_uniformName);
		if (result == __3Dvectors.end())
			__3Dvectors.insert(make_pair(_uniformName, _value));
		else
			result -> second = _value;
	}
}

void
ShaderDataHandler::updateData(const string& _uniformName, const sVector4D& _value) {
	if (__isStreamOpen) {
		__activeShader->setUniformFloat(_uniformName, _value);
	} else {
		auto result = __4Dvectors.find(_uniformName);
		if (result == __4Dvectors.end())
			__4Dvectors.insert(make_pair(_uniformName, _value));
		else
			result -> second = _value;
	}
}

void
ShaderDataHandler::updateData(const string& _uniformName, GLfloat _value) {
	if (__isStreamOpen) {
		__activeShader->setUniformFloat(_uniformName, _value);
	} else {
		auto result = __values.find(_uniformName);
		if (result == __values.end())
			__values.insert(make_pair(_uniformName, _value));
		else
			result -> second = _value;
	}
}

void
ShaderDataHandler::updateSampler2D(const string& _uniformName, GLint _sampler) {
	if (__isStreamOpen) {
		__activeShader->setUniformInt(_uniformName, _sampler);
	} else {
		auto result = __textures.find(_uniformName);
		if (result == __textures.end())
			__textures.insert(make_pair(_uniformName, _sampler));
		else
			result -> second = _sampler;
	}
}

void
ShaderDataHandler::openStream(const Shader* _shader) {
	__isStreamOpen = true;
	__sendDataToShader(_shader);
	__activeShader = _shader;
}

void
ShaderDataHandler::__sendDataToShader(const Shader* _shader) {
	// first, send matrices
	_shader->setMatrixFloat("sModelViewMatrix",	__matrices.getModelViewMatrix());
	_shader->setMatrixFloat("sProjectionMatrix",	__matrices.getProjectionMatrix());
	_shader->setMatrixFloat("sNormalMatrix",	__matrices.getNormalMatrix());
	_shader->setMatrixFloat("sModelViewProjectionMatrix",
			__matrices.getProjectionMatrix() * __matrices.getModelViewMatrix()
		);
	
	// then send vectors
	for (auto it = __2Dvectors.begin(); it != __2Dvectors.end(); ++it)
		_shader->setUniformFloat(it -> first, it -> second);
	
	for (auto it = __3Dvectors.begin(); it != __3Dvectors.end(); ++it)
		_shader->setUniformFloat(it -> first, it -> second);
	
	for (auto it = __4Dvectors.begin(); it != __4Dvectors.end(); ++it)
		_shader->setUniformFloat(it -> first, it -> second);
	
	// and then single values
	for (auto it = __values.begin(); it != __values.end(); ++it)
		_shader->setUniformFloat(it -> first, it -> second);
	
	for (auto it = __textures.begin(); it != __textures.end(); ++it)
		_shader->setUniformInt(it -> first, it -> second);
	
	checkGLErrors(AT);
}
