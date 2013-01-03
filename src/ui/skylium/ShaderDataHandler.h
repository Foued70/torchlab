/*
     ___ _            _         ___       _        _  _              _ _          _    
    / __| |_  __ _ __| |___ _ _|   \ __ _| |_ __ _| || |__ _ _ _  __| | |___ _ _ | |_  
    \__ \ ' \/ _` / _` / -_) '_| |) / _` |  _/ _` | __ / _` | ' \/ _` | / -_) '_|| ' \ 
    |___/_||_\__,_\__,_\___|_| |___/\__,_|\__\__,_|_||_\__,_|_||_\__,_|_\___|_|(_)_||_|
                                                                                   

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


#ifndef SHADERDATAHANDLER_H
#define SHADERDATAHANDLER_H

#include <iostream>
#include <map>
#include <string>

#include "opengl.h"
#include "Singleton.h"

#include "Vectors.h"
#include "Matrices.h"

class Shader;
class MatricesManager;

class ShaderDataHandler : public Singleton< ShaderDataHandler > {
public:
	/* Default ctor. */
     ShaderDataHandler();
	
	/**
	 * Updates some data.
	 * @param uniformName Name of the uniform in the shader - also name stored
	 * 				in ShaderDataHandler class.
	 * @param value Value of the uniform variable.
	 */
	void updateData(const std::string&, const sVector2D&);
	void updateData(const std::string&, const sVector3D&);
	void updateData(const std::string&, const sVector4D&);
	void updateData(const std::string&, GLfloat);
	void updateSampler2D(const std::string&, GLint);
	
	/**
	 * Opens the stream.
	 * @param shader Active shader.
	 */
	void openStream(const Shader*);
	
	/**
	 * Closes the stream.
	 */
	void closeStream() { __isStreamOpen = false; }

	
public:
	
	/**
	 * Sends all stored data to the shader.
	 * @param shader Active shader.
	 */
	void __sendDataToShader(const Shader*);
	
	std::map< std::string, sVector2D > __2Dvectors;
	
 	std::map< std::string, sVector3D > __3Dvectors;
	
	std::map< std::string, sVector4D > __4Dvectors;
	
	std::map< std::string, GLfloat > __values;
	
	std::map< std::string, GLuint > __textures;
	
	MatricesManager& __matrices;
	
	bool __isStreamOpen;
	
	const Shader* __activeShader;
	
};

#endif // SHADERDATAHANDLER_H
