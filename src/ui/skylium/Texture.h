/*
     _____        _                  _    
    |_   _|____ _| |_ _  _ _ _ ___  | |_  
      | |/ -_) \ /  _| || | '_/ -_)_| ' \ 
      |_|\___/_\_\\__|\_,_|_| \___(_)_||_|
                                
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


#ifndef TEXTURE_H
#define TEXTURE_H

#include "opengl.h"
#include <string>

class TextureManager;
class ShaderDataHandler;

enum Mode {
	MODE_TEXTURE,
	MODE_NORMAL_MAP,
	MODE_SPECULAR_MAP
};

class Texture {

public:
	
	/**
	 * Default ctor that gets the texture file's location.
	 * Ctor generates the texture and puts itself into TextureManager's
	 * vector. The name is the file name without an extension.
	 * @param fileName Location of file with the texture.
	 */
	Texture(const std::string&, Mode = MODE_TEXTURE);
	
	/**
	 * Destructor destroys the texture and removes it from the GPU's buffer.
	 */
	virtual ~Texture();
	
	/**
	 * Binds the texture and sets the pipeline parameters.
	 */
	void setTexture(unsigned);
	void unsetTexture(unsigned);
	
	/**
	 * Parses the file name and gives back the texture name.
	 * @param fileName Name of the file with texture.
	 * @return Name of the texture, if @fileName was texture's source.
	 */
	static std::string getName(const std::string&);

	/* Name of the texture */
	std::string name;
	
private:
	
	/**
	 * Checks if file exists.
	 */
	bool __fileExists(const std::string&);
	
	/**
	 * Loads the texture.
	 * @param fileName Name of the file.
	 * @param texturePtr Pointer to OpenGL texture.
	 * @return Texture.
	 */
	GLuint __loadTexture(const std::string&);
	
	/* Pointer to GL's texture */
	GLuint __texture;
	
	/* Texture type - 
		GL_TEXTURE_1D,
		GL_TEXTURE_2D,
		GL_TEXTURE_3D,
		GL_TEXTURE_CUBE_MAP.
 		GL_TEXTURE_2D by default */
	GLenum __type;
	
	/* Texture wrapping - GL_CLAMP,
		GL_CLAMP_TO_BORDER,
		GL_CLAMP_TO_EDGE,
		GL_MIRRORED_REPEAT,
		GL_REPEAT,
		GL_CLAMP.
		GL_CLAMP_TO_BORDER by default */
	GLenum __wrapping;
	
	/* Location of the texture file - may be useful */
	std::string __file;
	
	/* Channels, default 4 (RGBA) */
	GLint __channels;
	
	Mode __mode;
	
	/* TM's instance */
	TextureManager * __boss;
	
	ShaderDataHandler& __shaders;
};

#endif // TEXTURE_H
