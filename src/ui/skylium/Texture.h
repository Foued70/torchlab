ifndef TEXTURE_H
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
