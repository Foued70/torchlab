#ifndef TEXTUREMANAGER_H
#define TEXTUREMANAGER_H

#include <string>
#include <unordered_map>

#include "Singleton.h"

class Texture;

class TextureManager : public Singleton< TextureManager > {

public:
	
	/**
	 * Default ctor.
	 */
	TextureManager();
	
	/**
	 * Destructor destroys all textures.
	 */
	virtual ~TextureManager();

	/**
	 * Lets get the texture by its name.
	 * @param name Name of the texture to be found.
	 * @return Pointer to this texture.
	 */
	Texture * getTextureByName(const std::string&);
	
	/**
	 * Puts the texture into the map.
	 * @param texture Pointer to the texture.
	 */
	void insert(Texture*);

private:
	
	/* Textures vector */
	std::unordered_map< std::string, Texture* > __texturesMap;
};

#endif // TEXTUREMANAGER_H
