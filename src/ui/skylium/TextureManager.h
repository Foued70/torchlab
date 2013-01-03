/*
     _____        _                __  __                              _    
    |_   _|____ _| |_ _  _ _ _ ___|  \/  |__ _ _ _  __ _ __ _ ___ _ _ | |_  
      | |/ -_) \ /  _| || | '_/ -_) |\/| / _` | ' \/ _` / _` / -_) '_|| ' \ 
      |_|\___/_\_\\__|\_,_|_| \___|_|  |_\__,_|_||_\__,_\__, \___|_|(_)_||_|
                                                        |___/          
                                                        
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
