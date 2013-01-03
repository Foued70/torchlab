/*
     _____        _                __  __                                      
    |_   _|____ _| |_ _  _ _ _ ___|  \/  |__ _ _ _  __ _ __ _ ___ _ _  __ _ __ _ __ 
      | |/ -_) \ /  _| || | '_/ -_) |\/| / _` | ' \/ _` / _` / -_) '_|/ _| '_ \ '_ \
      |_|\___/_\_\\__|\_,_|_| \___|_|  |_\__,_|_||_\__,_\__, \___|_|(_)__| .__/ .__/
                                                        |___/            |_|  |_|   
      
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


#include "TextureManager.h"
#include "Texture.h"

#include "Skylium.h"

#include "defines.h"
#include "config.h"
#include "utils.h"

using namespace std;

TextureManager::TextureManager() {
	log(CONSTRUCTOR, "TextureManager constructed.");
}

TextureManager::~TextureManager() {
	for (auto it = __texturesMap.begin(); it != __texturesMap.end(); ++it)
		delete it -> second;
	log(DESTRUCTOR, "TextureManager destructed.");
}

Texture *
TextureManager::getTextureByName(const string &_name) {
	auto it = __texturesMap.find(_name);
	if (it == __texturesMap.end())
		return NULL;
	return it -> second;
}

void
TextureManager::insert(Texture *_tex) {
	if (_tex == NULL)
		return;
	__texturesMap.insert(make_pair(_tex -> name, _tex));
}