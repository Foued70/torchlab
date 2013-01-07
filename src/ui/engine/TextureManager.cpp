#include "TextureManager.h"
#include "Texture.h"

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