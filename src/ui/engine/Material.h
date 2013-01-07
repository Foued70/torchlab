#ifndef MATERIAL_H
#define MATERIAL_H

#include <string>
#include <vector>

#include "opengl.h"
#include "Vectors.h"

class Texture;
class ShaderDataHandler;

enum {
	MATERIAL_AMBIENT	= 1,
	MATERIAL_DIFFUSE	= 2,
	MATERIAL_SPECULAR	= 4
};

class Material {
	
public:
	
	/**
	 * Ctor gets the material's name. The rest is empty.
	 * @param name Material's name.
	 */
	Material(const std::string& = "");
	
	/**
	 * Copy ctor.
	 */
	Material(const Material&);
	
	/**
	 * Destructor sends some output.
	 */
	virtual ~Material();
	
	/**
	 * Sets chosen material's parameter.
	 * @param param Parameter's value.
	 * @param type Material's type to be set. Possible values:
	 *		&bump; MATERIAL_AMBIENT - mAmbient value;
	 *		&bump; MATERIAL_DIFFUSE - mDiffuse value;
	 *		&bump; MATERIAL_SPECULAR - mSpecular value.
	 */
	void loadMaterial(const sColor&, unsigned);
	
	/**
	 * Sets mAlpha parameter.
	 * @param alpha Transparency ([0; 1]).
	 */
	void loadAlpha(GLfloat);
	
	/**
	 * Sets shininess.
	 * @param shininess Shininess value.
	 */
	void loadShininess(GLfloat);
	
	/**
	 * Checks whether the particular material has any texture or not.
	 * @return True if has.
	 */
	bool hasAnyTexture() { return __textures.size() > 0; }
	
	/**
	 * Sets textures' parameters to be rendered.
	 * http://www.opengl.org/sdk/docs/man/xhtml/glBindTexture.xml
	 * http://www.opengl.org/sdk/docs/man/xhtml/glTexParameter.xml
	 */
	void setTextures();
	
	/**
	 * Sets materials' parameters to be rendered.
	 * http://www.opengl.org/sdk/docs/man/xhtml/glMaterial.xml
	 */
	void setMaterial();
	
	/**
	 * Calls glBindTexture(NULL) for each texture.
	 */
	void unsetTextures();
	
	/**
	 * Adds to __textures vector pointer to a texture.
	 * @param _tex Texture's pointer.
	 */
	void appendTexture(Texture*);
	
	/* Material's name */
	std::string name;
	
private:
	
	/* Material */
	sColor			__mAmbient;
	sColor			__mDiffuse;
	sColor			__mSpecular;
	sColor			__mEmission;
	GLfloat			__mAlpha;
	GLfloat			__mShininess;
	
	/* Textures */
	std::vector< Texture* >	__textures;
	
	ShaderDataHandler&	__shaders;

	
};

#endif // MATERIAL_H
