#ifndef VERTEX_H
#define VERTEX_H

typedef struct Position {
	GLfloat x;
	GLfloat y;
	GLfloat z;
	GLfloat w;
	
	Position();
	Position(const Position&);
	Position(GLfloat, GLfloat, GLfloat);
} Position;

typedef struct TexCoords {
	GLfloat u;
	GLfloat v;
	
	TexCoords();
	TexCoords(const TexCoords&);
	TexCoords(GLfloat, GLfloat);
} TexCoords;

typedef struct Normal {
	GLfloat x;
	GLfloat y;
	GLfloat z;
	
	Normal();
	Normal(const Normal&);
	Normal(GLfloat, GLfloat, GLfloat);
} Normal;


class Vertex {

public:
	Vertex();
	Vertex(const Position&, const TexCoords&, const Normal&);

	Position	vertexPosition;
	TexCoords	textureCoords;
	Normal	normalVector;
	
private:
	char		__align[28];
};

#endif // VERTEX_H
