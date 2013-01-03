/*
    __   __       _             _    
    \ \ / /__ _ _| |_ _____ __ | |_  
     \ V / -_) '_|  _/ -_) \ /_| ' \ 
      \_/\___|_|  \__\___/_\_(_)_||_|
                      
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
