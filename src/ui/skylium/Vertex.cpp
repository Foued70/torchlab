/*
    __   __       _                          
    \ \ / /__ _ _| |_ _____ __  __ _ __ _ __ 
     \ V / -_) '_|  _/ -_) \ /_/ _| '_ \ '_ \
      \_/\___|_|  \__\___/_\_(_)__| .__/ .__/
                                  |_|  |_|   
                                  
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

#include "opengl.h"
#include "Vertex.h"

Position::Position() :
		x(0),
		y(0),
		z(0),
		w(1) {}

Position::Position(const Position &_orig) :
		x(_orig.x),
		y(_orig.y),
		z(_orig.z),
		w(1) {}

Position::Position(GLfloat _x, GLfloat _y, GLfloat _z) :
		x(_x),
		y(_y),
		z(_z),
		w(1) {}
		
TexCoords::TexCoords() :
		u(0),
		v(0) {}

TexCoords::TexCoords(const TexCoords &_orig) :
		u(_orig.u),
		v(_orig.v) {}

TexCoords::TexCoords(GLfloat _u, GLfloat _v) :
		u(_u),
		v(_v) {}

Normal::Normal() :
		x(0),
		y(0),
		z(0) {}

Normal::Normal(const Normal &_orig) :
		x(_orig.x),
		y(_orig.y),
		z(_orig.z) {}

Normal::Normal(GLfloat _x, GLfloat _y, GLfloat _z) :
		x(_x),
		y(_y),
		z(_z) {}

Vertex::Vertex() : 
		vertexPosition(),
		textureCoords(),
		normalVector()
			{}

Vertex::Vertex(const Position &_p, const TexCoords &_t, const Normal &_n) :
		vertexPosition(_p),
		textureCoords(_t),
		normalVector(_n)
			{}
