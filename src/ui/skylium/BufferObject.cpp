/*
    ___       __  __          ___  _     _        _                  
   | _ )_  _ / _|/ _|___ _ _ / _ \| |__ (_)___ __| |_   __ _ __ _ __ 
   | _ \ || |  _|  _/ -_) '_| (_) | '_ \| / -_) _|  _|_/ _| '_ \ '_ \
   |___/\_,_|_| |_| \___|_|  \___/|_.__// \___\__|\__(_)__| .__/ .__/
                                      |__/                |_|  |_|   
   
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


#include "BufferObject.h"

using namespace std;

BufferObject::BufferObject() :
		vaoID(0),
		vboID({{0, GL_ELEMENT_ARRAY_BUFFER, 0, 0, false}, {0, GL_ARRAY_BUFFER, 0, 0, false}}),
		__gpu(GPUMemory::GetSingleton()) {}

