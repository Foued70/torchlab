/*
     ___ _           _     _              _    
    / __(_)_ _  __ _| |___| |_ ___ _ _   | |_  
    \__ \ | ' \/ _` | / -_)  _/ _ \ ' \ _| ' \ 
    |___/_|_||_\__, |_\___|\__\___/_||_(_)_||_|
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


#ifndef SINGLETON_H
#define SINGLETON_H

#include <assert.h>

template < typename T >
class Singleton {
private:
        static T* ms_Singleton;

public:
        Singleton(void) {
                assert(!ms_Singleton);

                intptr_t offset = (intptr_t)(T*)1 - (intptr_t)(Singleton *)(T*)1;
                ms_Singleton = (T*)((intptr_t)this + offset);
        }

        ~Singleton(void) {
                assert(ms_Singleton);
                ms_Singleton = 0;
        }

        static T& GetSingleton(void) {
                assert(ms_Singleton);
                return *ms_Singleton;
        }

        static T* GetSingletonPtr(void) {
                return ms_Singleton;
        }
};

template < typename T > T* Singleton <T>::ms_Singleton = 0;
#endif  /* SINGLETON_H */
