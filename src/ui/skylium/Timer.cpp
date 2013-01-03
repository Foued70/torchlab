/*
     _____ _                             
    |_   _(_)_ __  ___ _ _  __ _ __ _ __ 
      | | | | '  \/ -_) '_|/ _| '_ \ '_ \
      |_| |_|_|_|_\___|_|(_)__| .__/ .__/
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

#include <sys/time.h>

#include "Timer.h"

Timer::Timer() :
		__tv(),
		__curtime(0),
		__temp(0)
			{}

long long int
Timer::update(unsigned _clock) {
	gettimeofday(&__tv, NULL);

	__curtime = __tv.tv_sec;
	__curtime *= (1000 * 1000);
	if (_clock & MICROSECONDS) {
		__curtime += __tv.tv_usec;
	}
	return __curtime;
}

bool
Timer::passed(const long int &_howMuch, unsigned _clock) {
	long long temp = this -> update(_clock);
	bool result = false;
	
	if (temp - __temp > _howMuch) {
		result = true;
		__temp = temp;
	}
	return result;
	
}
