/*
     _____ _                _    
    |_   _(_)_ __  ___ _ _ | |_  
      | | | | '  \/ -_) '_|| ' \ 
      |_| |_|_|_|_\___|_|(_)_||_|

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


#ifndef TIMER_H
#define TIMER_H

#include <iostream>

enum {
	SECONDS			= 1,
	MICROSECONDS
};

/* Low-level timer, ~10 microseconds */
class Timer {
	
public:
	/**
	 * Default ctor.
      */
	Timer();
	
	/**
	 * Asks the system for the actual time, returns it as long long int.
	 * @param clock Precision of the time returned - SECONDS or MICROSECONDS.
	 * @return The time.
	 */
	long long int update(unsigned);
	
	/**
	 * Checks if from the last time the function was called, the time specified
	 * in the argument has gone or not.
	 * @param howMuch How much time?
	 * @param clock Precision - SECONDS or MICROSECONDS.
	 * @return True if the time is up, otherwise false.
	 */
	bool passed(const long int&, unsigned);
	
private:
	
	struct timeval __tv;
	
	time_t __curtime;
	
	long long int __temp;

};


#endif // TIMER_H
