#ifndef MATRICES_H
#define MATRICES_H

#include <iostream>
#include <cstring>

#include <assert.h>

#include "Vectors.h"
#include "utils.h"
#include <string>

/*
 * This matrix is a column-order matrix, as OpenGL prefers.
 * It is continuous in memory.
 */
template < typename T, int N >
class sMatrix {
	
private:
	T	__data[N * N];
	
public:
	// default ctor
	sMatrix() {
		memset(__data, 0, N * N * sizeof(T));
	}
	
	// Let GCC make the copy ctor
	sMatrix(const sMatrix< T, N >&) = default;
	
	// and the operator =
	sMatrix< T, N> & operator =(const sMatrix< T, N >&) = default;
	
	// pointer
	inline operator T*() {
		return __data;
	}
	
	inline operator const T*() const {
		return __data;
	}
	
	inline T& operator [](int _pos) {
		assert(_pos < N * N);
		return __data[_pos];
	}
	
	inline const T& operator [](int _pos) const {
		assert(_pos < N * N);
		return __data[_pos];
	}
	
	// not like in minor in maths in general - we count from 0.
	// i - row, j - column
	inline T& at(int _i, int _j) {
		assert((_j < N) && (_i < N));
		return __data[_j * N + _i];
	}
	
	inline const T& at(int _i, int _j) const {
		assert((_j < N) && (_i < N));
		return __data[_j * N + _i];
	}
	
	void loadIdentity() {
		memset(__data, 0, N * N * sizeof(T));
		for (int i = 0; i < N; ++i)
			__data[i * N + i] = 1;
	}
	
	inline T determinant() {
		if (N == 1) {
			return __data[0];
    }
		if (N == 2) {
			return (__data[0] * __data[3]) - (__data[2] * __data[1]);
    }
		if (N == 3) {
			return determinant3x3();
    }
		if (N == 4) {
  		int det = 0;
  		for (int j = 0; j < 4; ++j) {
  			if (j % 2 == 0) 
  				det += at(0, j) * getMinor(0, j).determinant3x3();
  			else
  				det -= at(0, j) * getMinor(0, j).determinant3x3();
  		}
  		return det;
		}
	}
  
  /* This exists to prevent determinant from being recursive, which requires getMinor<N-1> to decrement
     forever in the compiler and fail to compile */
  inline T determinant3x3() {
		return ((__data[0] * __data[4] * __data[8]) + (__data[3] * __data[7] * __data[2]) + (__data[6] * __data[1] * __data[5]))
			- ((__data[6] * __data[4] * __data[2]) + (__data[3] * __data[1] * __data[8]) + (__data[0] * __data[7] * __data[5]));
  }
	
	/* Counting form 0 - not like in minors in general */
	inline sMatrix< T, N - 1 > getMinor(int _i, int _j) {
		sMatrix< T, N - 1 > result;
		int idx = 0;
		
		for (int j = 0; j < N; ++j) {
			if (j == _j)
				continue;
			for (int i = 0; i < N; ++i) {
				if (i == _i)
					continue;
				result[idx++] = at(i, j);
			}
		}
		
		return result;
	}
	
	friend std::ostream& operator <<(std::ostream& _result, const sMatrix< T, N >& _orig) {
		for (int i = 0; i < N; ++i) {
			for (int j = 0; j < N; ++j) {
				if (_orig.at(i, j) >= 0) _result << " ";
				_result << _orig.at(i, j) << "\t";
			}
			_result << "\n";
		}
		return _result;
	}
	
	inline friend sMatrix< T, N > operator *(const sMatrix< T, N >& _a, const sMatrix< T, N >& _b) {
		sMatrix< T, N > result;
		for (int i = 0; i < N; ++i)
			for (int j = 0; j < N; ++j) 
				for (int idx = 0; idx < N; ++idx)
					result.at(i, j) += _a.at(i, idx) * _b.at(idx, j);
		
		return result;	
	}
	
	inline sMatrix< T, N >& operator *=(const sMatrix< T, N >& _b) {
		*this = *this * _b;
		return *this;
	}
	
	/**
	 * Sets the whole column in the matrix.
	 * @param c Column to be replaced, counting from 0.
	 * @param vector Vector with data.
	 * @param offset Which row to start from. Default 0.
	 */
	template < typename D >
	void setColumn(int _c, const sVectorBase< D >& _vector, unsigned _offset = 0) {
		assert(_c < N);
		int v = 0;
		for (unsigned i = _offset; i < _vector.size(); ++i, ++v)
			at(i, _c) = _vector[v];
	}
	
	/**
	 * Sets the while row in the matrix.
	 * @param r Row to be replaced, counting from 0.
	 * @param vector Vector with data.
	 * @param offset Which column to start from. Default 0.
	 */
	template < typename D >
	void setRow(int _r, const sVectorBase< D >& _vector, unsigned _offset = 0) {
		assert(_r < N);
		int v = 0;
		for (unsigned i = _offset; i < _vector.size(); ++i, ++v)
			at(_r, i) = _vector[v];
	}
	
	/**
	 * Calculates the matrix inversion.
	 * @return Inverted matrix.
	 */
	inline sMatrix< T, N > inversion() {
		sMatrix< T, N > result;
		double det = (T)1.0 / determinant();
		for (int j = 0; j < N; ++j) {
			for (int i = 0; i < N; ++i) {
				sMatrix< T, N - 1 > mMinor = getMinor(j, i);
				result.at(i, j) = det * mMinor.determinant();
				if ((i + j) % 2 == 1)
					result.at(i, j) = -result.at(i, j);
			}
		}
		
		return result;
	}
	
	inline void transpose() {
		for (int i = 0; i < N; ++i) {
			for (int j = i; j < N; ++j) {
				T temp = at(i, j);
				at(i, j) = at(j, i);
				at(j, i) = temp;
			}
		}
	}
  
  inline sVector< T, N > operator*(const sVector< T, N >& _vector) {
    sVector< T, N > result;
  	for (int i = 0; i < N; i++) {
      result[i] = 0;
  		for (int j = 0; j < N; j++) {
        result[i] += at(i, j) * _vector[j];
      }
    }
    return result;
  }
  
  void logAllComponents(const std::string& _name) {
	  for (int i = 0; i < N; ++i)
      for(int j = 0; j < N; ++j)
        log(PARAM, "%s[%d][%d]=%f",_name.c_str(), i, j, at(i, j));
  }	
};

typedef	sMatrix< float, 3 >	sMat9;
typedef	sMatrix< float, 4 >	sMat16;

#endif // MATRICES_H
