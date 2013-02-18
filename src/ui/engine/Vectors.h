#ifndef VECTORS_H
#define VECTORS_H

#include <cstring>
#include <cmath>
#include <initializer_list>

#include <assert.h>

#include <utils.h>

template < typename T >
class sVectorBase {
public:
	virtual const size_t size() const		= 0;
	virtual const T& operator [](int) const	= 0;
	virtual T& operator [](int) 			= 0;
};



template < typename T, int N >
class sVectorReferences {};

template < typename T >
class sVectorReferences< T, 2 > {
public:
	T&	x;
	T&	y;
	
	inline sVectorReferences(T* _t) : x(_t[0]), y(_t[1]) {}
	
};

template < typename T >
class sVectorReferences< T, 3 > {
public:
	T&	x;
	T&	y;
	T&	z;
	
	inline sVectorReferences(T* _t) : x(_t[0]), y(_t[1]), z(_t[2]) {}
};

template < typename T >
class sVectorReferences< T, 4 > {
public:
	T&	r;
	T&	g;
	T&	b;
	T&	a;
	
	inline sVectorReferences(T* _t) : r(_t[0]), g(_t[1]), b(_t[2]), a(_t[3]) {}
};






template < typename T, int N >
class sVector : public sVectorReferences< T, N >, public sVectorBase< T > {
private:
	T	__data[N];
	
public:
	/**
	 * Default ctor, sets all values to 0.
	 */
	inline sVector() : sVectorReferences< T, N >(__data) {
		memset(__data, 0, N * sizeof(T));
	}
	
	/**
	 * Initializer_list ctor.
	 */
	inline sVector(std::initializer_list< T > _i) : sVectorReferences< T, N >(__data) {
		assert(_i.size() == N);
		
		unsigned index = 0;
		for (auto it = _i.begin(); it != _i.end(); ++it) {
			__data[index] = (*it);
			++index;
		}
	}
  
  /**
  * Ctor that initializes all components as the same value.
  */
  inline sVector(const T& _component) : sVectorReferences< T, N >(__data) {
    for (short i = 0; i < N; ++i) {
      __data[i] = _component;
    }
  }
  
  /**
   * Copy ctor
   */
  inline sVector(const sVector& _orig) : sVectorReferences< T, N >(__data) {
    memcpy(__data, _orig, N * sizeof(T));
  }
	
	inline const size_t size() const { return N; }

		
			/*     OPERATORS     */

	inline const T& operator [](int _n) const { return __data[_n]; }
	inline T& operator [](int _n) { return __data[_n]; }
	
	inline operator const T*() const { return __data; }
	inline operator T*() { return __data; }
	
	inline bool operator ==(const sVector< T, N >& _orig) const {
		for (short i = 0; i < N; ++i)
			if (_orig[i] != __data[i])
				return false;
		return true;
	}
	
	inline bool operator !=(const sVector< T, N >& _orig) const {
		for (short i = 0; i < N; ++i)
			if (_orig[i] != __data[i])
				return true;
		return false;
	}
	
	
	
	inline sVector< T, N >& operator =(const sVector< T, N >& _orig) {
		memcpy(__data, _orig, N * sizeof(T));
		return *this;
	}
	
	inline sVector< T, N >& operator +=(const sVector< T, N >& _orig) {
		for (short i = 0; i < N; ++i)
			__data[i] += _orig[i];
		return *this;
	}
	
	inline sVector< T, N >& operator -=(const sVector< T, N >& _orig) {
		for (short i = 0; i < N; ++i)
			__data[i] -= _orig[i];
		return *this;
	}
	
	
	inline sVector< T, N >& operator *=(const T& _orig) {
		for (T& t: __data)
			t *= _orig;
		return *this;
	}
	
	inline sVector< T, N >& operator /=(const T& _orig) {
		for (T& t: __data)
			t /= _orig;
		return *this;
	}
	
	inline friend sVector< T, N > operator +(const sVector< T, N >& _a, const sVector< T, N >& _b) {
		sVector< T, N > result;
		for (short i = 0; i < N; ++i)
			result[i] = _a[i] + _b[i];
		
		return result;
	}
	
	inline friend sVector< T, N > operator -(const sVector< T, N >& _a, const sVector< T, N >& _b) {
		sVector< T, N > result;
		for (short i = 0; i < N; ++i)
			result[i] = _a[i] - _b[i];
		
		return result;
	}
	
	inline friend sVector< T, N > operator *(const sVector< T, N >& _a, const T& _b) {
		sVector< T, N > result;
		for (short i = 0; i < N; ++i)
			result[i] = _a[i] * _b;
		
		return result;
	}
  
	inline friend sVector< T, N > operator /(const sVector< T, N >& _a, const T& _b) {
		sVector< T, N > result;
		for (short i = 0; i < N; ++i)
			result[i] = _a[i] / _b;
		
		return result;
	}
	
	inline friend sVector< T, N > operator -(const sVector< T, N >& _a) {
		sVector< T, N > result;
		for (short i = 0; i < N; ++i)
			result[i] = -_a[i];
		
		return result;
	}
	
  
			/*     FUNCTIONS     */
	
	inline sVector<T,N>& normalize() {
		T len = magnitude() ;
		if (len == 0) return *this;
		for (T& t: __data) t /= len;
    
    return *this;
	}
  
  inline T magnitude() {
		T len = 0;
		
		for (T& t: __data)
			len += (t * t);
		
		len = sqrt(len);
    
    return len;
  }
};

template < typename T >
inline sVector< T, 3 > cross(const sVector< T, 3 >& _a, const sVector< T, 3 >& _b) {
	return sVector< T, 3 >( {
			(_a[1] * _b[2]) - (_a[2] * _b[1]),
			(_a[2] * _b[0]) - (_a[0] * _b[2]),
			(_a[0] * _b[1]) - (_a[1] * _b[0])
		} );
}

template < typename T >
inline T dot(const sVectorBase< T >& _a, const sVectorBase< T >& _b) {
	T result = 0;
	for (short i = 0; i < _a.size(); ++i)
		result += (_a[i] * _b[i]);
	return result;
}

template < typename T >
inline T distance(const sVector< T, 3 >& _a, const sVector< T, 3 >& _b) {
  return sqrt(  distanceSquared(_a, _b) );
}

template < typename T >
inline T distanceSquared(const sVector< T, 3>& _a, const sVector< T, 3 >& _b) {
  return  ((_b[0]-_a[0])*(_b[0]-_a[0])) +
          ((_b[1]-_a[1])*(_b[1]-_a[1])) +
          ((_b[2]-_a[2])*(_b[2]-_a[2]));
}

template < typename T, int N >
inline sVector< T, N > min(const sVector< T, N >& _a, const sVector< T, N >& _b) {
  sVector< T, N > result;
  for (short i = 0; i < _a.size(); ++i) {
    result[i] = (_a[i] < _b[i]) ? _a[i] : _b[i];
  }
  return result;
}

template < typename T, int N >
inline sVector< T, N > max(const sVector< T, N >& _a, const sVector< T, N >& _b) {
  sVector< T, N > result;
  for (short i = 0; i < _a.size(); ++i) {
    result[i] = (_a[i] > _b[i]) ? _a[i] : _b[i];
  }
  return result;
}

/*
rotate a vector by _angle radians around _axis.
_axis must be length 1.
*/
template < typename T >
inline sVector< T, 3 > rotateAxisAngle(const sVector< T, 3 >& _v, const sVector< T, 3 >& _axis, T _angle) {
  T angleSin = sin(_angle / 2);
  sVector<T,4> quat;
  quat[0] = _axis[0] * angleSin;
  quat[1] = _axis[1] * angleSin;
  quat[2] = _axis[2] * angleSin;
  quat[3] = cos(_angle / 2);
  
  T x1 = quat[1]*_v[2] - quat[2]*_v[1];
  T y1 = quat[2]*_v[0] - quat[0]*_v[2];
  T z1 = quat[0]*_v[1] - quat[1]*_v[0];

  sVector<T,3> res;
  res[0] = _v[0] + 2 * (quat[3]*x1 + quat[1]*z1 - quat[2]*y1);
  res[1] = _v[1] + 2 * (quat[3]*y1 + quat[2]*x1 - quat[0]*z1);
  res[2] = _v[2] + 2 * (quat[3]*z1 + quat[0]*y1 - quat[1]*x1);

  return res;
}


/*
rotate a vector by _angle radians around _axis.
_axis must be length 1.
*/
template < typename T >
inline sVector<T,3> rotateFromTo(const sVector<T,3>& _v, const sVector<T,3>& _from, const sVector<T,3>& _to) {
  sVector<T,3> axis = cross(_from, _to).normalize();
  float angle = acos(dot(_from, _to));
  return rotateAxisAngle(_v, axis, angle);
}


typedef sVector< float, 2 > Vector2;
typedef sVector< float, 3 > Vector3;
typedef sVector< float, 4 > Vector4;
typedef sVector< float, 4 > sColor;


const Vector3 Z_AXIS = Vector3({0,0,1});
const Vector3 Y_AXIS = Vector3({0,1,0});
const Vector3 X_AXIS = Vector3({1,0,0});
  
#endif /* VECTORS_H */