#ifndef VECTORS_H
#define VECTORS_H

#include <cstring>
#include <cmath>
#include <initializer_list>

#include <assert.h>

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
	
			/*     FUNCTIONS     */
	
	inline void normalize() {
		float len = 0;
		
		for (T& t: __data)
			len += (t * t);
		
		len = sqrt(len);
		
		if (len == 0)
			return;
		
		for (T& t: __data)
			t /= len;
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
inline T dotProduct(const sVectorBase< T >& _a, const sVectorBase< T >& _b) {
	T result = 0;
	for (short i = 0; i < _a.size(); ++i)
		result += (_a[i] * _b[i]);
	return result;
}

typedef sVector< float, 2 > sVector2D;
typedef sVector< float, 3 > sVector3D;
typedef sVector< float, 4 > sVector4D;
typedef sVector< float, 4 > sColor;

#endif /* VECTORS_H */