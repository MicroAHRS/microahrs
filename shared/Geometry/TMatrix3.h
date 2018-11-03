#ifndef __MATRIX3__
#define __MATRIX3__

#include "TPoint3.h"

template <class T>
class TMatrix3
{
public:
    T x[3][3];

    TMatrix3 () {}
    explicit TMatrix3(T a) { *this = a; }
    TMatrix3 ( const TPoint3<T>& c1, const TPoint3<T>& c2, const TPoint3<T>& c3 ) {
        x [0][0] = c1.x;
        x [0][1] = c2.x;
        x [0][2] = c3.x;

        x [1][0] = c1.y;
        x [1][1] = c2.y;
        x [1][2] = c3.y;

        x [2][0] = c1.z;
        x [2][1] = c2.z;
        x [2][2] = c3.z;
    }

    TMatrix3<T>& operator =  ( T a) {
        x [0][1] = x [0][2] = x [1][0] =
        x [1][2] = x [2][0] = x [2][1] = 0.0;
        x [0][0] = x [1][1] = x [2][2] = a;
    }
    TMatrix3<T>& operator += ( const TMatrix3<T>& a) {
        x [0][0] += a.x [0][0];
        x [0][1] += a.x [0][1];
        x [0][2] += a.x [0][2];
        x [1][0] += a.x [1][0];
        x [1][1] += a.x [1][1];
        x [1][2] += a.x [1][2];
        x [2][0] += a.x [2][0];
        x [2][1] += a.x [2][1];
        x [2][2] += a.x [2][2];
        return *this;
    }
    TMatrix3<T>& operator -= ( const TMatrix3<T>& a) {
        x [0][0] -=a.x [0][0];
        x [0][1] -=a.x [0][1];
        x [0][2] -=a.x [0][2];
        x [1][0] -=a.x [1][0];
        x [1][1] -=a.x [1][1];
        x [1][2] -=a.x [1][2];
        x [2][0] -=a.x [2][0];
        x [2][1] -=a.x [2][1];
        x [2][2] -=a.x [2][2];
    }
    TMatrix3<T>& operator *= ( const TMatrix3<T>& a) {
        TMatrix3<T> c ( *this );
        return c * a;
    }
    TMatrix3<T>& operator *= ( T a ) {
        x [0][0] *= a;
        x [0][1] *= a;
        x [0][2] *= a;
        x [1][0] *= a;
        x [1][1] *= a;
        x [1][2] *= a;
        x [2][0] *= a;
        x [2][1] *= a;
        x [2][2] *= a;
        return *this;
    }
    TMatrix3<T>& operator /= ( T a ) {
        x [0][0] /= a;
        x [0][1] /= a;
        x [0][2] /= a;
        x [1][0] /= a;
        x [1][1] /= a;
        x [1][2] /= a;
        x [2][0] /= a;
        x [2][1] /= a;
        x [2][2] /= a;
        return *this;
    }

    const T * operator [] ( int i ) const { return & x[i][0]; }
    T * operator [] ( int i ) { return & x[i][0]; }

    TMatrix3<T>& invert    () {
        T	 det = det();
        TMatrix3<T> a;
        a.x [0][0] = (x [1][1]*x [2][2]-x [1][2]*x [2][1]) / det;
        a.x [0][1] = (x [0][2]*x [2][1]-x [0][1]*x [2][2]) / det;
        a.x [0][2] = (x [0][1]*x [1][2]-x [0][2]*x [1][1]) / det;
        a.x [1][0] = (x [1][2]*x [2][0]-x [1][0]*x [2][2]) / det;
        a.x [1][1] = (x [0][0]*x [2][2]-x [0][2]*x [2][0]) / det;
        a.x [1][2] = (x [0][2]*x [1][0]-x [0][0]*x [1][2]) / det;
        a.x [2][0] = (x [1][0]*x [2][1]-x [1][1]*x [2][0]) / det;
        a.x [2][1] = (x [0][1]*x [2][0]-x [0][0]*x [2][1]) / det;
        a.x [2][2] = (x [0][0]*x [1][1]-x [0][1]*x [1][0]) / det;

        return *this = a;
    }
    TMatrix3<T>& transpose () {
        TMatrix3<T> a;
        a.x [0][0] = x [0][0];
        a.x [0][1] = x [1][0];
        a.x [0][2] = x [2][0];
        a.x [1][0] = x [0][1];
        a.x [1][1] = x [1][1];
        a.x [1][2] = x [2][1];
        a.x [2][0] = x [0][2];
        a.x [2][1] = x [1][2];
        a.x [2][2] = x [2][2];
        return *this = a;
    }
    T   		 det       () const {
        return x [0][0]*(x [1][1]*x [2][2]-x [1][2]*x [2][1]) -
               x [0][1]*(x [1][0]*x [2][2]-x [1][2]*x [2][0]) +
               x [0][2]*(x [1][0]*x [2][1]-x [1][1]*x [2][0]);
    }

    TMatrix3<T>	inverse () const {
        return TMatrix3<T> ( *this ).invert ();
	}

    static	TMatrix3<T>	identity () { return TMatrix3<T> ( 1 ); }

    TMatrix3<T> operator * (  const TMatrix3<T>& b) const {
        TMatrix3<T> c ( *this );
        const TMatrix3<T>& a = *this;

        c.x[0][0]=a.x[0][0]*b.x[0][0]+a.x[0][1]*b.x[1][0]+a.x[0][2]*b.x[2][0];
        c.x[0][1]=a.x[0][0]*b.x[0][1]+a.x[0][1]*b.x[1][1]+a.x[0][2]*b.x[2][1];
        c.x[0][2]=a.x[0][0]*b.x[0][2]+a.x[0][1]*b.x[1][2]+a.x[0][2]*b.x[2][2];
        c.x[1][0]=a.x[1][0]*b.x[0][0]+a.x[1][1]*b.x[1][0]+a.x[1][2]*b.x[2][0];
        c.x[1][1]=a.x[1][0]*b.x[0][1]+a.x[1][1]*b.x[1][1]+a.x[1][2]*b.x[2][1];
        c.x[1][2]=a.x[1][0]*b.x[0][2]+a.x[1][1]*b.x[1][2]+a.x[1][2]*b.x[2][2];
        c.x[2][0]=a.x[2][0]*b.x[0][0]+a.x[2][1]*b.x[1][0]+a.x[2][2]*b.x[2][0];
        c.x[2][1]=a.x[2][0]*b.x[0][1]+a.x[2][1]*b.x[1][1]+a.x[2][2]*b.x[2][1];
        c.x[2][2]=a.x[2][0]*b.x[0][2]+a.x[2][1]*b.x[1][2]+a.x[2][2]*b.x[2][2];

        return c;
    }
    TMatrix3<T> operator * ( T b) const { TMatrix3<T> c ( *this );return (c *= b); }
    TPoint3<T>  operator * ( const TPoint3<T>& b) const {
        return TPoint3<T>(x [0][0]*b.x + x [0][1]*b.y + x [0][2]*b.z,
                          x [1][0]*b.x + x [1][1]*b.y + x [1][2]*b.z,
                          x [2][0]*b.x + x [2][1]*b.y + x [2][2]*b.z );
    }
};

template <class T>
TMatrix3<T> operator * ( T b, const TMatrix3<T>& a)  {
    TMatrix3<T> c ( a );return (c *= b);
}

#endif
