#ifndef TPOINT3_H
#define TPOINT3_H

#include <math.h>

template<class T>
class TPoint3
{
public:

    T x;
    T y;
    T z;

public:
    TPoint3() { x = y = z = 0; }
    explicit TPoint3(T val) { x = y = z = val; }
    explicit TPoint3(T _x, T _y, T _z ) { x = _x;y = _y; z = _z; }

    inline bool operator == (const TPoint3<T>& p) const { return  x==p.x && y==p.y && z==p.z; }
    inline TPoint3<T> operator + (const TPoint3<T>& p) const { return TPoint3<T>(x + p.x, y + p.y, z + p.z);}
    inline TPoint3<T> operator - (const TPoint3<T>& p) const { return TPoint3<T>(x - p.x, y - p.y, z - p.z);}
    inline TPoint3<T> operator * (const T& f) const { return TPoint3<T>(x * f, y * f, z * f); }
    inline TPoint3<T> operator / (const T& f) const { return TPoint3<T>(x / f, y / f, z / f); }

    inline TPoint3<T>& operator = (const TPoint3<T>& p) {  x=p.x; y=p.y; z=p.z; return *this; }
    inline TPoint3<T>& operator += (const TPoint3<T>& p) {  x+=p.x; y+=p.y; z+=p.z; return *this; }
    inline TPoint3<T>& operator -= (const TPoint3<T>& p) {  x-=p.x; y-=p.y; z-=p.z; return *this; }

    inline TPoint3<T>& operator *= (const T& f) { x*=f; y*=f; z*=f; return *this; }
    inline TPoint3<T>& operator /= (const T& f) { x/=f; y/=f; z/=f; return *this; }
    inline TPoint3<T>& scale (const TPoint3<T>& p) { x*=p.x; y*=p.y; z*=p.z; return *this; }
    inline T           dot_product (const TPoint3<T>& p) const { return x*p.x + y*p.y + z*p.z;}
    inline TPoint3<T>& cross_product(const TPoint3<T>& p) const { return TPoint3<T>( y*p.z - z*p.y, z*p.x - x*p.z , x*p.y - y*p.x ) ; }

    inline bool isZero() const { return x == 0 || y == 0 || z == 0 ;}
    inline bool hasNan() const {return x!=x || y!=y || z!=z;}

    inline T lengthSq() const { return x*x + y*y + z*z;}

    //-------------------------------------------------------------------------------------------
    // Fast inverse square-root
    // See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
    // ProninE : I found that fast sqrt is dangerous at Arduion
    // it returns Nan for vector (0,0,10)
    // So use default sqrt!
    inline T length() const { return sqrt( lengthSq() );}

    TPoint3<T>& normalize() { return *this /= length(); }
};



#endif // TPOINT3_H
