#ifndef TQUATERNION_H
#define TQUATERNION_H

#include "TPoint3.h"

template<class T>
class TQuaternion
{
public:
    T w;
    T x;
    T y;
    T z;

public:
    TQuaternion() { w = x = y = z = 0; }
    explicit TQuaternion(T val) { w = x = y = z = val; }
    explicit TQuaternion(T _w, T _x, T _y, T _z ) { w = _w;x = _x;y = _y; z = _z; }
    TQuaternion(const TQuaternion<T>& q) { w = q.w;x = q.x;y = q.y;z = q.z;}
    explicit TQuaternion(const TPoint3<T>& v) { w = 0.0;x = v.x;y=v.y;z=v.z;}


    inline TQuaternion<T> operator + () const { return *this;}
    inline TQuaternion<T> operator - () const { return TQuaternion<T> ( -w , -x, -y, -z);}

    inline TQuaternion<T>& invert() { x = -x; y = -y; z = -z; return *this;}
    inline TQuaternion<T> getInvert() const { return TQuaternion<T>(w,-x,-y,-z);}
    inline bool isZero() const { return w==0 && x==0 && y==0 && z==0;}

    inline TQuaternion<T>& operator += ( const TQuaternion<T>& q ) { x += q.x; y += q.y; z += q.z; w += q.w;return *this;}
    inline TQuaternion<T>& operator -= ( const TQuaternion<T>& q ) { x -= q.x; y -= q.y; z -= q.z; w -= q.w;return *this;}
    inline TQuaternion<T>& operator *= (const T& f) { w*=f;x*=f; y*=f; z*=f; return *this; }
    inline TQuaternion<T>& operator /= (const T& f) { w/=f;x/=f; y/=f; z/=f; return *this; }

    TQuaternion<T>& operator *= ( const TQuaternion<T>& q ) {
        *this = TQuaternion<T> (
                w * q.w - x * q.x - y * q.y - z * q.z,
                w * q.x + x * q.w + y * q.z - z * q.y,
                w * q.y - x * q.z + y * q.w + z * q.x ,
                w * q.z + x * q.y - y * q.x + z * q.w );
        return *this;
    }

    inline TQuaternion<T> operator * ( const T& f) const { return TQuaternion<T>(w*f,x*f,y*f,z*f); }
    inline TQuaternion<T> operator / ( const T& f) const { return TQuaternion<T>(w/f,x/f,y/f,z/f); }
    inline TQuaternion<T> operator + ( const TQuaternion<T>& q2 ) const {
        return TQuaternion<T> ( x + q2.x, y + q2.y, z + q2.z, w + q2.w ); }
    inline TQuaternion<T> operator - ( const TQuaternion<T>& q2 ) const {
        return TQuaternion<T> ( x - q2.x, y - q2.y, z - q2.z, w - q2.w ); }

    inline TQuaternion<T> operator * ( const TQuaternion<T>& q ) const {
        return TQuaternion<T> (
                    w * q.w - x * q.x - y * q.y - z * q.z,
                    w * q.x + x * q.w + y * q.z - z * q.y,
                    w * q.y - x * q.z + y * q.w + z * q.x ,
                    w * q.z + x * q.y - y * q.x + z * q.w);
    }

    inline T lengthSq() const { return w*w + x*x + y*y + z*z;}
    inline T length() const { return sqrt( lengthSq() );}
    inline TQuaternion<T>& normalize()  { return *this /= length(); }

    inline TPoint3<T> toVector3() {return TPoint3<T>(x,y,z);}

    TPoint3<T> rotateVector ( const TPoint3<T>& v ) const {
        TPoint3<T> r;
        r.x = v.x * (0.5f - y * y - z * z)
            + v.y * (w * z + x * y)
            + v.z * (x * z - w * y);
        r.y = v.x * (x * y - w * z)
            + v.y * (0.5f - x * x - z * z)
            + v.z * (w * x + y * z);
        r.z = v.x * (w * y + x * z)
            + v.y * (y * z - w * x)
            + v.z * (0.5f - x * x - y * y);

        return r * 2;
        // странно наданный метод возвращает другие результаты

        //TQuaternion<T> p  = *this;
        //p *= TQuaternion<T>(0, v.x, v.y , v.z);
        //p *= getInvert();
        //return TVector3<T>( p.x, p.y, p.z );
    }
};


#endif // TQUATERNION_H
