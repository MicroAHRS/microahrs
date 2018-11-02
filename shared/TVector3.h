#ifndef TVECTOR3_H
#define TVECTOR3_H


template<class T>
class TVector3
{
public:

    T x;
    T y;
    T z;

public:
    TVector3() { x = y = z = 0; }
    explicit TVector3(T val) { x = y = z = val; }
    explicit TVector3(T _x, T _y, T _z ) { x = _x;y = _y; z = _z; }

    inline bool operator == (const TVector3<T>& p) const { return  x==p.x && y==p.y && z==p.z; }
    inline TVector3<T> operator + (const TVector3<T>& p) const { return TVector3<T>(x + p.x, y + p.y, z + p.z);}
    inline TVector3<T> operator - (const TVector3<T>& p) const { return TVector3<T>(x - p.x, y - p.y, z - p.z);}
    inline TVector3<T> operator * (const T& f) const { return TVector3<T>(x * f, y * f, z * f); }
    inline TVector3<T> operator / (const T& f) const { return TVector3<T>(x / f, y / f, z / f); }

    inline TVector3<T>& operator = (const TVector3<T>& p) {  x=p.x; y=p.y; z=p.z; return *this; }
    inline TVector3<T>& operator += (const TVector3<T>& p) {  x+=p.x; y+=p.y; z+=p.z; return *this; }
    inline TVector3<T>& operator -= (const TVector3<T>& p) {  x-=p.x; y-=p.y; z-=p.z; return *this; }

    inline TVector3<T>& operator *= (const T& f) { x*=f; y*=f; z*=f; return *this; }
    inline TVector3<T>& operator /= (const T& f) { x/=f; y/=f; z/=f; return *this; }
    inline TVector3<T>& scale (const TVector3<T>& p) { x*=p.x; y*=p.y; z*=p.z; return *this; }
    inline T            cross (const TVector3<T>& p) const { return x*p.x + y*p.y + z*p.z;}

    inline bool isZero() const { return x == 0 || y == 0 || z == 0 ;}
    inline bool hasNan() const {return x!=x || y!=y || z!=z;}

    inline T lengthSq() const { return x*x + y*y + z*z;}
    inline T length() const { return sqrt( lengthSq() );}

    TVector3<T>& normalize() { return *this /= length(); }
};

//typedef TVector3<double> TVector3d;
typedef TVector3<float>  TVector3f;
typedef TVector3<int>    TVector3i;


#endif // TVECTOR_H
