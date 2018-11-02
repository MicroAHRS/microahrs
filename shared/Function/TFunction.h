#ifndef TFUNCTION_H
#define TFUNCTION_H


template <class T>
class TFunction {
public:
    inline T operator () (const T& param) const {return param;}
};

#endif // TFUNCTION_H
