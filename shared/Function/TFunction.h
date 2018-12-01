#ifndef TFUNCTION_H
#define TFUNCTION_H


template <class TVal, class TArg>
class TFunction {
public:
    TVal operator () (const TArg& param) const  {return TVal(0);}
};

#endif // TFUNCTION_H
