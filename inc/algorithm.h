#ifndef _ALGORITHM_H_
#define _ALGORITHM_H_

#include "define.h"

class Algorithm
{
    public:
        Algorithm()=default;
        ~Algorithm()=default;

        virtual void SetParameters(const std::vector<std::any>& paramters){};

        virtual void Execute(){};

};

#endif