//
// Created by jordan on 5/22/16.
//

#ifndef __UNIT_TEST_UTILS__
#define __UNIT_TEST_UTILS__

#include <math.h>

namespace unit_test_utils
{
    template<typename T>
    static bool checkArraysEpsilonClose(T* array1, T* array2, int size, double epsilon)
    {
        for(int i = 0; i<size; i++)
        {
            if(fabs(array2[i]-array1[i])>epsilon)
            {
                return false;
            }

            return true;
        }
    }

    template<typename T>
    static bool checkArraysEq(T* array1, T* array2, int size)
    {
        for(int i = 0; i<size; i++)
        {
            if(array2[i]!=array1[i])
            {
                return false;
            }

            return true;
        }
    }
}

#endif //__UNIT_TEST_UTILS__
