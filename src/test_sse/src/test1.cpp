
#include <iostream>

// #include "emmintrin.h"
// #include "xmmintrin.h"
#include <wmmintrin.h> // AVX

int main(int argc, char* argv[])
{
    float num = 3.14159265358979;
    __m128 numl = _mm_set1_ps(num);
    float temp[4] = { 0 };
    _mm_store_ps(temp, numl);
    for(int i = 0; i < 4; i++)
    {
        std::cout << temp[i] << std::endl;
    }

    float temp2[] = { 0.1, 0.2, 0.3, 0.4 };
    // __m128 tmp2l =  _mm_load1_ps(temp2);
    __m128 tmp2l =  _mm_load_ps(temp2);
    float temp3[4] = { 0 };
    _mm_store_ps(temp3, tmp2l);
    for(int i = 0; i < 4; i++)
    {
        std::cout << temp3[i] << std::endl;
    }
    float temp4[] = { 0.1, 0.2, 0.3, 0.4 };
    __m128 tmp4l =  _mm_load_ps(temp4);
    float temp5[4] = { 0 };
    _mm_store_ps(temp5, _mm_mul_ps(tmp2l, tmp4l));
    for(int i = 0; i < 4; i++)
    {
        std::cout << temp5[i] << std::endl;
    }
    return 0;
}