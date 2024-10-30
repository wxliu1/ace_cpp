
#include <iostream>

// #include "emmintrin.h"
// #include "xmmintrin.h"
#include <wmmintrin.h> // AVX

#include <Eigen/Dense>
#include <Eigen/Core>

using namespace Eigen;

void MatrixMultiplySSE(const float* A, const float* B, float* C) {
    __m128 row1 = _mm_loadu_ps(&A[0]);
    float row1_temp[4] = { 0 };
    _mm_store_ps(row1_temp, row1);
    std::cout << "\nrow1:\n";
    for(int i = 0; i < 4; i++)
    {
        std::cout << row1_temp[i] << std::endl;
    }
    __m128 row2 = _mm_loadu_ps(&A[4]);
    __m128 row3 = _mm_loadu_ps(&A[8]);
    __m128 row4 = _mm_loadu_ps(&A[12]);

    for (int i = 0; i < 4; i++) {
        __m128 brod1 = _mm_set1_ps(B[4 * i + 0]);
        float brod1_temp[4] = { 0 };
        _mm_store_ps(brod1_temp, brod1);
        std::cout << "\nbrod1:\n";
        for(int i = 0; i < 4; i++)
        {
            std::cout << brod1_temp[i] << std::endl;
        }
        __m128 brod2 = _mm_set1_ps(B[4 * i + 1]);
        float brod2_temp[4] = { 0 };
        _mm_store_ps(brod2_temp, brod2);
        std::cout << "\nbrod2:\n";
        for(int i = 0; i < 4; i++)
        {
            std::cout << brod2_temp[i] << std::endl;
        }
        __m128 brod3 = _mm_set1_ps(B[4 * i + 2]);
        __m128 brod4 = _mm_set1_ps(B[4 * i + 3]);
        __m128 row = _mm_add_ps(
            _mm_add_ps(_mm_mul_ps(row1, brod1), _mm_mul_ps(row2, brod2)),
            _mm_add_ps(_mm_mul_ps(row3, brod3), _mm_mul_ps(row4, brod4))
        );
        _mm_storeu_ps(&C[4 * i], row);
    }
}

int main(int argc, char* argv[])
{
    int a[5] = {1,2,3,4,5};
    // a += 4;
    int *p_a = a;
    p_a +=4;
    std::cout << "p_a[0]=" << p_a[0] << std::endl;
    return 0;

    Matrix<float, 4, 4> mat1;// = Matrix<int, 4, 4>::Random();
    Matrix<float, 4, 4> mat2;// = Matrix<int, 4, 4>::Random();

    mat1 << 1,2,3,4,
            5,6,7,8,
            9,10,11,12,
            13,14,15,16;

    mat2 << 4,3,2,1,
            8,7,6,5,
            12,11,10,9,
            16,15,14,13;        

    std::cout << "1 random matrix: \n" << mat1 << std::endl;
    std::cout << "2 random matrix: \n" << mat2 << std::endl;

    std::cout << "mat1 * mat2=\n" << mat1 * mat2 << std::endl;

    float* A = mat1.data();
    float* B = mat2.data();

    std::cout << "1 mat1(0, 0)=" << mat1(0, 0) << std::endl;
    A[0] = 100;
    std::cout << "2 mat1(0, 0)=" << mat1(0, 0) << std::endl;

    for(int i = 0; i < 16; i++)
    {
        std::cout << "A" << i << "=" << A[i] << std::endl;
        std::cout << "B" << i << "=" << B[i] << std::endl;
    }

    float C[16] = { 0 };
    MatrixMultiplySSE(A, B, C);
    for(int i = 0; i < 16; i++)
    std::cout << "C" << i << "=" << C[i] << std::endl;


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