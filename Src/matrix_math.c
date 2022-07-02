#include "matrix_math.h"

void add_matrix(float *matrix, float *matrix2, float *matrix3, int M, int N)
{
    for (int i = 0; i < M; i++)
    {
        for (int p = 0; p < N; p++)
        {
            *((matrix3 + i * N) + p) = *((matrix + i * N) + p) + *((matrix2 + i * N) + p);
        }
    }
}

void sub_matrix(float *matrix, float *matrix2, float *matrix3, int M, int N)
{
    for (int i = 0; i < M; i++)
    {
        for (int p = 0; p < N; p++)
        {
            *((matrix3 + i * N) + p) = *((matrix + i * N) + p) - *((matrix2 + i * N) + p);
        }
    }
}

void mul_matrix(float *matrix, int M, int N, float *matrix2, int M2, int N2, float *matrix3)
{
    for (int i = 0; i < M; i++)
    {
        for (int p = 0; p < N; p++)
        {
            float sum = 0.0f;
            for (int j = 0; j < N; j++)
            {
                sum += *((matrix + i * N) + j) * *((matrix2 + j * N) + p);
            }
            *((matrix3 + i * N) + p) = sum;
        }
    }
}
