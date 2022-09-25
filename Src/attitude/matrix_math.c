#include "attitude/matrix_math.h"


void Matrix_create(struct Matrix *res, uint8_t M, uint8_t N)
{
    res->M = M;
    res->N = N;
    for (uint8_t i = 0; i < M * N; ++i)
    {
        res->rows[i] = 0.0;
    }
}

double Matrix_get(struct Matrix *mat, uint8_t m, uint8_t n)
{
    return mat->rows[m * mat->N + n];
}

void Matrix_set(struct Matrix *mat, uint8_t m, uint8_t n, double val)
{
    mat->rows[m * mat->N + n] = val;
}

void Matrix_multiply(struct Matrix *m1, struct Matrix *m2, struct Matrix *res)
{
    for (uint8_t t = 0; t < m1->M; t++)
    {
        for (uint8_t i = 0; i < m1->N; i++)
        {
            float sum = 0.0f;
            for (uint8_t j = 0; j < m2->M; j++)
            {
                sum += Matrix_get(m1, t, j) * Matrix_get(m2, j, i);
            }
            Matrix_set(res, t, i, sum);
        }
    }
}

void Matrix_multiply_scalar(struct Matrix *m1, double scalar)
{
    for (uint8_t i = 0; i < m1->M * m1->N; i++)
    {
        m1->rows[i] *= scalar;
    }
}

void Matrix_add(struct Matrix *m1, struct Matrix *m2)
{
    for (uint8_t i = 0; i < m1->M * m1->N; i++)
    {
        m1->rows[i] += m2->rows[i];
    }
}

void Matrix_subtract(struct Matrix *m1, struct Matrix *m2) {
    for (uint8_t i = 0; i < m1->M * m1->N; i++)
    {
        m1->rows[i] -= m2->rows[i];
    }
}

void Matrix_diagonal_divide(struct Matrix *m1, struct Matrix *m2) {
    for (uint8_t i = 0; i < m1->M; i++) {
        Matrix_set(m1, i, i, Matrix_get(m1, i, i) / Matrix_get(m2, i, i));
    }
}

void Matrix_add_scalar(struct Matrix *m1, double scalar)
{
    for (uint8_t i = 0; i < m1->M * m1->N; i++)
    {
        m1->rows[i] += scalar;
    }
}

void Matrix_copy(struct Matrix *src, struct Matrix *dest)
{
    for (uint8_t i = 0; i < src->M * src->N; i++)
    {
        dest->rows[i] = src->rows[i];
    }
}

double Matrix_max(struct Matrix *mat)
{
    double max = mat->rows[0];
    for (uint8_t i = 1; i < mat->M * mat->N; i++)
    {
        if (max < mat->rows[i]) max = mat->rows[i];
    }

    return max;
}

double Matrix_min(struct Matrix *mat)
{
    double min = mat->rows[0];
    for (uint8_t i = 1; i < mat->M * mat->N; i++)
    {
        if (min > mat->rows[i]) min = mat->rows[i];
    }

    return min;
}

