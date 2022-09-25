#ifndef MATRIX_MATH_H
#define MATRIX_MATH_H

#include "stdint.h"
#include "memory.h"
#include "string.h"
#include "stdlib.h"

struct Matrix
{
    uint8_t M;
    uint8_t N;
    float rows[4 * 4];
};


void Matrix_create(struct Matrix *res, uint8_t M, uint8_t N);

double Matrix_get(struct Matrix *mat, uint8_t m, uint8_t n);

void Matrix_set(struct Matrix *mat, uint8_t m, uint8_t n, double val);

void Matrix_multiply(struct Matrix *m1, struct Matrix *m2, struct Matrix *res);

void Matrix_multiply_scalar(struct Matrix *m1, double scalar);

void Matrix_add(struct Matrix *m1, struct Matrix *m2);

void Matrix_subtract(struct Matrix *m1, struct Matrix *m2);

void Matrix_diagonal_divide(struct Matrix *m1, struct Matrix *m2);

void Matrix_add_scalar(struct Matrix *m1, double scalar);

void Matrix_copy(struct Matrix *src, struct Matrix *dest);

double Matrix_max(struct Matrix *mat);

double Matrix_min(struct Matrix *mat);

#endif
