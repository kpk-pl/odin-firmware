#ifndef _VECTOR_H_
#define _VECTOR_H_

#include "stm32f4xx.h"

typedef struct {
	int16_t x, y, z;
} Vector16;

typedef struct {
	float x, y, z;
} VectorF;

void VectorSet(VectorF *res, float num);
float VectorLen(const VectorF *v);
void VectorAdd(const VectorF *a, const VectorF *b, VectorF *res);
void VectorScale(const VectorF *v, const float scale, VectorF *res);
void VectorNormalize(const VectorF *v, VectorF *res);
void VectorCross(const VectorF *op1, const VectorF *op2, VectorF *res);
float VectorDot(const VectorF *op1, const VectorF *op2);


#endif /* _VECTOR_H_ */
