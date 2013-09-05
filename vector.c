#include "vector.h"
#include <math.h>

void VectorSet(VectorF *res, float num) {
	res->x = res->y = res->z = num;
}

void VectorNormalize(const VectorF *v, VectorF *res) {
	float len = VectorLen(v);
	VectorScale(v, 1.0f/len, res);
}

void VectorCross(const VectorF *op1, const VectorF *op2, VectorF *res) {
	res->x = op1->y * op2->z - op1->z * op2->y;
	res->y = op1->z * op2->x - op1->x * op2->z;
	res->z = op1->x * op2->y - op1->y * op2->x;
}

float VectorDot(const VectorF *op1, const VectorF *op2) {
	return op1->x * op2->x + op1->y * op2->y + op1->z * op2->z;
}

float VectorLen(const VectorF *v) {
	return sqrtf(VectorDot(v, v));
}

void VectorAdd(const VectorF *a, const VectorF *b, VectorF *res) {
	res->x = a->x + b->x;
	res->y = a->y + b->y;
	res->z = a->z + b->z;
}

void VectorScale(const VectorF *v, const float scale, VectorF *res) {
	res->x = v->x * scale;
	res->y = v->y * scale;
	res->z = v->z * scale;
}
