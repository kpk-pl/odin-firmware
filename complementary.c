#include "complementary.h"

void ComplementaryInit(Complementary_State* state, float coef) {
	state->coef1 = coef;
	state->coef2 = 1.0f - coef;
}

float ComplementaryGet(Complementary_State* state, float value1, float value2) {
	return state->coef1 * value1 + state->coef2 * value2;
}
