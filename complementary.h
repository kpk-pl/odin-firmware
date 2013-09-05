#ifndef _COMPLEMENTARY_H_
#define _COMPLEMENTARY_H_

typedef struct {
	float coef1;
	float coef2;
} Complementary_State;

void ComplementaryInit(Complementary_State* state, float coef);
float ComplementaryGet(Complementary_State* state, float value1, float value2);

#endif /* _COMPLEMENTARY_H_ */
