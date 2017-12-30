#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>

typedef struct {
    
    float qAngle;
    float qBias;
    float rMeasure;
    
    float angle; // out
    float bias;
    float rate;
    
    float P[2][2];
    
} kalman_filter_t;

void kalman_init(kalman_filter_t *filter);
void kalman_calculate(kalman_filter_t *filter, float newAngle, float newRate, float dt);

#endif /* KALMAN_H */
