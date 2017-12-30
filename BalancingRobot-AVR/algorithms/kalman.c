#include "kalman.h"
#include "string.h"

void kalman_init(kalman_filter_t *filter) {

    memset(filter, 0, sizeof(kalman_filter_t));
    
    filter->qAngle = 0.001f;
    filter->qBias = 0.003f;
    filter->rMeasure = 0.03f;
    
    filter->bias = 0.0f;
    filter->angle = 0.0f;
    
    filter->P[0][0] = 0.0f;
    filter->P[0][1] = 0.0f;
    filter->P[1][0] = 0.0f;
    filter->P[1][1] = 0.0f;
}

void kalman_calculate(kalman_filter_t *filter, float newAngle, float newRate, float dt) {

    filter->rate = newRate - filter->bias;
    filter->angle += dt * filter->rate;
    
    filter->P[0][0] += dt * (dt*filter->P[1][1] - filter->P[0][1] - filter->P[1][0] + filter->qAngle);
    filter->P[0][1] -= dt * filter->P[1][1];
    filter->P[1][0] -= dt * filter->P[1][1];
    filter->P[1][1] += filter->qBias * dt;
    
    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = filter->P[0][0] + filter->rMeasure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = filter->P[0][0] / S;
    K[1] = filter->P[1][0] / S;
    
    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - filter->angle; // Angle difference
    /* Step 6 */
    filter->angle += K[0] * y;
    filter->bias += K[1] * y;
    
    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = filter->P[0][0];
    float P01_temp = filter->P[0][1];
    
    filter->P[0][0] -= K[0] * P00_temp;
    filter->P[0][1] -= K[0] * P01_temp;
    filter->P[1][0] -= K[1] * P00_temp;
    filter->P[1][1] -= K[1] * P01_temp;
}
