#include "pid.h"
#include "string.h"

void pid_reset_integrator(pid_regulator_t *pid) {
    
    pid->sumError = 0;
    pid->lastValue = 0;
}

void pid_init(pid_regulator_t *pid) {

    memset(pid, 0, sizeof(pid_regulator_t));
    
    pid->output = 0;
    pid->lastValue = 0;
    pid->sumError = 0;

    pid_set_tunings(pid, 5, 0, 0);
}

void pid_set_tunings(pid_regulator_t *pid, pid_short_int Kp, pid_short_int Ki, pid_short_int Kd) {

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    
    //pid->maxError = MAX_INT / (pid->Kp + 1);
    //pid->maxSumError = MAX_I_TERM / (pid->Ki + 1);
    
    pid_reset_integrator(pid);
}


void pid_calculate(pid_regulator_t *pid, pid_short_int desired, pid_short_int actual, float dt) {
    (void)dt;
    
#ifdef USE_SIMPLE_PID
    /* Update PID values */
    
    float Kp = pid->Kp / 10.0f;
    float Ki = pid->Ki / 10.0f;
    float Kd = pid->Kd / 10.0f;
    /*
    float error = desired - actual;
    float pTerm = Kp * error;
    pid->sumError += Ki * 100.0f * error * dt; // Multiplication with Ki is done before integration limit, to make it independent from integration limit value
    if (pid->sumError < -100.0f) {
        pid->sumError = -100.0f;
    }
    else if (pid->sumError > 100.0f) {
        pid->sumError = 100.0f;
    }
    float dTerm = (Kd / 100.0f) * (error - pid->lastValue) / dt;
    pid->lastValue = error;
    pid->output = pTerm + pid->sumError + dTerm;
    */
    
    float error = desired - actual;
    float pTerm = Kp * error;
    pid->sumError += error;
#define GUARD_GAIN 50
    if (pid->sumError < -GUARD_GAIN) {
        pid->sumError = -GUARD_GAIN;
    }
    else if (pid->sumError > GUARD_GAIN) {
        pid->sumError = GUARD_GAIN;
    }
    float iTerm = Ki * pid->sumError;
    float dTerm = Kd * (error - pid->lastValue);
    pid->lastValue = error;
    
    float tmp = (pTerm + iTerm + dTerm);
    if (tmp > 255) {
        tmp = 255;
    } else if (tmp < -255) {
        tmp = -255;
    }
    pid->output = tmp;
#else
    pid_short_int pTerm, dTerm;
    pid_long_int iTerm;
    pid_short_int error = desired - actual;
    
    if (error > pid->maxError) {
        
        pTerm = MAX_INT;
    } else if (error < -pid->maxError) {
        
        pTerm = -MAX_INT;
    } else {
        
        pTerm = pid->Kp * error;
    }
    
    pid_long_int tmp = pid->sumError + error;
    if (tmp > pid->maxSumError) {
        iTerm = MAX_I_TERM;
        pid->sumError = pid->maxSumError;
    } else if (tmp < -pid->maxSumError) {
        
        iTerm = -MAX_I_TERM;
        pid->sumError = -pid->maxSumError;
    } else {
        
        pid->sumError = tmp;
        iTerm = pid->Ki * pid->sumError;
    }
    
    dTerm = pid->Kd * (pid->lastValue - actual);
    
    pid->lastValue = actual;
    
    tmp = (pTerm + iTerm + dTerm) / SCALING_FACTOR;
    if (tmp > MAX_INT) {
        tmp = MAX_INT;
    }
    else if (tmp < -MAX_INT) {
        
        tmp = -MAX_INT;
    }
    
    pid->output = tmp;
#endif
}
