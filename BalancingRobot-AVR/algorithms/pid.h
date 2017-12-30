#ifndef PID_H
#define PID_H

#include <stdint.h>

#define USE_FLOAT_PID
#define USE_SIMPLE_PID

#define SCALING_FACTOR  (128)

#ifdef USE_FLOAT_PID
#define MAX_INT         INT16_MAX
#define MAX_LONG        INT16_MAX
#else
#define MAX_INT         INT16_MAX
#define MAX_LONG        INT32_MAX
#endif

#define MAX_I_TERM      (MAX_LONG / 2)

#ifdef USE_FLOAT_PID
typedef float pid_short_int;
typedef double pid_long_int;
#else
typedef int16_t pid_short_int;
typedef int32_t pid_long_int;
#endif

typedef struct {
    
    pid_short_int output;
    
    pid_short_int lastValue;
    
    pid_long_int sumError;
    pid_short_int maxError;
    pid_long_int maxSumError;
    
    // Tuning contants
    pid_short_int Kp, Ki, Kd;
} pid_regulator_t;

void pid_init(pid_regulator_t *pid);
void pid_calculate(pid_regulator_t *pid, pid_short_int desired, pid_short_int actual, float dt);
void pid_set_tunings(pid_regulator_t *pid, pid_short_int Kp, pid_short_int Ki, pid_short_int Kd);
#endif /* PID_H */
