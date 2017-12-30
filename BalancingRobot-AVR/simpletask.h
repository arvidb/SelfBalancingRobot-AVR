#ifndef SIMPLETASK_H
#define SIMPLETASK_H

#include <stdint.h>

typedef void (*tick_func)(void *);

void simpletask_add(tick_func func, uint16_t period);

void simpletask_init(void);
void simpletask_run(void);

#endif /* SIMPLETASK_H */
