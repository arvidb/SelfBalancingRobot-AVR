#include "simpletask.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>

#define MAX_TASKS (10)

typedef struct task {
    uint32_t period;      // Rate at which the task should tick
    uint32_t elapsed_time; // Time since task's last tick
    tick_func func;     // Function to call for task's tick
} task;

task tasks[MAX_TASKS];
int task_count = 0;

static volatile uint32_t timer = 0;
static volatile uint32_t last_time = 0;

ISR (TIMER1_COMPA_vect)  // timer1 overflow interrupt
{
    timer += 1;
}

void simpletask_init(void) {

    task_count = 0;
    
    // Set the Timer Mode to CTC
    //TCCR1B = _BV(WGM12);
    // 249
    // 8000000 / (64 * 1000hz) - 1 = 124
    uint64_t comp_val = (F_CPU / (8UL * 1000UL)) - 1UL;
    OCR1A = (uint16_t)comp_val;
    //OCR1A = 0xF9; // TODO: calculate value
    
    TIMSK1 = _BV(OCIE1A);    //Set the ISR COMPA vect
    
    TCCR1B = _BV(CS11) | _BV(WGM12); // set prescaler to 8 and start the timer
}

void simpletask_add(tick_func func, uint16_t period) {

    tasks[task_count].period = period;
    tasks[task_count].elapsed_time = 0;
    tasks[task_count].func = func;
    task_count++;
}

void simpletask_run(void) {

    uint32_t now = 0;
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        now = timer;
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE);
    
    uint32_t elapsed = now - last_time;
    
    // Heart of the scheduler code
    for (uint8_t i=0; i < task_count; ++i) {
        tasks[i].elapsed_time += elapsed;
        
        if (tasks[i].elapsed_time >= tasks[i].period) { // Ready
            tasks[i].func(0); //execute task tick
            tasks[i].elapsed_time = 0;
        }
    }
    
    last_time = now;
}
