#ifndef __TASK_H
#define __TASK_H
#include "stm32f4xx.h"

void Task1_Run(void);
void Task2_Run(void);
void Task3_Run(void);
void Task4_Run(void);
void LineFollow_UseDefaultProfile(void);
void LineFollow_UseTaskProfile(uint8_t task_id);

#endif
