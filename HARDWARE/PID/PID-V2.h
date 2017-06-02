#ifndef PID2_H
#define PID2_H


#include "stm32f10x.h"
#include "data_transfer.h"

typedef struct _pid 
{
	int pv; /*integer that contains the process value*/
	int sp; /*integer that contains the set point*/
	float integral;
	float pgain;
	float igain;
	float dgain;
	int deadband;
	int last_error;
}pid_type;


extern pid_type warm;
extern int process_point, set_point;

void PID_main(void);
float pid_calc(pid_type *pid);
void pid_bumpless(pid_type *pid);






#endif

