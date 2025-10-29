#include <stdlib.h>
#include <string.h>
#include "pid.h"

/*pid初始化*/
void pid_init(double kp, double ki, double kd, PID *pid)
{
		memset(pid,0,sizeof(PID)); 
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}

float increment;
void incrementalPid(PID *pid,float setValue,float actualValue,float *total_output)
{
//    float increment;
		pid->setValue = setValue;
		pid->actualValue = actualValue;
    pid->error = pid->setValue - pid->actualValue;
    pid->P = pid->Kp*(pid->error-pid->errorPre);			//比例项
    pid->I = pid->Ki*pid->error;							//积分项
    pid->D = pid->Kd*(pid->error-2*pid->errorPre + pid->errorPrePre); 			//微分项
    increment = pid->P + pid->I + pid->D;   				//pid计算得到的输出增量
		*total_output += increment;
		if(*total_output >100) *total_output = 100;//限幅
		if(*total_output <0)	*total_output = 0;	
    pid->actualValue += increment;    	 					//实际反馈值加上增量
    pid->errorPrePre = pid->errorPre;
    pid->errorPre = pid->error;
}