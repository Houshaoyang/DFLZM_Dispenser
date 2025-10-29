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

/*增量式PID算法
@param pid  要计算的pid结构体指针
返回给执行器的输出值
*/
void incrementalPid(PID *pid,float setValue,float actualValue,float *total_output)
{
    float increment;
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

/*位置式pid
@param pid  要计算的pid结构体指针
返回 PID控制器输出值，在模拟仿真里面也作为传感器的值
*/
void positionPid(PID *pid,float setValue,float actualValue,float *total_output)
{ 
    pid->error = pid->setValue - pid->actualValue;
    pid->integral += pid->error;   		/*误差累积 与增量式pid不同，位置式需要进行错误累计积分*/
    pid->P = pid->Kp*pid->error;		//比例项
    pid->I = pid->Ki*pid->integral;		//积分项
    pid->D =pid->Kd*(pid->error-pid->errorPre);//微分项
    pid->errorPrePre = pid->errorPre;
    pid->errorPre = pid->error;
    *total_output = pid->P + pid->I +pid->D;
		if(*total_output >100) *total_output = 100;//限幅
		if(*total_output <0)	*total_output = 0;
}
