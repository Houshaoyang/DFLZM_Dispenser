#ifndef _PID_H_
#define _PID_H_

typedef struct PID_
{
    float actualValue;     //实际值 每次反馈回来的值，模拟传感器采集到的值 
    float Kd, Ki, Kp;      //比例、积分、微分常数
    float P, I, D;         //比例项 积分项 微分项
		float error;           //本次误差值
    float errorPre;        //E[k-1]  //上一次误差
    float errorPrePre;     //E[k-2]  //上上次误差
    float setValue;        //设定值 你期望系统达到的值
    float integral;        //积分值
}PID;

void pid_init(double kp, double ki, double kd, PID *pid);
void incrementalPid(PID *pid,float setValue,float actualValue,float *total_output);
#endif
