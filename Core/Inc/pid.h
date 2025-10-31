#ifndef _PID_H_
#define _PID_H_

#define KP_BASE 1.5
#define KI_BASE 0.2
#define KD_BASE 0.5

typedef enum{
DRIECT,
REVERSE
}ctrl_dir;

// 定义dPID结构体，存储△kP、△ki、△kd
typedef struct {
    float Kp;
    float Ki;
    float Kd;
}KPID;

typedef struct PID_
{
    float actualValue;     //实际值 每次反馈回来的值，模拟传感器采集到的值 
    float Kp,Ki,Kd;      //比例、积分、微分常数
    float P, I, D;         //比例项 积分项 微分项
		float error;           //本次误差值
    float errorPre;        //E[k-1]  //上一次误差
    float errorPrePre;     //E[k-2]  //上上次误差
    float setValue;        //设定值 你期望系统达到的值
    float integral;        //积分值
		unsigned char ctrl_dir;        //控制方向
}PID;

void pid_init(double kp, double ki, double kd,ctrl_dir dir, PID *pid);
void incrementalPid(PID *pid,float setValue,float actualValue,float *total_output);
void positionPid(PID *pid,float setValue,float actualValue,float *total_output);
void fuzzy_adjust(PID *pid);
#endif
