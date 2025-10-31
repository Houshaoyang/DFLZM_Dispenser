#include <stdlib.h>
#include <string.h>
#include "pid.h"

// 定义模糊集序号宏
typedef enum{
	NB,   // 负大
	NS,  // 负小
	ZR,  // 零
	PS,  // 正小
	PB,  // 正大
	FUZZY_NUM
}fuzzy_num;

// 定义温度误差e隶属界限
#define e_NB -10  // 
#define e_NS -2   // 
#define e_ZR 0    // 
#define e_PS 2    // 
#define e_PB 10   // 

// 定义温度误差变化率ec隶属界限
#define ec_NB -5   // 
#define ec_NS -1   // 
#define ec_ZR 0    // 
#define ec_PS 1    // 
#define ec_PB 5    // 


/*******模糊规则表：每个元素是对应隶属度的调整输出[kp, ki, kd]*********/
// - Δkp  --（ -0.2 ）；-(-0.1)；+(0.2 )；++(0.4)
// - Δki  --（      ）；-(-0.1)；+(0.05)；++(0.1)
// - Δkd  --（      ）；-(-0.1)；+(0.1 )；++(0.2)
// 定义调整Δkp、Δki、Δkd的程度
#define DKP1 -0.2   
#define DKP2 -0.1 
#define DKP3 0.2    
#define DKP4 0.4    

#define DKI1 -0.1    
#define DKI2 -0.05     
#define DKI3 0.05 
#define DKI4 0.1   	 

#define DKD1 -0.1
#define DKD2 -0.05     
#define DKD3 0.1    	 
#define DKD4 0.2

#define ZERO 0

KPID rule_table[5][5] = {
//            ec=NB        ec=NS          ec=ZR        ec=PS         ec=PB
/*e=NB*/{ {NB, NB, PS}, {NB, NB, PS}, {NS, ZR, ZR}, {ZR, NB, NB}, {PB, PB, NS} },
/*e=NS*/{ {NB, NB, PS}, {ZR, ZR, ZR}, {ZR, PS, NS}, {PB, PS, NS}, {PB, PB, NS} },
/*e=ZR*/{ {NB, NB, NB}, {ZR, ZR, NB}, {NS, ZR, ZR}, {ZR, PS, PS}, {PB, NS, PS} },
/*e=PS*/{ {NB, NB, PS}, {NB, NB, PS}, {PB, PS, ZR}, {PB, PS, ZR}, {PB, ZR, NS} },
/*e=PB*/{ {NB, ZR, PS}, {PB, ZR, PS}, {PB, PS, PS}, {PB, PS, ZR}, {PB, PB, NS} }
};

// 隶属度计算值数组（示例值，需根据温度误差、温度误差变化率计算得到）
float e[5],ec[5];   // e[NB], e[NS], e[ZR], e[PS], e[PB];   ec[NB], ec[NS], ec[ZR], ec[PS], ec[PB]

/*pid初始化*/
void pid_init(double kp, double ki, double kd,ctrl_dir dir, PID *pid)
{
		memset(pid,0,sizeof(PID)); 
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
		pid->ctrl_dir = dir;
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
	
		fuzzy_adjust(pid);
	
    pid->P = pid->Kp*(pid->error-pid->errorPre);			//比例项
    pid->I = pid->Ki*pid->error;							//积分项
    pid->D = pid->Kd*(pid->error-2*pid->errorPre + pid->errorPrePre); 			//微分项
    increment = pid->P + pid->I + pid->D;   				//pid计算得到的输出增量
		if(pid->ctrl_dir == DRIECT){
			*total_output += increment;
		}else *total_output -= increment;
		
	
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

// 模糊逻辑：误差(e)的隶属度函数（三角形分布）
// 模糊集：负大(NB)、负小(NS)、零(ZR)、正小(PS)、正大(PB)
float e_NB_cal(float e){
    if (e <= -10) return 1.0;
    if (e >= -2) return 0.0;
    return (e + 10) / (-2 + 10); // 斜率为负
}

float e_NS_cal(float e) {
    if (e <= -10 || e >= 2) return 0.0;
    if (e <= -2) return (e + 10) / (-2 + 10);
    return (2 - e) / (2 + 2);
}
float e_ZR_cal(float e) {
    if (e <= -2 || e >= 2) return 0.0;
    return (e + 2) / (2 + 2);
}
float e_PS_cal(float e) {
    if (e <= -2 || e >= 10) return 0.0;
    if (e <= 2) return (e + 2) / (2 + 2);
    return (10 - e) / (10 - 2);
}
float e_PB_cal(float e) {
    if (e <= 2) return 0.0;
    if (e >= 10) return 1.0;
    return (e - 2) / (10 - 2);
}

// 模糊逻辑：误差变化率(ec)的隶属度函数
float ec_NB_cal(float ec) {
    if (ec <= -5) return 1.0;
    if (ec >= -1) return 0.0;
    return (ec + 5) / (-1 + 5);
}
float ec_NS_cal(float ec) {
    if (ec <= -5 || ec >= 1) return 0.0;
    if (ec <= -1) return (ec + 5) / (-1 + 5);
    return (1 - ec) / (1 + 1);
}
float ec_ZR_cal(float ec) {
    if (ec <= -1 || ec >= 1) return 0.0;
    return (ec + 1) / (1 + 1);
}
float ec_PS_cal(float ec) {
    if (ec <= -1 || ec >= 5) return 0.0;
    if (ec <= 1) return (ec + 1) / (1 + 1);
    return (5 - ec) / (5 - 1);
}
float ec_PB_cal(float ec) {
    if (ec <= 1) return 0.0;
    if (ec >= 5) return 1.0;
    return (ec - 1) / (5 - 1);
}

// 定义隶属度计算的函数指针
typedef float (*Membership_cal)(float e);
// 模糊逻辑：误差变化率(e)的隶属度函数指针数组
const Membership_cal Membership_cal_e[] =
{
	e_NB_cal,    		 			
	e_NS_cal,         			
	e_ZR_cal,		
	e_PS_cal,
	e_PB_cal,	
	NULL
};

// 模糊逻辑：误差变化率(ec)的隶属度函数指针数组
const Membership_cal Membership_cal_ec[] =
{
	ec_NB_cal,    		 			
	ec_NS_cal,         			
	ec_ZR_cal,		
	ec_PS_cal,
	ec_PB_cal,	
	NULL
};

void fuzzy_adjust(PID *pid){
	int i,j;
	float _e = pid->error;
	float _ec = pid->error - pid->errorPre;
	KPID _dpid,sum_dpid={0,0,0};
	for(i=0;i<FUZZY_NUM;i++){
		e[i]=Membership_cal_e[i](_e);			//计算所有的误差隶属度值
		ec[i]=Membership_cal_ec[i](_ec);	//计算所有的误差变化率隶属度值
	}
	
	for(i=0;i<FUZZY_NUM;i++)         //根据模糊规则表计算dKp、dKi、dKd
	{
		if(e[i] ==0) continue;
		for(j=0;j<FUZZY_NUM;j++)
		{
			if(ec[j] ==0) continue;
			_dpid = rule_table[i][j];
			sum_dpid.Kp += e[i]*ec[j]*_dpid.Kp;
			sum_dpid.Ki += e[i]*ec[j]*_dpid.Ki;
			sum_dpid.Kd += e[i]*ec[j]*_dpid.Kd;
		}
	}
	
	pid->Kp = KP_BASE + sum_dpid.Kp;//根据模糊控制策略结果调整PID的Kp、Ki、Kd参数
	pid->Ki = KI_BASE + sum_dpid.Ki;
	pid->Kd = KD_BASE + sum_dpid.Kd;
}
