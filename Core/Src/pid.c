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
#define DKP1 -0.4   
#define DKP2 -0.2 
#define DKP3 0.2    
#define DKP4 0.4    

#define DKI1 -0.03    
#define DKI2 -0.01     
#define DKI3 0.01 
#define DKI4 0.03   	 

#define DKD1 -0.2
#define DKD2 -0.1     
#define DKD3 0.1    	 
#define DKD4 0.2

#define ZERO 0

KPID rule_table[5][5] = {
//             ec=NB 偏差快速减     ec=NS               ec=ZR              ec=PS                ec=PB 偏差快速增大
/*e=NB*/{ {DKP1, DKI1, DKD4}, {DKP2, DKI2, DKD3}, {DKP1,    0,    0}, {   0, DKI3, DKD2}, {DKP3, DKI4, DKD1} },//实际远高于目标，需降温
/*e=NS*/{ {DKP2, DKI2, DKD3}, {   0, 		0, 		0}, {		0, DKI3, DKD2}, {DKP3, DKI4, DKD1}, {DKP4, DKI4, DKD1} },//实际略高于目标
/*e=ZR*/{ {		0, DKI4, DKD1}, {		0, DKI3, DKD2}, {		0, 		0, 		0}, {		0, DKI2, DKD3}, {		0, DKI1, DKD4} },//无偏差
/*e=PS*/{ {DKP1, DKI1, DKD4}, {DKP2, DKI2, DKD3}, {DKP3, DKI3, 		0}, {DKP4, DKI4, 		0}, {DKP4, 		0, DKD2} },//实际略低于目标
/*e=PB*/{ {DKP1, 		0, DKD4}, {DKP2, 		0, DKD3}, {DKP3, DKI3, 		0}, {DKP4, DKI4, 		0}, {DKP4, DKI4, DKD1} },//实际远低于目标，需升温
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
#ifdef FUZZY_ADJUST
		fuzzy_adjust(pid);
#endif	
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
#ifdef FUZZY_ADJUST
		fuzzy_adjust(pid);
#endif
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
// 温度误差e的三角形隶属度函数
float e_NB_cal(float e_val) {
    // NB（负大）：左边界e_NB，右边界e_NS，隶属度随e_val增大而线性减小
    if (e_val <= e_NB) return 1.0f;       // 完全属于NB
    if (e_val >= e_NS) return 0.0f;       // 完全不属于NB
    return (e_NS - e_val) / (e_NS - e_NB); // 线性下降段
}

float e_NS_cal(float e_val) {
    // NS（负小）：左边界e_NB，顶点e_NS，右边界e_ZR
    if (e_val <= e_NB || e_val >= e_ZR) return 0.0f; // 完全不属于NS
    if (e_val <= e_NS) {
        // 左半段：从e_NB到e_NS，隶属度线性上升
        return (e_val - e_NB) / (e_NS - e_NB);
    } else {
        // 右半段：从e_NS到e_ZR，隶属度线性下降
        return (e_ZR - e_val) / (e_ZR - e_NS);
    }
}

float e_ZR_cal(float e_val) {
    // ZR（零）：左边界e_NS，顶点e_ZR，右边界e_PS
    if (e_val <= e_NS || e_val >= e_PS) return 0.0f; // 完全不属于ZR
    if (e_val <= e_ZR) {
        // 左半段：从e_NS到e_ZR，隶属度线性上升
        return (e_val - e_NS) / (e_ZR - e_NS);
    } else {
        // 右半段：从e_ZR到e_PS，隶属度线性下降
        return (e_PS - e_val) / (e_PS - e_ZR);
    }
}

float e_PS_cal(float e_val) {
    // PS（正小）：左边界e_ZR，顶点e_PS，右边界e_PB
    if (e_val <= e_ZR || e_val >= e_PB) return 0.0f; // 完全不属于PS
    if (e_val <= e_PS) {
        // 左半段：从e_ZR到e_PS，隶属度线性上升
        return (e_val - e_ZR) / (e_PS - e_ZR);
    } else {
        // 右半段：从e_PS到e_PB，隶属度线性下降
        return (e_PB - e_val) / (e_PB - e_PS);
    }
}

float e_PB_cal(float e_val) {
    // PB（正大）：左边界e_PS，右边界e_PB，隶属度随e_val增大而线性增大
    if (e_val <= e_PS) return 0.0f;       // 完全不属于PB
    if (e_val >= e_PB) return 1.0f;       // 完全属于PB
    return (e_val - e_PS) / (e_PB - e_PS); // 线性上升段
}

// 温度误差变化率ec的三角形隶属度函数
float ec_NB_cal(float ec_val) {
    // NB（负大）：左边界ec_NB，右边界ec_NS
    if (ec_val <= ec_NB) return 1.0f;
    if (ec_val >= ec_NS) return 0.0f;
    return (ec_NS - ec_val) / (ec_NS - ec_NB);
}

float ec_NS_cal(float ec_val) {
    // NS（负小）：左边界ec_NB，顶点ec_NS，右边界ec_ZR
    if (ec_val <= ec_NB || ec_val >= ec_ZR) return 0.0f;
    if (ec_val <= ec_NS) {
        return (ec_val - ec_NB) / (ec_NS - ec_NB);
    } else {
        return (ec_ZR - ec_val) / (ec_ZR - ec_NS);
    }
}

float ec_ZR_cal(float ec_val) {
    // ZR（零）：左边界ec_NS，顶点ec_ZR，右边界ec_PS
    if (ec_val <= ec_NS || ec_val >= ec_PS) return 0.0f;
    if (ec_val <= ec_ZR) {
        return (ec_val - ec_NS) / (ec_ZR - ec_NS);
    } else {
        return (ec_PS - ec_val) / (ec_PS - ec_ZR);
    }
}

float ec_PS_cal(float ec_val) {
    // PS（正小）：左边界ec_ZR，顶点ec_PS，右边界ec_PB
    if (ec_val <= ec_ZR || ec_val >= ec_PB) return 0.0f;
    if (ec_val <= ec_PS) {
        return (ec_val - ec_ZR) / (ec_PS - ec_ZR);
    } else {
        return (ec_PB - ec_val) / (ec_PB - ec_PS);
    }
}

float ec_PB_cal(float ec_val) {
    // PB（正大）：左边界ec_PS，右边界ec_PB
    if (ec_val <= ec_PS) return 0.0f;
    if (ec_val >= ec_PB) return 1.0f;
    return (ec_val - ec_PS) / (ec_PB - ec_PS);
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
