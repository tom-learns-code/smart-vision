#ifndef __PID_H
#define __PID_H

typedef struct {
    float Target;       // 目标值
    float Actual;       // 实际值
    float Out;          // 输出值
    float OutLast;      // 上次输出值
    
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    
    float Error0;       // 当前误差
    float Error1;       // 上次误差
    float Error2;       // 上上次误差
    
    float ErrorInt;     // 积分项累积误差
    
    float ErrorIntMax;  // 积分抗饱和上限：ErrorInt累积超过此值时截顶
	
    float OutMax;       // 输出上限
    float OutMin;       // 输出下限
} PID_t;

void incremental_pid_update(PID_t *p);
void position_pid_update(PID_t *p);

#endif
