#include "zf_common_headfile.h"
#include "pid.h"

// 增量式PID更新函数
void incremental_pid_update(PID_t *p)
{
    // 保存上一次和上上次的误差
    p->Error2 = p->Error1;
    p->Error1 = p->Error0;
    
    // 计算当前误差
    p->Error0 = p->Target - p->Actual;
    
    // 计算控制量增量
    float deltaOut = p->Kp * (p->Error0 - p->Error1)
                   + p->Ki * p->Error0
                   + p->Kd * (p->Error0 - 2 * p->Error1 + p->Error2);
    
    // 更新输出值（加上增量）上次输出 + 这次增量 = 这次输出
		//输出不能超过±10000（PWM满量程）
    p->Out = p->OutLast + deltaOut;
    
    // 输出限幅
    if (p->Out > p->OutMax) {p->Out = p->OutMax;}
    if (p->Out < p->OutMin) {p->Out = p->OutMin;}
    
    // 保存当前输出作为下次的上次输出
    p->OutLast = p->Out;
}


// 位置式PID更新函数
void position_pid_update(PID_t *p)
{
    // 保存上一次的误差
    p->Error1 = p->Error0;
    
    // 计算当前误差
    p->Error0 = p->Target - p->Actual;
    
    // 累积误差（积分项）+ 积分抗饱和：始终积分，但限制积分累积上限
    if (p->Ki != 0)
    {
        p->ErrorInt += p->Error0;
        // 积分抗饱和：限制积分累积量，防止过度积分导致超调
        if (p->ErrorIntMax > 0)
        {
            if (p->ErrorInt > p->ErrorIntMax)  p->ErrorInt = p->ErrorIntMax;
            if (p->ErrorInt < -p->ErrorIntMax) p->ErrorInt = -p->ErrorIntMax;
        }
    }
    else
    {
        p->ErrorInt = 0;
    }
    
    // 计算完整控制量输出（位置式PID公式）
    p->Out = p->Kp * p->Error0
           + p->Ki * p->ErrorInt
           + p->Kd * (p->Error0 - p->Error1);
    
    // 输出限幅
    if (p->Out > p->OutMax) {p->Out = p->OutMax;}
    if (p->Out < p->OutMin) {p->Out = p->OutMin;}
}
