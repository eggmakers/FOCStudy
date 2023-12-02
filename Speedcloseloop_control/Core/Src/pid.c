#include "pid.h"

float _P; //!< 比例增益(P环增益)
float _I; //!< 积分增益（I环增益）
float _D; //!< 微分增益（D环增益）
float _output_ramp;
float _limit;

float error_prev;             //!< 最后的跟踪误差值
float output_prev;            //!< 最后一个 pid 输出值
float integral_prev;          //!< 最后一个积分分量值
unsigned long timestamp_prev; //!< 上次执行时间戳

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

void PIDInit(float P, float I, float D, float ramp, float limit)
{
    _P = P;
    _I = I;
    _D = D;
    _output_ramp = ramp;
    _limit = limit;
    error_prev = 0.f;
    output_prev = 0.f;
    integral_prev = 0.f;
    timestamp_prev = SysTick->VAL;
}

float PIDOperator(float error)
{
    unsigned long timestamp_now = SysTick->VAL;
    float Ts = (timestamp_now - timestamp_prev) * 1e-6f;
    if (Ts <= 0 || Ts > 0.5f)
        Ts = 1e-3f;

    float proportional = _P * error; // P环
    float integral = integral_prev + _I * Ts * 0.5f * (error + error_prev);
    integral = _constrain(integral, -_limit, _limit);  // I环
    float derivative = _D * (error - error_prev) / Ts; // D环

    // 将P,I,D三环的计算值加起来
    float output = proportional + integral + derivative;
    output = _constrain(output, -_limit, _limit);

    if (_output_ramp > 0)
    {
        float output_rate = (output - output_prev) / Ts;
        if (output_rate > _output_ramp)
            output = output_prev + _output_ramp * Ts;
        else if (output_rate < -_output_ramp)
            output = output_prev - _output_ramp * Ts;
    }
    // 保存值（为了下一次循环）
    integral_prev = integral;
    output_prev = output;
    error_prev = error;
    timestamp_prev = timestamp_now;
    return output;
}
