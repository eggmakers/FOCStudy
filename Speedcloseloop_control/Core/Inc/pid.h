#ifndef PID_H
#define PID_H

#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "string.h"
#include <stdio.h>
#include "math.h"
#include "pid.h"

extern float _P; //!< 比例增益(P环增益)
extern float _I; //!< 积分增益（I环增益）
extern float _D; //!< 微分增益（D环增益）
extern float _output_ramp;
extern float _limit;

extern float error_prev;             //!< 最后的跟踪误差值
extern float output_prev;            //!< 最后一个 pid 输出值
extern float integral_prev;          //!< 最后一个积分分量值
extern unsigned long timestamp_prev; //!< 上次执行时间戳

void PIDInit(float P, float I, float D, float ramp, float limit);
float PIDOperator(float error);

#endif
