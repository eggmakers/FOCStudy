#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#include "i2c.h"
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "string.h"
#include <stdio.h>
#include "math.h"
#include "pid.h"
#include "lowpass_filter.h"
#include "bsp_as5600.h"

extern unsigned long filter_timestamp_prev; //!< ���ִ��ʱ���
extern float y_prev;                 //!< ��һ��ѭ���еĹ��˺��ֵ
extern float Tf;                     //!< ��ͨ�˲�ʱ�䳣��

void lowpass_filter_init(float time_constant);
float lowpass_filter_operator(float x);

#endif
