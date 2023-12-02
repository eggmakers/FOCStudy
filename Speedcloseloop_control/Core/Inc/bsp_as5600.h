#ifndef __BSP_AS5600_H
#define __BSP_AS5600_H

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

#define abs(x) ((x) > 0 ? (x) : -(x))
#define _2PI 6.28318530718f
#define _PI 3.14159265359f
#define _3PI_2 4.71238898038f

#define AS5600_I2C_HANDLE hi2c1

#define I2C_TIME_OUT_BASE 10
#define I2C_TIME_OUT_BYTE 1

/*--------------------------------*/
extern uint32_t angle_prev_ts;

#define AS5600_RAW_ADDR 0x36
#define AS5600_ADDR (AS5600_RAW_ADDR << 1)
#define AS5600_WRITE_ADDR (AS5600_RAW_ADDR << 1)
#define AS5600_READ_ADDR ((AS5600_RAW_ADDR << 1) | 1)

#define AS5600_RESOLUTION 4096 // 12bit Resolution

#define AS5600_RAW_ANGLE_REGISTER 0x0C

void bsp_as5600Init(void);
void Sensor_update(void);
uint16_t bsp_as5600GetRawAngle(void);
float bsp_as5600GetAngle(void);
float GetAngle(void);
float GetAngle_without_track(void);
float GetVelocity(void);

#endif /* __BSP_AS5600_H */
