#include "bsp_as5600.h"

static float angle_data_prev;      // 上次位置
static float full_rotation_offset; // 转过的整圈数
int32_t full_rotation;

void bsp_as5600Init(void)
{
  /* init i2c interface */

  /* init var */
  full_rotation_offset = 0;
  angle_data_prev = bsp_as5600GetRawAngle();
}

static int i2cWrite(uint8_t dev_addr, uint8_t *pData, uint32_t count)
{
  int status;
  int i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;

  status = HAL_I2C_Master_Transmit(&AS5600_I2C_HANDLE, dev_addr, pData, count, i2c_time_out);
  return status;
}

static int i2cRead(uint8_t dev_addr, uint8_t *pData, uint32_t count)
{
  int status;
  int i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;

  status = HAL_I2C_Master_Receive(&AS5600_I2C_HANDLE, (dev_addr | 1), pData, count, i2c_time_out);
  return status;
}

uint16_t bsp_as5600GetRawAngle(void)
{
  uint16_t raw_angle;
  uint8_t buffer[2] = {0};
  uint8_t raw_angle_register = AS5600_RAW_ANGLE_REGISTER;

  i2cWrite(AS5600_ADDR, &raw_angle_register, 1);
  i2cRead(AS5600_ADDR, buffer, 2);
  raw_angle = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
  return raw_angle;
}

float GetAngle_without_track()
{
  float angle = bsp_as5600GetRawAngle();
  angle = ((angle * 359) / 4095);
  return (angle * (_PI / 180));
}

float GetAngle(void)
{
  return (180 / _PI * bsp_as5600GetAngle());
}

float bsp_as5600GetAngle(void) // 弧度
{
  float angle_data = bsp_as5600GetRawAngle();

  float d_angle = angle_data - angle_data_prev;
  if (abs(d_angle) > (0.8 * AS5600_RESOLUTION))
  {
    full_rotation_offset += (d_angle > 0 ? -_2PI : _2PI);
  }
  angle_data_prev = angle_data;

  return (full_rotation_offset + (angle_data / (float)AS5600_RESOLUTION) * _2PI);
}
