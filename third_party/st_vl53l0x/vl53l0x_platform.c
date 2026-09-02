/**
  *****************************************************************************
  * @file     vl53l0x_platform.c
  * @brief    ST VL53L0X APIµÄAT32Æ½Ì¨ÊÊÅä²ã
  *****************************************************************************
  */

#include "vl53l0x_platform.h"

#include "at32f45x_board.h"
#include "tof_i2c3.h"

VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV Dev)
{
  (void)Dev;
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV Dev)
{
  (void)Dev;
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev,
                                 uint8_t index,
                                 uint8_t *pdata,
                                 uint32_t count)
{
  if((Dev == 0) || ((count > 0U) && (pdata == 0)))
  {
    return VL53L0X_ERROR_INVALID_PARAMS;
  }
  if(tof_i2c3_write(Dev->I2cDevAddr, index, pdata, count) != 0)
  {
    return VL53L0X_ERROR_CONTROL_INTERFACE;
  }
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev,
                                uint8_t index,
                                uint8_t *pdata,
                                uint32_t count)
{
  if((Dev == 0) || (pdata == 0) || (count == 0U))
  {
    return VL53L0X_ERROR_INVALID_PARAMS;
  }
  if(tof_i2c3_read(Dev->I2cDevAddr, index, pdata, count) != 0)
  {
    return VL53L0X_ERROR_CONTROL_INTERFACE;
  }
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev,
                             uint8_t index,
                             uint8_t data)
{
  return VL53L0X_WriteMulti(Dev, index, &data, 1U);
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev,
                             uint8_t index,
                             uint16_t data)
{
  uint8_t buffer[2];

  buffer[0] = (uint8_t)(data >> 8);
  buffer[1] = (uint8_t)data;
  return VL53L0X_WriteMulti(Dev, index, buffer, 2U);
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev,
                              uint8_t index,
                              uint32_t data)
{
  uint8_t buffer[4];

  buffer[0] = (uint8_t)(data >> 24);
  buffer[1] = (uint8_t)(data >> 16);
  buffer[2] = (uint8_t)(data >> 8);
  buffer[3] = (uint8_t)data;
  return VL53L0X_WriteMulti(Dev, index, buffer, 4U);
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev,
                                 uint8_t index,
                                 uint8_t and_data,
                                 uint8_t or_data)
{
  uint8_t data;
  VL53L0X_Error status;

  status = VL53L0X_RdByte(Dev, index, &data);
  if(status == VL53L0X_ERROR_NONE)
  {
    data = (uint8_t)((data & and_data) | or_data);
    status = VL53L0X_WrByte(Dev, index, data);
  }
  return status;
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev,
                             uint8_t index,
                             uint8_t *data)
{
  return VL53L0X_ReadMulti(Dev, index, data, 1U);
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev,
                             uint8_t index,
                             uint16_t *data)
{
  uint8_t buffer[2];
  VL53L0X_Error status;

  if(data == 0)
  {
    return VL53L0X_ERROR_INVALID_PARAMS;
  }
  status = VL53L0X_ReadMulti(Dev, index, buffer, 2U);
  if(status == VL53L0X_ERROR_NONE)
  {
    *data = (uint16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
  }
  return status;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev,
                              uint8_t index,
                              uint32_t *data)
{
  uint8_t buffer[4];
  VL53L0X_Error status;

  if(data == 0)
  {
    return VL53L0X_ERROR_INVALID_PARAMS;
  }
  status = VL53L0X_ReadMulti(Dev, index, buffer, 4U);
  if(status == VL53L0X_ERROR_NONE)
  {
    *data = ((uint32_t)buffer[0] << 24) |
            ((uint32_t)buffer[1] << 16) |
            ((uint32_t)buffer[2] << 8) |
            (uint32_t)buffer[3];
  }
  return status;
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
  (void)Dev;
  delay_ms(1U);
  return VL53L0X_ERROR_NONE;
}
