/*
 * i2c.c
 *
 *  Created on: 2014/09/10
 *      Author: Koji Nagakawa
 */
#include "stm32f10x.h"

extern uint8_t i2c_write(uint8_t, uint8_t, uint8_t *, uint8_t);
extern uint8_t i2c_read(uint8_t, uint8_t, uint8_t *, uint8_t);

uint8_t i2c_write(uint8_t address, uint8_t reg_adrs, uint8_t *d, uint8_t len)
{
  int i, k;
  I2C_GenerateSTART(I2C1, ENABLE);

  for(i = 0; !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT); i++){
      if(i > 1000){
          I2C_GenerateSTOP(I2C1, ENABLE);
          return -1;
      }
  }

  I2C_Send7bitAddress(I2C1, address, I2C_Direction_Transmitter);

  for(i = 0; !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED); i++){
      if(i > 1000){
          I2C_GenerateSTOP(I2C1, ENABLE);
          return -2;
      }
  }

  I2C_SendData(I2C1, reg_adrs);
  for(i = 0; !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED); i++){
      if(i > 1000){
          I2C_GenerateSTOP(I2C1, ENABLE);
          return -3;
      }
  }

  for(k = 0; k < len; k++){
    I2C_SendData(I2C1, *d);

    for(i = 0; !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED); i++){
        if(i > 1000){
            I2C_GenerateSTOP(I2C1, ENABLE);
            return -4;
        }
    }
  }

  I2C_GenerateSTOP(I2C1, ENABLE);

  return 0;
}

uint8_t i2c_read(uint8_t address, uint8_t reg_adrs, uint8_t *d, uint8_t len)
{
  uint16_t i;
  uint8_t k;

  for(i = 0; I2C_CheckEvent(I2C1, I2C_FLAG_BUSY); i++){
      if(i > 1000){
          I2C_GenerateSTOP(I2C1, ENABLE);
          return -1;
      }
  }

  I2C_GenerateSTART(I2C1, ENABLE);

  for(i = 0; !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT); i++){
    if(i > 1000){
        I2C_GenerateSTOP(I2C1, ENABLE);
        return -2;
    }
  }

  I2C_Send7bitAddress(I2C1, address, I2C_Direction_Transmitter);

  for(i = 0; !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED); i++){
      if(i > 1000){
          I2C_GenerateSTOP(I2C1, ENABLE);
          return -3;
      }
  }

  I2C_SendData(I2C1, reg_adrs);

  for(i = 0; !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED); i++){
      if(i > 1000){
          I2C_GenerateSTOP(I2C1, ENABLE);
          return -4;
      }
  }

  I2C_GenerateSTART(I2C1, ENABLE);

  for(i = 0; !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT); i++){
      if(i > 1000){
          I2C_GenerateSTOP(I2C1, ENABLE);
          return -5;
      }
  }

  I2C_Send7bitAddress(I2C1, address, I2C_Direction_Receiver);

  for(i = 0; !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED); i++){
      if(i > 1000){
          I2C_GenerateSTOP(I2C1, ENABLE);
          return -6;
      }
  }

  for(k = 0; k < len; k++){
      if(k + 1 == len){
          I2C_AcknowledgeConfig(I2C1, DISABLE);
          I2C_GenerateSTOP(I2C1, ENABLE);
      }
      for(i = 0; !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED); i++){
          if(i > 1000){
              I2C_GenerateSTOP(I2C1, ENABLE);
              return -7;
          }
      }
      *(d + k) = I2C_ReceiveData(I2C1);
  }

  I2C_AcknowledgeConfig(I2C1, ENABLE);
  return 0;
}
