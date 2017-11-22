/**
 * @file api_spi.h
 * 
 * spi�ӿں������� �궨��
 *
 */
 #include "stm32f10x.h"




 
 #ifndef _API_SPI_H
 #define _API_SPI_H
 
 // spi�ӿڷ��ͽ���һ���ֽ����ݵȴ�ʱ��
 #define SPI_WAIT_TIMEOUT 100;
 
 
 
  uint8_t spi_send_byte(SPI_TypeDef* SPIx, uint8_t byte);
  uint16_t spi_timeout_usercallback(uint8_t errorCode);
 
 
 
 
 #endif
 
 

