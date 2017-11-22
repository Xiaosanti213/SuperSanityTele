/**
 * @file api_spi.c
 *
 * spi接口函数定义
 * 
 */

#include "stm32f10x.h"
#include "api_spi.h"
#include <stdio.h>


	





/**
  * 名称：spi_send_byte
  *
  * 描述：spi接口发送一个字节数据
  * 
  */

 uint8_t spi_send_byte(SPI_TypeDef* SPIx, uint8_t byte)
{
	
	// 发送数据最长等待时间
	uint8_t spi_wait_time = SPI_WAIT_TIMEOUT;
	// 发送数据寄存器是否为空
	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET)
	{
		// RESET表示事件没发生
		spi_wait_time--;
		if(spi_wait_time == 0)
		{
			// 返回回调函数，结束发送函数
			return spi_timeout_usercallback(0);
		}
	}
	// 写入数据寄存器，将要写入的数据写入发送缓冲区
	SPI_I2S_SendData(SPIx, byte);
	
	
	// 接收数据最长等待时间
	spi_wait_time = SPI_WAIT_TIMEOUT;
	
	// 事件SPI_I2S_FLAG_RXNE
	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET)
		{
			spi_wait_time--;
			if(spi_wait_time == 0)
			{
				return spi_timeout_usercallback(1);
			}
		}
	// 将数据从缓冲区读出
	return SPI_I2S_ReceiveData(SPIx);
		
}















/**
  * 名称：spi_timeout_usercallback
  *
  * 描述：等待超时回调函数
  * 
  */

  uint16_t spi_timeout_usercallback(uint8_t errorCode)
{
  // 等待超时后的处理,输出错误信息
  printf("SPI 等待超时!errorCode = ,%d\n", errorCode);
  return 0;
}


