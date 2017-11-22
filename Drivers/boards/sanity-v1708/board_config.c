/**
 * @file board_config.c
 *
 * sanity-v1708 外设初始化函数
 * 
 */
 
 
#include "board_config.h"

#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h" 
#include "stm32f10x_i2c.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_spi.h"
 
 
 
/************************************************************************************
 * 
 * 名称: gpio_clk_config
 *
 * 描述: 使能全部外设gpio时钟
 *   
 ************************************************************************************/
 
  void gpio_clk_config(void)
 {
	 
	 // 使能全部外设GPIO时钟
	 GPIO_CLK_Config_FUN(All_Periph_GPIO_CLK, ENABLE); 
 
 }
 
 
 
 
 
/************************************************************************************
 * 
 * 名称: pwm_tim_gpio_config
 *
 * 描述: 配置TIM2对应的PA0~PA3引脚
 *   
 ************************************************************************************/

 void pwm_tim_gpio_config(void)
{
	
	
	// 配置GPIO初始化结构体
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// 复用推挽输出
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = PWM1_TIM_PIN;
	
	GPIO_Init(PWM_TIM_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = PWM2_TIM_PIN;
    GPIO_Init(PWM_TIM_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = PWM3_TIM_PIN;
    GPIO_Init(PWM_TIM_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = PWM4_TIM_PIN;
    GPIO_Init(PWM_TIM_PORT, &GPIO_InitStructure);
	
	// 应该置位0

}





/************************************************************************************
 * 
 * 名称: pwm_tim_config
 *
 * 描述: 配置TIM2输出的PWM信号的模式，如周期，极性。
 *   
 ************************************************************************************/

 void pwm_tim_config(void)
{
	
	
	// 配置TIM2初始化结构体
	
	
	// 对于通用TIM的输出模式，只需要配置其中两个:
	// 配置时基初始化结构体	,输出比较初始化结构体
	TIM_OCInitTypeDef   TIM_OCBaseInitStructure;
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseInitStructure;
	
	
	// 使能时钟
	PWM_TIM_APB1Clock_FUN(PWM_TIM_CLK, ENABLE);
	
	// 不分频
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	// 向上计数
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	// 定时器周期（影子寄存器ARR的值）和下面的时钟构成周期
	TIM_TimeBaseInitStructure.TIM_Period = (8-1);
	// 定时器预分频的值, 配置驱动时钟1ms
	TIM_TimeBaseInitStructure.TIM_Prescaler = (9-1);
	// TIM_RepetitionCounter只存在与高级定时器当中, 无需设置
	
	PWM_TIM_TimeBaseInit_FUN(PWM_TIM, &TIM_TimeBaseInitStructure);
	
	
		
	TIM_OCBaseInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	// 使能输出
	TIM_OCBaseInitStructure.TIM_OutputState = TIM_OutputState_Enable ;
	// 设置初始PWM脉冲宽度0
	TIM_OCBaseInitStructure.TIM_Pulse = 0;
	// 定时器计数值小于CCR_Val时有效电平为低电平
	TIM_OCBaseInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low ;
	
	PWM_TIM_CH1_Init_FUN(TIM2, &TIM_OCBaseInitStructure);
	PWM_TIM_CH2_Init_FUN(TIM2, &TIM_OCBaseInitStructure);
	PWM_TIM_CH3_Init_FUN(TIM2, &TIM_OCBaseInitStructure);
	PWM_TIM_CH4_Init_FUN(TIM2, &TIM_OCBaseInitStructure);
	
	// 单独配置占空比功能使能	
	PWM_TIM_CH1_DUTY_EN_FUN();
	PWM_TIM_CH2_DUTY_EN_FUN();
	PWM_TIM_CH3_DUTY_EN_FUN();
	PWM_TIM_CH4_DUTY_EN_FUN();

}







/************************************************************************************
 * 
 * 名称: rc_spi_gpio_config
 *
 * 描述: 配置gpio对应的PA4~PA7，PB0引脚
 *   
 ************************************************************************************/

 void rc_spi_gpio_config(void)
{
	
	
	// 配置GPIO初始化结构体
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// 复用推挽输出
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Pin = RC_SPI_CLK_PIN;
	GPIO_Init(RC_SPI_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = RC_SPI_MISO_PIN;
	GPIO_Init(RC_SPI_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = RC_SPI_MOSI_PIN;
	GPIO_Init(RC_SPI_PORT, &GPIO_InitStructure);
	
	// INT, CE, NSS引脚配置成普通输出模式
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = RC_SPI_NSS_PIN;
	GPIO_Init(RC_SPI_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = RC_SPI_INT_PIN;
	GPIO_Init(RC_SPI_INT_CE_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = RC_SPI_CE_PIN;
	GPIO_Init(RC_SPI_INT_CE_PORT, &GPIO_InitStructure);

	

}








/************************************************************************************
 * 
 * 名称: rc_spi_config
 *
 * 描述: 配置无线模块与stm32c8t6的spi通信
 *   
 ************************************************************************************/

 void rc_spi_config(void)
{
	
	
	// 配置SPI初始化结构体
	SPI_InitTypeDef   RC_SPI_InitStructure;
	
	// 使能时钟
	RC_SPI_APB2Clock_FUN(RC_SPI_CLK, ENABLE);
	
	// 8分频，时钟9M，可以调节
	RC_SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	// SCK信号线第一个上升沿采集数据
	RC_SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	// SCK信号线空闲状态低电平
	RC_SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	// CRC校验多项式
	RC_SPI_InitStructure.SPI_CRCPolynomial = 7;
	// SPI通讯数据帧大小
	RC_SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b ;
	// 双线全双工工作模式
	RC_SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex ;
	// 高位先行
	RC_SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB ;
	// 芯片主机模式
	RC_SPI_InitStructure.SPI_Mode = SPI_Mode_Master ;

	SPI_Init(RC_SPI, &RC_SPI_InitStructure);
	// 使能SPI1外设
	SPI_Cmd(RC_SPI, ENABLE);

}





/************************************************************************************
 * 
 * 名称: ms5611_i2c_gpio_config
 *
 * 描述: 配置I2C对应的PB10~PB11引脚
 *   
 ************************************************************************************/

  void ms5611_i2c_gpio_config(void)
 {
	 
	 GPIO_InitTypeDef GPIO_InitStructure;
	 
	 
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 
	 // 中断引脚是普通输出模式
   GPIO_InitStructure.GPIO_Pin = MS5611_I2C_INT_PIN;
	 GPIO_Init(MS5611_I2C_PORT, &GPIO_InitStructure); 
	 
	 // 其他引脚配置复用模式
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	 GPIO_InitStructure.GPIO_Pin = MS5611_I2C_SCL_PIN;
	 GPIO_Init(MS5611_I2C_PORT, &GPIO_InitStructure); 
	 
	 GPIO_InitStructure.GPIO_Pin = MS5611_I2C_SDA_PIN;
	 GPIO_Init(MS5611_I2C_PORT, &GPIO_InitStructure); 
 
 
 }
 
 
 
 
 
 
 
 
 
 /************************************************************************************
 * 
 * 名称: ms5611_i2c_config
 *
 * 描述: 配置MS5611与主机通过I2C连接
 *   
 ************************************************************************************/

   void ms5611_i2c_config(void)
 {
	 
	 
	 	I2C_InitTypeDef  I2C_InitStructure;
	 
		// 使能时钟
		MS5611_I2C_APB1Clock_FUN(RCC_APB1Periph_I2C1, ENABLE);
		 
		// 设置可以发送响应信号
		I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
		// 设置从机地址位数
		I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
		// 设置SCL线高低电平占空比，一般要求不会很严格
		I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
		// 不需要在此处区分主从模式
		I2C_InitStructure.I2C_Mode  = I2C_Mode_I2C;
		// 设置通讯速率100kHz
		I2C_InitStructure.I2C_ClockSpeed = 100000;
		// 主机地址
		I2C_InitStructure.I2C_OwnAddress1 = 0x00;

		 
		I2C_Init(MS5611_I2C, &I2C_InitStructure); 
		 
		// 使能I2C1外设
		I2C_Cmd(MS5611_I2C, ENABLE);
 
 
 }
 
 
 
 
 
 
 /************************************************************************************
 * 
 * 名称: debug_usart_gpio_config
 *
 * 描述: 配置USART对应的PA9~PA10引脚
 *   
 ************************************************************************************/

  void debug_usart_gpio_config(void)
 {
	 
	 GPIO_InitTypeDef GPIO_InitStructure;
	 
	 // 其他引脚配置复用模式
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	 GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_PIN;
	 GPIO_Init(DEBUG_USART_PORT, &GPIO_InitStructure); 
	 
	 GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_PIN;
	 GPIO_Init(DEBUG_USART_PORT, &GPIO_InitStructure); 
 
 
 }
 
 
 
 
 
 
 
 
 
 /************************************************************************************
 * 
 * 名称: debug_usart_config
 *
 * 描述: 配置USART烧录程序调参
 *   
 ************************************************************************************/

   void debug_usart_config(void)
 {
	 
	 	 
   USART_InitTypeDef  USART_InitStructure;
	 
	 // 使能时钟
   DEBUG_USART_APB2Clock_FUN(DEBUG_USART_CLK, ENABLE);
	 
   // 设置波特率
   USART_InitStructure.USART_BaudRate = 115200;
   // 设置硬件流控制RTS和CTS无效
   USART_InitStructure.USART_Mode = (USART_Mode_Rx | USART_Mode_Tx);
   // 设置奇偶校验选择无校验
   USART_InitStructure.USART_Parity = USART_Parity_No;
   // 设置停止位
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   // 设置数据帧字长
   USART_InitStructure.USART_WordLength = USART_WordLength_8b ;
	 
   USART_Init(DEBUG_USART, &USART_InitStructure); 
	 
   // 使能I2C1外设
   USART_Cmd(DEBUG_USART, ENABLE);
 
 
 }



/************************************************************************************
 * 
 * 名称: mpu6050_i2c_gpio_config
 *
 * 描述: 配置I2C对应的PB5~PB7引脚
 *   
 ************************************************************************************/

  void mpu6050_i2c_gpio_config(void)
 {
	 
	 GPIO_InitTypeDef GPIO_InitStructure;
	 
	 
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 
	 // 中断引脚是普通输出模式
   GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_INT_PIN;
	 GPIO_Init(MPU6050_I2C_PORT, &GPIO_InitStructure); 
	 
	 // 其他引脚配置复用模式
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	 GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_SCL_PIN;
	 GPIO_Init(MPU6050_I2C_PORT, &GPIO_InitStructure); 
	 
	 GPIO_InitStructure.GPIO_Pin = MPU6050_I2C_SDA_PIN;
	 GPIO_Init(MPU6050_I2C_PORT, &GPIO_InitStructure); 
 
 
 }
 
 
 
 
 
 
 /************************************************************************************
 * 
 * 名称: mpu6050_i2c_config
 *
 * 描述: 配置与MPU6050传感器I2C连接
 *   
 ************************************************************************************/

   void mpu6050_i2c_config(void)
 {
	 
		 I2C_InitTypeDef  I2C_InitStructure;
		 
		 // 使能时钟
		 MPU6050_I2C_APB1Clock_FUN(RCC_APB1Periph_I2C1, ENABLE);
		 
		 // 设置可以发送响应信号
		 I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
		 // 设置从机地址位数
		 I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
		 // 设置SCL线高低电平占空比，一般要求不会很严格
		 I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
		 // 不需要在此处区分主从模式
		 I2C_InitStructure.I2C_Mode  = I2C_Mode_I2C;
		 // 设置通讯速率100kHz慢速模式
		 I2C_InitStructure.I2C_ClockSpeed = 100000;
		 // 主机地址
		 I2C_InitStructure.I2C_OwnAddress1 = 0x00;

	 
		 I2C_Init(MS5611_I2C, &I2C_InitStructure); 
		 
		 // 使能I2C2外设
		 I2C_Cmd(MS5611_I2C, ENABLE);
	 
 
 }
 
 
 
 
 

