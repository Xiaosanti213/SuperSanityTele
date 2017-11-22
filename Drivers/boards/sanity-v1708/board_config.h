/**
 * @file board_config.h
 *
 * sanity_controller-v1708 �ڲ���������
 * 
 */
 #ifndef _BOARD_CONFIG_H
 #define _BOARD_CONFIG_H



 #include "stm32f10x.h"


 
 /**
  * ͨ�ö˿ڣ��������ú�
	*
	*/
 #define						GPIO_CLK_Config_FUN							RCC_APB2PeriphClockCmd
 // ��Ϊͬʱ��Ҫ��������ҲҪ�������GPIOʱ�� �궨��Ҫǰһ�����з�
 #define						All_Periph_GPIO_CLK								(RCC_APB2Periph_AFIO|\
																						RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|\
																								RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD)
 
 
 
 
 
 
 

 /************************************************************************************
 * 
 * 10~13 (PA0~PA3)��������
 *
 * ����: ����TIM2ͨ�ö�ʱ�������PWM
 *   
 ************************************************************************************/

 #define						PWM_TIM											TIM2
 
 // ȷ����������λ��APB1��������Ӧ����ʱ��
 #define						PWM_TIM_APB1Clock_FUN							RCC_APB1PeriphClockCmd
 #define						PWM_TIM_CLK										RCC_APB1Periph_TIM2

 
 // PA0~PA3������Ҫ��ӳ�䵽TIM2��
 #define						PWM_TIM_REMAP_FUN()								GPIO_PinRemapConfig(GPIO_FullRemap_TIM2,ENABLE)
 
 // �˿���������
 #define						PWM_TIM_PORT									GPIOA
 #define						PWM1_TIM_PIN									GPIO_Pin_0
 #define						PWM2_TIM_PIN									GPIO_Pin_1
 #define						PWM3_TIM_PIN									GPIO_Pin_2
 #define						PWM4_TIM_PIN									GPIO_Pin_3
 
 // ��ʼ��ʱ�����ýṹ�庯��
 #define						PWM_TIM_TimeBaseInit_FUN							TIM_TimeBaseInit
 
 // ��ʼ��TIM2���ĸ�ͨ��CH1~CH4�Ƚ�����ṹ�庯��
 #define						PWM_TIM_CH1_Init_FUN  							TIM_OC1Init
 #define						PWM_TIM_CH2_Init_FUN  							TIM_OC2Init
 #define						PWM_TIM_CH3_Init_FUN  							TIM_OC3Init
 #define						PWM_TIM_CH4_Init_FUN  							TIM_OC4Init
 
 // ʹ��CCR�ȽϼĴ���Ԥװ�غ���,�Ӷ������ⲿ����ռ�ձ�
 #define						PWM_TIM_CH1_DUTY_EN_FUN()						TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable)
 #define						PWM_TIM_CH2_DUTY_EN_FUN()						TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable)
 #define						PWM_TIM_CH3_DUTY_EN_FUN()						TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable)
 #define						PWM_TIM_CH4_DUTY_EN_FUN()						TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable)

 // �ȽϼĴ�����װ
 #define						PWM_TIM_CH1_CCR									CCR1		
 #define						PWM_TIM_CH2_CCR									CCR2
 #define						PWM_TIM_CH3_CCR									CCR3
 #define						PWM_TIM_CH4_CCR									CCR4
 
 
 
 
 
 
 
 
 
 /************************************************************************************
 * 
 * 14~18 (PA4~PA7,PB0)��������
 *
 * ����: SPI1�������߽���ģ��
 *   
 ************************************************************************************/
 #define						RC_SPI											SPI1
 
 // ȷ����������λ��APB2��������Ӧ����ʱ��
 #define						RC_SPI_APB2Clock_FUN							RCC_APB2PeriphClockCmd
 #define						RC_SPI_CLK										RCC_APB2Periph_SPI1
 
 
 // �˿���������
 #define						RC_SPI_PORT										      GPIOA
 #define						RC_SPI_NSS_PIN									    GPIO_Pin_4
 #define						RC_SPI_CLK_PIN									    GPIO_Pin_5
 #define						RC_SPI_MISO_PIN									    GPIO_Pin_6
 #define						RC_SPI_MOSI_PIN   							    GPIO_Pin_7
 
 #define						RC_SPI_INT_CE_PORT								GPIOB
 #define						RC_SPI_INT_PIN									  GPIO_Pin_0 
 #define						RC_SPI_CE_PIN									    GPIO_Pin_1
 
 // ��ʼ��SPI���ýṹ�庯��
 #define						RC_SPI_Init_FUN						  			SPI_Init

 // ʹ����������ߵ͵�ƽ���� ����ߵ͵�ƽ���
 #define  					RC_SPI_NSS_LOW_FUN()     						GPIO_ResetBits( RC_SPI_PORT, RC_SPI_NSS_PIN )
 #define  					RC_SPI_NSS_HIGH_FUN()    						GPIO_SetBits( RC_SPI_PORT, RC_SPI_NSS_PIN )
 #define						RC_SPI_CE_LOW_FUN()							  	GPIO_ResetBits( RC_SPI_INT_CE_PORT, RC_SPI_CE_PIN )
 #define						RC_SPI_CE_HIGH_FUN()								GPIO_SetBits( RC_SPI_INT_CE_PORT, RC_SPI_CE_PIN )
 #define						RC_SPI_INT_SCAN_FUN()								GPIO_ReadInputDataBit( RC_SPI_INT_CE_PORT, RC_SPI_INT_PIN )
 #define						RC_SPI_INT_LOW											0				





/************************************************************************************
 * 
 * 46, 21~22 (PB9~PB11)��������
 *
 * ����: I2C2����MS5611ģ��
 *   
 ************************************************************************************/
 #define						MS5611_I2C										I2C2
 
 // ȷ����������λ��APB1��������Ӧ����ʱ��
 #define						MS5611_I2C_APB1Clock_FUN						RCC_APB1PeriphClockCmd
 #define						MS5611_I2C_CLK						    			RCC_APB1Periph_I2C2
 
 // PB5~PB7������Ҫ��ӳ�䵽TIM2��
 #define						MS5611_I2C_REMAP_FUN()							GPIO_PinRemapConfig(GPIO_Remap_I2C2,ENABLE) 
 
 // �˿���������
 #define						MS5611_I2C_PORT									  GPIOB
 #define						MS5611_I2C_INT_PIN								GPIO_Pin_9
 #define						MS5611_I2C_SCL_PIN								GPIO_Pin_10
 #define						MS5611_I2C_SDA_PIN								GPIO_Pin_11
 
 // ��ʼ��SPI���ýṹ�庯��
 #define						MS5611_I2C_Init_FUN							  	I2C_Init					 






/************************************************************************************
 * 
 * 30~31 (PA9~PA10)��������
 *
 * ����: ���ô��ڹ�����¼�������ʹ��
 *   
 ************************************************************************************/
 
 #define						DEBUG_USART													USART1
 
 // ȷ����������λ��APB2��������Ӧ����ʱ��
 #define						DEBUG_USART_APB2Clock_FUN						RCC_APB2PeriphClockCmd
 #define						DEBUG_USART_CLK											RCC_APB2Periph_USART1
 
 // PA9~PA10������Ҫ��ӳ�䵽TIM2��
 #define						DEBUG_USART_REMAP_FUN()							GPIO_PinRemapConfig(GPIO_Remap_USART1,ENABLE) 
 
 // �˿���������
 #define						DEBUG_USART_PORT										GPIOA
 #define						DEBUG_USART_TX_PIN									GPIO_Pin_9
 #define						DEBUG_USART_RX_PIN									GPIO_Pin_10
 
 // ��ʼ��SPI���ýṹ�庯��
 #define						DEBUG_USART_Init_FUN							  USART_Init	
 
 // �жϷ�����









/************************************************************************************
 * 
 * 41~43 (PB5~PB7)��������
 *
 * ����: I2C1����MPU6050ģ��
 *   
 ************************************************************************************/
 #define						MPU6050_I2C													I2C1
 
 // ȷ����������λ��APB1��������Ӧ����ʱ��
 #define						MPU6050_I2C_APB1Clock_FUN						RCC_APB1PeriphClockCmd
 #define						MPU6050_I2C_CLK											RCC_APB1Periph_I2C1
 
 // PB5~PB7������Ҫ��ӳ�䵽TIM2��
 #define						MPU6050_I2C_REMAP_FUN()							GPIO_PinRemapConfig(GPIO_Remap_I2C1,ENABLE) 
 
 // �˿���������
 #define						MPU6050_I2C_PORT										GPIOB
 #define						MPU6050_I2C_INT_PIN									GPIO_Pin_5
 #define						MPU6050_I2C_SCL_PIN									GPIO_Pin_6
 #define						MPU6050_I2C_SDA_PIN       					GPIO_Pin_7
 
 // ��ʼ��SPI���ýṹ�庯��
 #define						MPU6050_I2C_Init_FUN								I2C_Init








/************************************************************************************
 * 
 * 
 *
 * ����: ��ʼ����������
 *   
 ************************************************************************************/

  void gpio_clk_config(void);

  void pwm_tim_gpio_config(void);
  void pwm_tim_config(void);

  void rc_spi_gpio_config(void);
  void rc_spi_config(void);

  void ms5611_i2c_gpio_config(void);
  void ms5611_i2c_config(void);
 
  void debug_usart_gpio_config(void);
  void debug_usart_config(void);
 
  void mpu6050_i2c_gpio_config(void);
  void mpu6050_i2c_config(void);
 
 







 
 
 #endif




