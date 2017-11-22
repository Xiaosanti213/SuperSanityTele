/**
 * @file nRF24L01.h
 *
 * nRF24L01 �����շ�ģ�����ָ�� �Ĵ���ӳ�� ��������
 * 
 */

#ifndef _NRF24L01_H
#define _NRF24L01_H

#include "stm32f10x.h"


//NRF24L01�Ĵ�����������
#define READ_REG_NRF         0x00  //�����üĴ���,��5λΪ�Ĵ�����ַ
#define WRITE_REG_NRF        0x20  //д���üĴ���,��5λΪ�Ĵ�����ַ
#define RD_RX_PAYLOAD        0x61  //��RX��Ч����,1~32�ֽ�
#define WR_TX_PAYLOAD        0xA0  //дTX��Ч����,1~32�ֽ�
#define FLUSH_TX  		       0xE1  //���TX FIFO�Ĵ���.����ģʽ����
#define FLUSH_RX             0xE2  //���RX FIFO�Ĵ���.����ģʽ���ã�ACK����ʱ��ִֹ��
#define REUSE_TX_PL          0xE3  //PTX�豸ʹ�ã�����ʹ����һ������,CEΪ��,���ݰ������Ϸ���
#define R_RX_PL_WID					 0x60  //��RX FIFO�Ĵ�������Ч���ݿ��
#define NOP                  0xFF  //�ղ���,����������״̬�Ĵ���	 

//SPI(NRF24L01)�Ĵ�����ַӳ��
#define CONFIG          0x00  //���üĴ�����ַ;bit0:1����ģʽ,0����ģʽ;bit1:��ѡ��;bit2:CRCģʽ;bit3:CRCʹ��;
                              //bit4:�ж�MAX_RT(�ﵽ����ط������ж�)ʹ��;bit5:�ж�TX_DSʹ��;bit6:�ж�RX_DRʹ��
#define EN_AA           0x01  //ʹ���Զ�Ӧ����  bit0~5,��Ӧͨ��0~5
#define EN_RXADDR       0x02  //���յ�ַ����,bit0~5,��Ӧͨ��0~5
#define SETUP_AW        0x03  //���õ�ַ���(��������ͨ��):bit1,0:00,3�ֽ�;01,4�ֽ�;02,5�ֽ�;
#define SETUP_RETR      0x04  //�����Զ��ط�;bit3:0,�Զ��ط�������;bit7:4,�Զ��ط���ʱ 250*x+86us
#define RF_CH           0x05  //RFͨ��,bit6:0,����ͨ��Ƶ��;
#define RF_SETUP        0x06  //RF�Ĵ���;bit3:��������(0:1Mbps,1:2Mbps);bit2:1,���书��;bit0:�������Ŵ�������
#define STATUS          0x07  //״̬�Ĵ���;bit0:TX FIFO����־;bit3:1,��������ͨ����(���:6);bit4,�ﵽ�����ط�
                              //bit5:���ݷ�������ж�;bit6:���������ж�;
#define MAX_TX  	      0x10  //�ﵽ����ʹ����ж�
#define TX_OK   	      0x20  //TX��������ж�
#define RX_OK   	      0x40  //���յ������ж�

#define OBSERVE_TX      0x08  //���ͼ��Ĵ���,bit7:4,���ݰ���ʧ������;bit3:0,�ط�������
#define CD              0x09  //�ز����Ĵ���,bit0,�ز����;
#define RX_ADDR_P0      0x0A  //����ͨ��0���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define RX_ADDR_P1      0x0B  //����ͨ��1���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define RX_ADDR_P2      0x0C  //����ͨ��2���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P3      0x0D  //����ͨ��3���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P4      0x0E  //����ͨ��4���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P5      0x0F  //����ͨ��5���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define TX_ADDR         0x10  //���͵�ַ(���ֽ���ǰ),ShockBurstTMģʽ��,RX_ADDR_P0��˵�ַ���
#define RX_PW_P0        0x11  //��������ͨ��0��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P1        0x12  //��������ͨ��1��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P2        0x13  //��������ͨ��2��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P3        0x14  //��������ͨ��3��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P4        0x15  //��������ͨ��4��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P5        0x16  //��������ͨ��5��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define FIFO_STATUS     0x17  //FIFO״̬�Ĵ���;bit0,RX FIFO�Ĵ����ձ�־;bit1,RX FIFO����־;bit2,3,����
                              //bit4,TX FIFO�ձ�־;bit5,TX FIFO����־;bit6,1,ѭ��������һ���ݰ�.0,��ѭ��;
															
															
															
	
// �Ĵ�������:

// CONFIG�Ĵ������ã�bit7:Ԥ�� bit6~bit0:
#define MASK_RX_DR			0x01<<6  //����RX_DR�����IRQ�����ж�
#define MASK_TX_DS      0x01<<5  //����TX_DS�����IRQ�����ж�
#define MASK_MAX_RT     0x01<<4  //����MAX_RT�����IRQ�����ж�
#define EN_CRC          0x01<<3  //ʹ��CRCУ�飬���EN_AA����λ�ߣ���ǿ������
#define CRCO_2Byte			0x01<<2  //CRC�������'0'-1�ֽ� '1'-2�ֽ�
#define PWR_UP          0x01<<1  //1: POWER UP, 0: POWER DOWN
#define PRIM_RX         0x01<<0  //RX/TX���� 1:PRX, 0:PTX


// EN_AA�Ĵ������ã� bit7~bit6��Ԥ�� bit5~bit0:
#define ENAA_P5         0x01<<5  //pipe5ͨ���Զ�Ӧ��ʹ��
#define ENAA_P4					0x01<<4  //pipe4ͨ���Զ�Ӧ��ʹ��
#define ENAA_P3					0x01<<3  //pipe3ͨ���Զ�Ӧ��ʹ��
#define ENAA_P2         0x01<<2  //pipe2ͨ���Զ�Ӧ��ʹ��
#define ENAA_P1					0x01<<1  //pipe1ͨ���Զ�Ӧ��ʹ��
#define ENAA_P0					0x01<<0  //pipe0ͨ���Զ�Ӧ��ʹ��


// EN_RXADDR�Ĵ������ã� bit7~bit6: Ԥ�� bit5~bit0:
#define ERX_P5					0x01<<5  //EN_RXADDR�Ĵ������ã�ʹ������pipe5
#define ERX_P4					0x01<<4  //EN_RXADDR�Ĵ������ã�ʹ������pipe4
#define ERX_P3					0x01<<3  //EN_RXADDR�Ĵ������ã�ʹ������pipe3
#define ERX_P2					0x01<<2  //EN_RXADDR�Ĵ������ã�ʹ������pipe2
#define ERX_P1          0x01<<1  //EN_RXADDR�Ĵ������ã�ʹ������pipe1
#define ERX_P0					0x01<<0  //EN_RXADDR�Ĵ������ã�ʹ������pipe0



// SETUP_AW�Ĵ������ã� bit7~bit2��Ԥ�� bit1~bit0:
#define AW_5BYTES				0x03<<0  //	RX/TX��ַ����: '11'-5bytes
#define AW_4BYTES       0x02<<0  // RX/TX��ַ����: '10'-4bytes								
#define AW_3BYTES				0x01<<0  //	RX/TX��ַ����: '01'-3bytes
#define AW_ILLEGAL      0x00<<0  // RX/TX��ַ����: '00'-illegal


// SETUP_RETR�Ĵ������ã�bit7~bit4:
#define ARD_WAIT_250US  			0x00<<4  // �Զ��ش�����ʱ: '0000'-250us
#define ARD_WAIT_500US  			0x01<<4  // �Զ��ش�����ʱ: '0001'-500us
#define ARD_WAIT_750US  			0x02<<4  // �Զ��ش�����ʱ: '0010'-750us
#define ARD_WAIT_1000US 			0x03<<4  // �Զ��ش�����ʱ: '0011'-1000us
#define ARD_WAIT_1250US 			0x04<<4  // �Զ��ش�����ʱ: '0011'-1250us
#define ARD_WAIT_1500US 			0x05<<4  // �Զ��ش�����ʱ: '0011'-1500us
#define ARD_WAIT_1750US 			0x06<<4  // �Զ��ش�����ʱ: '0011'-1750us
#define ARD_WAIT_2000US 			0x07<<4  // �Զ��ش�����ʱ: '0011'-2000us
#define ARD_WAIT_2250US 			0x08<<4  // �Զ��ش�����ʱ: '0011'-2250us
#define ARD_WAIT_2500US 			0x09<<4  // �Զ��ش�����ʱ: '0011'-2500us
#define ARD_WAIT_2750US 			0x0A<<4  // �Զ��ش�����ʱ: '0011'-2750us
#define ARD_WAIT_3000US 			0x0B<<4  // �Զ��ش�����ʱ: '0011'-3000us
#define ARD_WAIT_3250US 			0x0C<<4  // �Զ��ش�����ʱ: '0011'-3250us
#define ARD_WAIT_3500US 			0x0D<<4  // �Զ��ش�����ʱ: '0011'-3500us
#define ARD_WAIT_3750US 			0x0E<<4  // �Զ��ش�����ʱ: '0011'-3750us
#define ARD_WAIT_4000US 			0x0F<<4  // �Զ��ش�����ʱ: '0011'-4000us



// SETUP_RETR�Ĵ������ã�bit3~bit0:
#define ARC_RETRANSMIT_DIS    0x00<<0  // �Զ��ش���ʧ��
#define ARC_RETRANSMIT_1			0x01<<0  // �Զ��ش���1��
#define ARC_RETRANSMIT_2			0x02<<0  // �Զ��ش���2��
#define ARC_RETRANSMIT_3			0x03<<0  // �Զ��ش���3��
#define ARC_RETRANSMIT_4			0x04<<0  // �Զ��ش���4��
#define ARC_RETRANSMIT_5			0x05<<0  // �Զ��ش���5��
#define ARC_RETRANSMIT_6			0x06<<0  // �Զ��ش���6��
#define ARC_RETRANSMIT_7			0x07<<0  // �Զ��ش���7��
#define ARC_RETRANSMIT_8			0x08<<0  // �Զ��ش���8��
#define ARC_RETRANSMIT_9			0x09<<0  // �Զ��ش���9��
#define ARC_RETRANSMIT_10			0x0A<<0  // �Զ��ش���10��
#define ARC_RETRANSMIT_11		  0x0B<<0  // �Զ��ش���11��
#define ARC_RETRANSMIT_12			0x0C<<0  // �Զ��ش���12��
#define ARC_RETRANSMIT_13			0x0D<<0  // �Զ��ش���13��
#define ARC_RETRANSMIT_14			0x0E<<0  // �Զ��ش���14��
#define ARC_RETRANSMIT_15			0x0F<<0  // �Զ��ش���15��




// RF_SETUP�Ĵ������ã�bit6��Ԥ�� bit7��bit5~bit0:
#define CONT_WAVE							0x01<<7  // Enables continuous carrier transmit when high
#define RF_DR_LOW							0x01<<5  // ���������������ʵ�250kbps ����ο�RF_DR_HIGH
#define PLL_LOCK							0x01<<4  // ǿ��PLL���źţ�ֻ�ڲ���ʱʹ��
#define RF_DR_HIGH						0x01<<3  // ѡ��������� ���룺[RF_DR_LOW, RF_DR_HIGH]
																			 // '00'-1Mbps '01'-2Mbps '10'-250kbos '11'-Reserved
#define RF_PWR_18dBm  				0x00<<2  // RF������ʴ�С  '00'- -18dBm
#define RF_PWR_12dBm					0x01<<2	 // RF������ʴ�С  '01'- -12dBm 
#define RF_PWR_6dBm						0x02<<2	 // RF������ʴ�С  '10'- -6dBm 
#define RF_PWR_0dBm						0x03<<2	 // RF������ʴ�С  '11'- 0dBm


// STATUS�Ĵ������ã�bit7��Ԥ�� bit6~bit0:
#define RX_DR									0x01<<6  // RX FIFO��������׼���� �����жϣ���'1'�����־λ
#define TX_DS									0x01<<5  // TX FIFO��������׼���� �����жϣ�AUTO_ACKʹ����ACK���յ����ݸ�λ�ø�
																			 // ��'1'�����־λ
#define MAX_RT								0x01<<4  // ���TX���·��������жϣ���'1'�����־λ		
#define RX_P_NO_PIPE0					0x00<<1  // ��������ȡRX_FIFO�Ŀ������ݹܵ���� '000'~'101' - PIPE0~PIPE5
#define RX_P_NO_PIPE1					0x01<<1  
#define RX_P_NO_PIPE2					0x02<<1
#define RX_P_NO_PIPE3					0x03<<1
#define RX_P_NO_PIPE4					0x04<<1
#define RX_P_NO_PIPE5					0x05<<1
#define RX_P_NO_PIPE_NOTUSED	0x06<<1  // δʹ��
#define RX_P_NO_PIPE_FIFOEMP  0x07<<1  // RX FIFOΪ��

#define TX_FULL				        0x01<<0  // TX FIFO��������־λ '1'-�������� '0'-TX FIFO�п���λ��			



// check����ֵ��
#define SUCCESS								  0x01
#define FAILURE									0x00


//nRF24L01���ͽ������ݿ�ȶ���
#define TX_ADR_WIDTH    5   //5�ֽڵĵ�ַ���
#define RX_ADR_WIDTH    5   //5�ֽڵĵ�ַ���
#define TX_PLOAD_WIDTH  32  //32�ֽڵ��û����ݿ��
#define RX_PLOAD_WIDTH  32  //32�ֽڵ��û����ݿ��
#define TX_ADDRESS      0x34
#define RX_ADDRESS      0x34



//��������
void spi_nrf_init(void);
uint8_t spi_nrf_check(void);

uint8_t spi_nrf_reg_read(uint8_t reg );
uint8_t spi_nrf_reg_write(uint8_t reg, uint8_t value);

uint8_t spi_nrf_write_buffer(uint8_t reg, uint8_t* pBuf, uint8_t bytes);
uint8_t spi_nrf_read_buffer(uint8_t reg, uint8_t* pBuf, uint8_t bytes);

void spi_nrf_rx_mode(void);
void spi_nrf_tx_mode(void);

uint8_t spi_nrf_tx_packet(uint8_t *txbuffer);
uint8_t spi_nrf_rx_packet(uint8_t *rxbuf);

															
#endif
