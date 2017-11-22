/**
 * @file nRF24L01.c
 *
 * nRF24L01 �����շ�ģ���������
 * 
 */

#include "nRF24L01.h"

#include "api_spi.h"
#include "board_config.h"

#include "stm32f10x.h"






/**
  * ���ƣ�spi_nrf_init
  *  
  * ������nRF24L01�豸��ʼ��
  *
  */
void spi_nrf_init(void)
{
	gpio_clk_config(); //������
	
	rc_spi_gpio_config();
  rc_spi_config();
	
	// �������಻����
	RC_SPI_CE_LOW_FUN();
	RC_SPI_NSS_HIGH_FUN();
}









/**
  * ���ƣ�spi_nrf_check
  *
  * ���������nRF24L01�豸�Ƿ����� 
  *
  */
uint8_t spi_nrf_check(void)
{
   uint8_t index;
   uint8_t check_data[5] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
	 //У������
	 
	 spi_nrf_write_buffer(WRITE_REG_NRF + TX_ADDR, check_data, 5);
	 //д�뷢�ͼĴ���
	 spi_nrf_read_buffer(READ_REG_NRF + TX_ADDR, check_data, 5);
	 //������д��TX_ADDR�Ĵ����е�����
	 
   for(index = 1; index < 5; index++)
	 {
	    if (check_data[index] != 0xAA)
		  break;
	 }
	 
	 if(index == 5)
	 {
	    return SUCCESS;
	 }
	 return FAILURE;

}









/**
  * ���ƣ�spi_nrf_reg_write
  *  
  * ����������nRF24L01�豸�Ĵ���
  *
  */
  
uint8_t spi_nrf_reg_write(uint8_t reg, uint8_t value)
{
	
	uint8_t status;
	// ��ʼSPI����
	RC_SPI_NSS_LOW_FUN() ;
	
	// ���ؼĴ���״ֵ̬
	status = spi_send_byte(RC_SPI, reg);
	spi_send_byte(RC_SPI, value);
	
	RC_SPI_NSS_HIGH_FUN() ;
	return status;
	
}








/**
  * ���ƣ�spi_nrf_reg_read
  *  
  * ���������������ȡ�Ĵ�����ֵ
  *
  */

uint8_t spi_nrf_reg_read(uint8_t reg )
{
	uint8_t reg_val;
	
	RC_SPI_NSS_LOW_FUN();
	
	// ���üĴ������Եõ�״̬�������޷����أ��ɵ���ʹ�����
	spi_send_byte(RC_SPI, reg);
	reg_val = spi_send_byte(RC_SPI, 0);

  RC_SPI_NSS_HIGH_FUN();
	
	return reg_val;

}








/**
  * ���ƣ�spi_nrf_read_buffer
  *  
  * ��������ȡ�����ֽ�����
  *
  */


uint8_t spi_nrf_read_buffer(uint8_t reg, uint8_t* pBuf, uint8_t bytes)
{
	uint8_t status, byte_ctrl; 
	
	RC_SPI_NSS_LOW_FUN();
	
	status = spi_send_byte(RC_SPI, reg);
	
	for (byte_ctrl = 1; byte_ctrl < bytes; byte_ctrl++)
	{
			pBuf[byte_ctrl] = spi_send_byte(RC_SPI, 0);
	}
	
	RC_SPI_NSS_HIGH_FUN();

  return status;
		
}








/**
  * ���ƣ�spi_nrf_write_buffer
  *  
  * ������д�������ֽ�����
  *
  */


uint8_t spi_nrf_write_buffer(uint8_t reg, uint8_t* pBuf, uint8_t bytes)
{
	uint8_t byte_ctrl, status; 
	
	RC_SPI_NSS_LOW_FUN();
	
	status = spi_send_byte(RC_SPI, reg);
	
	for (byte_ctrl = 1; byte_ctrl<bytes; byte_ctrl++)
	{
			spi_send_byte(RC_SPI, pBuf[byte_ctrl]);
	}
	
	RC_SPI_NSS_HIGH_FUN();

  return status;
		
}









/**
  * ���ƣ�spi_nrf_rx_mode
  *  
  * ��������ʼ��NRF24L01ΪRXģʽ
  *
  */

void spi_nrf_rx_mode(void)
{
	  RC_SPI_CE_LOW_FUN();
	  //CEΪ��,��������ģʽ 
  
  	spi_nrf_write_buffer(WRITE_REG_NRF+RX_ADDR_P0, (uint8_t *)RX_ADDRESS, RX_ADR_WIDTH);
	  //дRX�ڵ��ַ
	
	  spi_nrf_reg_write(WRITE_REG_NRF+EN_AA, ENAA_P0);    
	  //ʹ��ͨ��0���Զ�Ӧ��
	 	spi_nrf_reg_write(WRITE_REG_NRF+EN_RXADDR, ERX_P0);
	  //ʹ��ͨ��0�Ľ��յ�ַ  	 
  	spi_nrf_reg_write(WRITE_REG_NRF+RF_CH,40);	     
	  //����RFͨ��Ƶ��40Hz
  	spi_nrf_reg_write(WRITE_REG_NRF+RX_PW_P0,RX_PLOAD_WIDTH);
	  //ѡ��ͨ��0����Ч���ݿ�� 	    
  	spi_nrf_reg_write(WRITE_REG_NRF+RF_SETUP, RF_DR_HIGH | RF_PWR_0dBm);
	  //���÷������,0db����,2Mbps   
  	spi_nrf_reg_write(WRITE_REG_NRF+CONFIG, EN_CRC | CRCO_2Byte | PWR_UP | PRIM_RX);
	  //���û�������ģʽ�Ĳ���; PWR_UP, ʹ��CRCУ��, 16λCRC����, ����ģʽ������IRQȫ���ж�
	
  	RC_SPI_CE_HIGH_FUN(); 
	  //CEΪ��,���빤��ģʽ(����) 
}		



/**
  * ���ƣ�spi_nrf_tx_mode
  *  
  * ��������ʼ��NRF24L01ΪTXģʽ
  *
  */

void spi_nrf_tx_mode(void)
{														 
	  RC_SPI_CE_LOW_FUN();    
	
  	spi_nrf_write_buffer(WRITE_REG_NRF+TX_ADDR, (uint8_t *)TX_ADDRESS, TX_ADR_WIDTH);
	  //дTX�ڵ��ַ 
  	spi_nrf_write_buffer(WRITE_REG_NRF+RX_ADDR_P0, (uint8_t*)RX_ADDRESS, RX_ADR_WIDTH); 
	  //����pipe0�ڵ��ַ, ��������ACK�ź�	  

  	spi_nrf_reg_write(WRITE_REG_NRF+EN_AA, ENAA_P0);     
	  //ʹ��ͨ��0���Զ�Ӧ��    
  	spi_nrf_reg_write(WRITE_REG_NRF+EN_RXADDR,ERX_P0); 
	  //ʹ��ͨ��0�Ľ��յ�ַ  
  	spi_nrf_reg_write(WRITE_REG_NRF+SETUP_RETR,ARD_WAIT_500US | ARC_RETRANSMIT_10);
	  //�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
  	spi_nrf_reg_write(WRITE_REG_NRF+RF_CH,40);       
	  //����RFͨ��Ƶ��Ϊ40Hz
  	spi_nrf_reg_write(WRITE_REG_NRF+RF_SETUP, RF_DR_HIGH | RF_PWR_0dBm);  
	  //���÷������,0db����,2Mbps 
  	spi_nrf_reg_write(WRITE_REG_NRF+CONFIG, EN_CRC | CRCO_2Byte | PWR_UP );    
	  //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	
	  RC_SPI_CE_HIGH_FUN(); 
		//CEΪ��,10us����������
}		  





/**
  * ���ƣ�spi_nrf_tx_packet
  *  
  * ������ͨ��nRF24L01��������
  *
  */

uint8_t spi_nrf_tx_packet(uint8_t *txbuffer)
{
	uint8_t tx_status;
	
	RC_SPI_CE_LOW_FUN();
	//д����������
	spi_nrf_write_buffer(WRITE_REG_NRF + WR_TX_PAYLOAD, txbuffer, TX_PLOAD_WIDTH);
	//дtxbuffer�е����ݵ�WR_TX_PAYLOAD�Ĵ���, txbuffer��һ�����ݵ��׵�ַ���Զ���ȡ��Ӧ32Byte����
	RC_SPI_CE_HIGH_FUN();
	//��������	 
  
	while(RC_SPI_INT_SCAN_FUN()!=RC_SPI_INT_LOW)
	{
		;
	}//�ȴ��������
	
	tx_status = spi_nrf_reg_read(READ_REG_NRF + STATUS);  
	//��ȡ״̬�Ĵ�����ֵ	
	spi_nrf_reg_write(WRITE_REG_NRF + STATUS, tx_status); 
	//���TX_DS��MAX_RT�жϱ�־
	
	if(tx_status & MAX_RT)//�ﵽ����ط�����
	{
		spi_nrf_reg_write(FLUSH_TX, 0xff);
		//���TX FIFO�Ĵ��� 
		
		return MAX_RT; 
	}
	
	if(tx_status & TX_DS)//�������
	{
		return SUCCESS;
	}
	return FAILURE;//����ʧ��
}







/**
  * ���ƣ�spi_nrf_rx_packet
  *  
  * ������ͨ��nRF24L01��������
  *
  */

uint8_t spi_nrf_rx_packet(uint8_t *rxbuf)
{
	uint8_t rx_status;		    
	
	rx_status = spi_nrf_reg_read(STATUS);  
	//��ȡ״̬�Ĵ�����ֵ    	 
	spi_nrf_reg_write(WRITE_REG_NRF+STATUS, rx_status); 
	//���RX_DR�жϱ�־
	
	
	if( rx_status & RX_DR )//���յ�����
	{ 
		spi_nrf_write_buffer(READ_REG_NRF + RD_RX_PAYLOAD, rxbuf, RX_PLOAD_WIDTH);
		//��ȡ����
		spi_nrf_reg_write(FLUSH_RX, 0xff);
		//���RX FIFO�Ĵ��� 
		return SUCCESS; 
	}	   
	return FAILURE;//û�յ��κ�����
}		

