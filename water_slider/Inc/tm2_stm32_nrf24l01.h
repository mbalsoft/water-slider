/*
 * tm2_stm32_nrf24l01.h
 *
 *  Created on: 25.05.2019
 *      Author: mbal
 *
 *      It's modification of tm_stm32_nrf24l01.h only
 *      Tilen Majerle
 *      email   tilen@majerle.eu
 *      website http://stm32f4-discovery.net
 *      link    http://stm32f4-discovery.net/2015/09/hal-library-25-nrf24l01-for-stm32fxxx/
 */

#ifndef TM2_STM32_NRF24L01_H_
#define TM2_STM32_NRF24L01_H_

#include "main.h"
#include "stm32f1xx_hal.h"

/* NRF24L01+ registers*/
#define NRF24L01_REG_CONFIG			0x00	//Configuration Register
#define NRF24L01_REG_EN_AA			0x01	//Enable ‘Auto Acknowledgment’ Function
#define NRF24L01_REG_EN_RXADDR		0x02	//Enabled RX Addresses
#define NRF24L01_REG_SETUP_AW		0x03	//Setup of Address Widths (common for all data pipes)
#define NRF24L01_REG_SETUP_RETR		0x04	//Setup of Automatic Retransmission
#define NRF24L01_REG_RF_CH			0x05	//RF Channel
#define NRF24L01_REG_RF_SETUP		0x06	//RF Setup Register
#define NRF24L01_REG_STATUS			0x07	//Status Register
#define NRF24L01_REG_OBSERVE_TX		0x08	//Transmit observe registerf
#define NRF24L01_REG_RPD			0x09
#define NRF24L01_REG_RX_ADDR_P0		0x0A	//Receive address data pipe 0. 5 Bytes maximum length.
#define NRF24L01_REG_RX_ADDR_P1		0x0B	//Receive address data pipe 1. 5 Bytes maximum length.
#define NRF24L01_REG_RX_ADDR_P2		0x0C	//Receive address data pipe 2. Only LSB
#define NRF24L01_REG_RX_ADDR_P3		0x0D	//Receive address data pipe 3. Only LSB
#define NRF24L01_REG_RX_ADDR_P4		0x0E	//Receive address data pipe 4. Only LSB
#define NRF24L01_REG_RX_ADDR_P5		0x0F	//Receive address data pipe 5. Only LSB
#define NRF24L01_REG_TX_ADDR		0x10	//Transmit address. Used for a PTX device only
#define NRF24L01_REG_RX_PW_P0		0x11
#define NRF24L01_REG_RX_PW_P1		0x12
#define NRF24L01_REG_RX_PW_P2		0x13
#define NRF24L01_REG_RX_PW_P3		0x14
#define NRF24L01_REG_RX_PW_P4		0x15
#define NRF24L01_REG_RX_PW_P5		0x16
#define NRF24L01_REG_FIFO_STATUS	0x17	//FIFO Status Register
#define NRF24L01_REG_DYNPD			0x1C	//Enable dynamic payload length
#define NRF24L01_REG_FEATURE		0x1D

/* Registers default values */
#define NRF24L01_REG_DEFAULT_VAL_CONFIG			0x08
#define NRF24L01_REG_DEFAULT_VAL_EN_AA			0x3F
#define NRF24L01_REG_DEFAULT_VAL_EN_RXADDR		0x03
#define NRF24L01_REG_DEFAULT_VAL_SETUP_AW		0x03
#define NRF24L01_REG_DEFAULT_VAL_SETUP_RETR		0x03
#define NRF24L01_REG_DEFAULT_VAL_RF_CH			0x02
#define NRF24L01_REG_DEFAULT_VAL_RF_SETUP		0x0E
#define NRF24L01_REG_DEFAULT_VAL_STATUS			0x0E
#define NRF24L01_REG_DEFAULT_VAL_OBSERVE_TX		0x00
#define NRF24L01_REG_DEFAULT_VAL_RPD			0x00
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_0	0xE7
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_1	0xE7
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_2	0xE7
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_3	0xE7
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_4	0xE7
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_0	0xC2
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_1	0xC2
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_2	0xC2
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_3	0xC2
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_4	0xC2
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P2		0xC3
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P3		0xC4
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P4		0xC5
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P5		0xC6
#define NRF24L01_REG_DEFAULT_VAL_TX_ADDR_0		0xE7
#define NRF24L01_REG_DEFAULT_VAL_TX_ADDR_1		0xE7
#define NRF24L01_REG_DEFAULT_VAL_TX_ADDR_2		0xE7
#define NRF24L01_REG_DEFAULT_VAL_TX_ADDR_3		0xE7
#define NRF24L01_REG_DEFAULT_VAL_TX_ADDR_4		0xE7
#define NRF24L01_REG_DEFAULT_VAL_RX_PW_P0		0x00
#define NRF24L01_REG_DEFAULT_VAL_RX_PW_P1		0x00
#define NRF24L01_REG_DEFAULT_VAL_RX_PW_P2		0x00
#define NRF24L01_REG_DEFAULT_VAL_RX_PW_P3		0x00
#define NRF24L01_REG_DEFAULT_VAL_RX_PW_P4		0x00
#define NRF24L01_REG_DEFAULT_VAL_RX_PW_P5		0x00
#define NRF24L01_REG_DEFAULT_VAL_FIFO_STATUS	0x11
#define NRF24L01_REG_DEFAULT_VAL_DYNPD			0x00
#define NRF24L01_REG_DEFAULT_VAL_FEATURE		0x00

/* Configuration register*/
#define NRF24L01_MASK_RX_DR		6
#define NRF24L01_MASK_TX_DS		5
#define NRF24L01_MASK_MAX_RT	4
#define NRF24L01_EN_CRC			3
#define NRF24L01_CRCO			2
#define NRF24L01_PWR_UP			1
#define NRF24L01_PRIM_RX		0

/* Enable auto acknowledgment*/
#define NRF24L01_ENAA_P5		5
#define NRF24L01_ENAA_P4		4
#define NRF24L01_ENAA_P3		3
#define NRF24L01_ENAA_P2		2
#define NRF24L01_ENAA_P1		1
#define NRF24L01_ENAA_P0		0

/* Enable rx addresses */
#define NRF24L01_ERX_P5			5
#define NRF24L01_ERX_P4			4
#define NRF24L01_ERX_P3			3
#define NRF24L01_ERX_P2			2
#define NRF24L01_ERX_P1			1
#define NRF24L01_ERX_P0			0

/* Setup of address width */
#define NRF24L01_AW				0 //2 bits

/* Setup of auto re-transmission*/
#define NRF24L01_ARD			4 //4 bits
#define NRF24L01_ARC			0 //4 bits

/* RF setup register*/
#define NRF24L01_PLL_LOCK		4
#define NRF24L01_RF_DR_LOW		5
#define NRF24L01_RF_DR_HIGH		3
#define NRF24L01_RF_DR			3
#define NRF24L01_RF_PWR			1 //2 bits

/* General status register */
#define NRF24L01_RX_DR			6
#define NRF24L01_TX_DS			5
#define NRF24L01_MAX_RT			4
#define NRF24L01_RX_P_NO		1 //3 bits
#define NRF24L01_TX_FULL		0

/* Transmit observe register */
#define NRF24L01_PLOS_CNT		4 //4 bits
#define NRF24L01_ARC_CNT		0 //4 bits

/* FIFO status*/
#define NRF24L01_TX_REUSE		6
#define NRF24L01_FIFO_FULL		5
#define NRF24L01_TX_EMPTY		4
#define NRF24L01_RX_FULL		1
#define NRF24L01_RX_EMPTY		0

//Dynamic length
#define NRF24L01_DPL_P0			0
#define NRF24L01_DPL_P1			1
#define NRF24L01_DPL_P2			2
#define NRF24L01_DPL_P3			3
#define NRF24L01_DPL_P4			4
#define NRF24L01_DPL_P5			5

/* Transmitter power*/
#define NRF24L01_M18DBM			0 //-18 dBm
#define NRF24L01_M12DBM			1 //-12 dBm
#define NRF24L01_M6DBM			2 //-6 dBm
#define NRF24L01_0DBM			3 //0 dBm

/* Data rates */
#define NRF24L01_2MBPS			0
#define NRF24L01_1MBPS			1
#define NRF24L01_250KBPS		2

/* Configuration */
#define NRF24L01_CONFIG			((1 << NRF24L01_EN_CRC) | (0 << NRF24L01_CRCO) | (1 << NRF24L01_MASK_RX_DR) | (1 << NRF24L01_MASK_TX_DS) | (1 << NRF24L01_MASK_MAX_RT))

/* Instruction Mnemonics */
#define NRF24L01_REGISTER_MASK				0x1F

#define NRF24L01_READ_REGISTER_MASK(reg)	(0x00 | (NRF24L01_REGISTER_MASK & reg)) //Last 5 bits will indicate reg. address
#define NRF24L01_WRITE_REGISTER_MASK(reg)	(0x20 | (NRF24L01_REGISTER_MASK & reg)) //Last 5 bits will indicate reg. address
#define NRF24L01_R_RX_PAYLOAD_MASK			0x61
#define NRF24L01_W_TX_PAYLOAD_MASK			0xA0
#define NRF24L01_FLUSH_TX_MASK				0xE1
#define NRF24L01_FLUSH_RX_MASK				0xE2
#define NRF24L01_REUSE_TX_PL_MASK			0xE3
#define NRF24L01_ACTIVATE_MASK				0x50
#define NRF24L01_R_RX_PL_WID_MASK			0x60
#define NRF24L01_NOP_MASK					0xFF

#define NRF24L01_TRANSMISSON_OK 			0
#define NRF24L01_MESSAGE_LOST   			1

#define NRF24L01_CHECK_BIT(reg, bit)       (reg & (1 << bit))

/**
 * @defgroup TM_NRF24L01P_Typedefs
 * @brief    Library Typedefs
 * @{
 */

/**
 * @brief  Interrupt structure
 */
typedef union _TM_NRF24L01_IRQ_t {
	struct {
		uint8_t reserved0:4;
		uint8_t MaxRT:1;     /*!< Set to 1 if MAX retransmissions flag is set */
		uint8_t DataSent:1;  /*!< Set to 1 if last transmission is OK */
		uint8_t DataReady:1; /*!< Set to 1 if data are ready to be read */
		uint8_t reserved1:1;
	} F;
	uint8_t Status;          /*!< NRF status register value */
} TM_NRF24L01_IRQ_t;

/**
 * @brief  Transmission status enumeration
 */
typedef enum _TM_NRF24L01_Transmit_Status_t {
	TM_NRF24L01_Transmit_Status_Lost = 0x00,   /*!< Message is lost, reached maximum number of retransmissions */
	TM_NRF24L01_Transmit_Status_Ok = 0x01,     /*!< Message sent successfully */
	TM_NRF24L01_Transmit_Status_Sending = 0xFF /*!< Message is still sending */
} TM_NRF24L01_Transmit_Status_t;

/**
 * @brief  Data rate enumeration
 */
typedef enum _TM_NRF24L01_DataRate_t {
	TM_NRF24L01_DataRate_2M = 0x00, /*!< Data rate set to 2Mbps */
	TM_NRF24L01_DataRate_1M,        /*!< Data rate set to 1Mbps */
	TM_NRF24L01_DataRate_250k       /*!< Data rate set to 250kbps */
} TM_NRF24L01_DataRate_t;

/**
 * @brief  Output power enumeration
 */
typedef enum _TM_NRF24L01_OutputPower_t {
	TM_NRF24L01_OutputPower_M18dBm = 0x00,/*!< Output power set to -18dBm */
	TM_NRF24L01_OutputPower_M12dBm,       /*!< Output power set to -12dBm */
	TM_NRF24L01_OutputPower_M6dBm,        /*!< Output power set to -6dBm */
	TM_NRF24L01_OutputPower_0dBm          /*!< Output power set to 0dBm */
} TM_NRF24L01_OutputPower_t;

/**
 * @brief  NRF24L01 configuration record
 */
typedef struct {
	uint16_t                  CE_pin;
	GPIO_TypeDef*             CE_port;
	uint16_t                  CSN_pin;
	GPIO_TypeDef*             CSN_port;
	SPI_HandleTypeDef*        SPI;

	uint8_t                   rx_address[ 5 ];
	uint8_t                   tx_address[ 5 ];
	uint8_t                   radio_channel;
	TM_NRF24L01_DataRate_t    baud_rate;
	uint8_t                   payload_len;
	uint8_t                   crc_len;
	TM_NRF24L01_OutputPower_t output_power;
} NRF24L01_config_TypeDef;

/**
 * @brief  Public function definition
 */
uint8_t tm2_NRF24L01_Init( NRF24L01_config_TypeDef* );
void tm2_NRF24L01_Transmit( NRF24L01_config_TypeDef*, uint8_t* );
void tm2_NRF24L01_GetData( NRF24L01_config_TypeDef*, uint8_t* );
uint8_t tm2_NRF24L01_DataReady( NRF24L01_config_TypeDef* );
uint8_t tm2_NRF24L01_RxFifoEmpty( NRF24L01_config_TypeDef* );
uint8_t tm2_NRF24L01_GetStatus( NRF24L01_config_TypeDef* );
TM_NRF24L01_Transmit_Status_t tm2_NRF24L01_GetTransmissionStatus( NRF24L01_config_TypeDef* );
void tm2_NRF24L01_SoftwareReset( NRF24L01_config_TypeDef* );
uint8_t tm2_NRF24L01_GetRetransmissionsCount( NRF24L01_config_TypeDef* );
void tm2_NRF24L01_SetChannel( NRF24L01_config_TypeDef* );
uint8_t tm2_NRF24L01_SetRF( NRF24L01_config_TypeDef* );
void tm2_NRF24L01_Clear_Interrupts( NRF24L01_config_TypeDef* );
void tm2_NRF24L01_PowerUpTx( NRF24L01_config_TypeDef* );
void tm2_NRF24L01_PowerUpRx( NRF24L01_config_TypeDef* );
void tm2_NRF24L01_PowerDown( NRF24L01_config_TypeDef* );

void tm2_NRF24L01_ReadConfig( NRF24L01_config_TypeDef*, uint8_t* );


#endif /* TM2_STM32_NRF24L01_H_ */
