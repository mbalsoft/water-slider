/*
 * tm2_stm32_nrf24l01.c
 *
 *  Created on: 25.05.2019
 *      Author: mbal
 *      It's modification of tm_stm32_nrf24l01.c only
 *      Tilen Majerle
 *      email   tilen@majerle.eu
 *      website http://stm32f4-discovery.net
 *      link    http://stm32f4-discovery.net/2015/09/hal-library-25-nrf24l01-for-stm32fxxx/
 */

#include "main.h"
#include "stm32f1xx_hal.h"
#include "tm2_stm32_nrf24l01.h"

// private function definitions
void tm2_NRF24L01_CE_LOW( NRF24L01_config_TypeDef* );
void tm2_NRF24L01_CE_HIGH( NRF24L01_config_TypeDef* );
void tm2_NRF24L01_CSN_LOW( NRF24L01_config_TypeDef* );
void tm2_NRF24L01_CSN_HIGH( NRF24L01_config_TypeDef* );
void tm2_NRF24L01_FLUSH_TX( NRF24L01_config_TypeDef* );
void tm2_NRF24L01_FLUSH_RX( NRF24L01_config_TypeDef* );
void tm2_NRF24L01_WriteBit( NRF24L01_config_TypeDef*, uint8_t, uint8_t, uint8_t );
uint8_t tm2_NRF24L01_ReadBit( NRF24L01_config_TypeDef*, uint8_t, uint8_t );
uint8_t tm2_NRF24L01_ReadRegister( NRF24L01_config_TypeDef*, uint8_t );
void tm2_NRF24L01_ReadRegisterMulti( NRF24L01_config_TypeDef*, uint8_t, uint8_t*, uint8_t );
void tm2_NRF24L01_WriteRegister( NRF24L01_config_TypeDef*, uint8_t, uint8_t );
void tm2_NRF24L01_WriteRegisterMulti( NRF24L01_config_TypeDef*, uint8_t, uint8_t*, uint8_t );

//================================================================
//============  functions bodys                    ===============
//================================================================

uint8_t tm2_NRF24L01_Init( NRF24L01_config_TypeDef *nrf_config ) {
	uint8_t control_value;
	uint8_t tmp_int8;

	/* CSN high = disable SPI */
	tm2_NRF24L01_CSN_HIGH( nrf_config );

	/* CE low = disable TX/RX */
	tm2_NRF24L01_CE_LOW( nrf_config );

	/* Max payload is 32bytes */
	if( nrf_config->payload_len > 32 ) {
		nrf_config->payload_len = 32;
	}

	/* Reset nRF24L01+ to power on registers values */
	tm2_NRF24L01_SoftwareReset( nrf_config );

	/* Channel select */
	tm2_NRF24L01_SetChannel( nrf_config );
	//tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_RF_CH, nrf_config->radio_channel );
	control_value = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_RF_CH );
	if( control_value != nrf_config->radio_channel ) return 2;

	/* Set pipeline to max possible 32 bytes */
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_RX_PW_P0, nrf_config->payload_len ); // Auto-ACK pipe
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_RX_PW_P1, nrf_config->payload_len ); // Data payload pipe
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_RX_PW_P2, nrf_config->payload_len );
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_RX_PW_P3, nrf_config->payload_len );
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_RX_PW_P4, nrf_config->payload_len );
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_RX_PW_P5, nrf_config->payload_len );

	control_value = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_RX_PW_P0 );
	if( control_value != nrf_config->payload_len ) return 10;
	control_value = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_RX_PW_P1 );
	if( control_value != nrf_config->payload_len ) return 11;
	control_value = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_RX_PW_P2 );
	if( control_value != nrf_config->payload_len ) return 12;
	control_value = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_RX_PW_P3 );
	if( control_value != nrf_config->payload_len ) return 13;
	control_value = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_RX_PW_P4 );
	if( control_value != nrf_config->payload_len ) return 14;
	control_value = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_RX_PW_P5 );
	if( control_value != nrf_config->payload_len ) return 15;

	/* Set RF settings (2mbps, output power) */
	if( tm2_NRF24L01_SetRF( nrf_config ) != 0 ) return 3;

	/* Config register */
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_CONFIG, NRF24L01_CONFIG );
	control_value = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_CONFIG );
	if( control_value != NRF24L01_CONFIG ) return 4;

	/* Enable auto-acknowledgment for all pipes */
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_EN_AA, 0x00 ); //0x3F
	control_value = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_EN_AA );
	if( control_value != 0x00 ) return 5;

	/* Enable RX addresses */
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_EN_RXADDR, 0x3F );
	control_value = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_EN_RXADDR );
	if( control_value != 0x3f ) return 6;

	/* Auto retransmit delay: 1000 (4x250) us and Up to 15 retransmit trials */
	//tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_SETUP_RETR, 0x4F );
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_SETUP_RETR, 0x40 ); //retransmit disabled
	control_value = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_SETUP_RETR );
	if( control_value != 0x40 ) return 7;

	/* Dynamic length configurations: No dynamic length */
	tmp_int8 = (0 << NRF24L01_DPL_P0) | (0 << NRF24L01_DPL_P1) | (0 << NRF24L01_DPL_P2) | (0 << NRF24L01_DPL_P3) | (0 << NRF24L01_DPL_P4) | (0 << NRF24L01_DPL_P5);
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_DYNPD, tmp_int8 );
	control_value = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_DYNPD );
	if( control_value != tmp_int8 ) return 8;

	/* Clear FIFOs */
	tm2_NRF24L01_FLUSH_TX( nrf_config );
	tm2_NRF24L01_FLUSH_RX( nrf_config );

	/* Clear interrupts */
	tm2_NRF24L01_Clear_Interrupts( nrf_config );

	/* Go to RX mode */
	tm2_NRF24L01_PowerUpRx( nrf_config );

	/* Return OK */
	return 0;
}

/* Pins configuration */
//#define NRF24L01_CE_LOW				TM_GPIO_SetPinLow(NRF24L01_CE_PORT, NRF24L01_CE_PIN)
//#define NRF24L01_CE_HIGH			TM_GPIO_SetPinHigh(NRF24L01_CE_PORT, NRF24L01_CE_PIN)
//#define NRF24L01_CSN_LOW			TM_GPIO_SetPinLow(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN)
//#define NRF24L01_CSN_HIGH			TM_GPIO_SetPinHigh(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN)

void tm2_NRF24L01_CE_LOW( NRF24L01_config_TypeDef *nrf_config ) {
	HAL_GPIO_WritePin( nrf_config->CE_port, nrf_config->CE_pin, GPIO_PIN_RESET );
}

void tm2_NRF24L01_CE_HIGH( NRF24L01_config_TypeDef *nrf_config ) {
	HAL_GPIO_WritePin( nrf_config->CE_port, nrf_config->CE_pin, GPIO_PIN_SET );
}

void tm2_NRF24L01_CSN_LOW( NRF24L01_config_TypeDef *nrf_config ) {
	HAL_GPIO_WritePin( nrf_config->CSN_port, nrf_config->CSN_pin, GPIO_PIN_RESET );
}

void tm2_NRF24L01_CSN_HIGH( NRF24L01_config_TypeDef *nrf_config ) {
	HAL_GPIO_WritePin( nrf_config->CSN_port, nrf_config->CSN_pin, GPIO_PIN_SET );
}

/* Flush FIFOs */
//#define NRF24L01_FLUSH_TX					do { NRF24L01_CSN_LOW; TM_SPI_Send(NRF24L01_SPI, NRF24L01_FLUSH_TX_MASK); NRF24L01_CSN_HIGH; } while (0)
//#define NRF24L01_FLUSH_RX					do { NRF24L01_CSN_LOW; TM_SPI_Send(NRF24L01_SPI, NRF24L01_FLUSH_RX_MASK); NRF24L01_CSN_HIGH; } while (0)

void tm2_NRF24L01_FLUSH_TX( NRF24L01_config_TypeDef *nrf_config ) {
	do {
		tm2_NRF24L01_CSN_LOW( nrf_config );
		unsigned char cData = NRF24L01_FLUSH_TX_MASK;
		HAL_SPI_Transmit( nrf_config->SPI, &cData, 1, 100 );
		tm2_NRF24L01_CSN_HIGH( nrf_config );
	} while( 0 );
}

void tm2_NRF24L01_FLUSH_RX( NRF24L01_config_TypeDef *nrf_config ) {
	do {
		tm2_NRF24L01_CSN_LOW( nrf_config );
		unsigned char cData = NRF24L01_FLUSH_RX_MASK;
		HAL_SPI_Transmit( nrf_config->SPI, &cData, 1, 100 );
		tm2_NRF24L01_CSN_HIGH( nrf_config );
	} while( 0 );
}

void tm2_NRF24L01_WriteBit( NRF24L01_config_TypeDef *nrf_config, uint8_t reg, uint8_t bit, uint8_t value ) {
	uint8_t tmp;

	/* Read register */
	tmp = tm2_NRF24L01_ReadRegister( nrf_config, reg );
	/* Make operation */
	if (value) {
		tmp |= 1 << bit;
	} else {
		tmp &= ~(1 << bit);
	}
	/* Write back */
	tm2_NRF24L01_WriteRegister( nrf_config, reg, tmp );
}

uint8_t tm2_NRF24L01_ReadBit( NRF24L01_config_TypeDef *nrf_config, uint8_t reg, uint8_t bit ) {
	uint8_t tmp;

	tmp = tm2_NRF24L01_ReadRegister(nrf_config, reg );
	if( ! NRF24L01_CHECK_BIT( tmp, bit )) {
		return 0;
	}
	return 1;
}

uint8_t tm2_NRF24L01_ReadRegister( NRF24L01_config_TypeDef *nrf_config, uint8_t reg ) {
	uint8_t value;
	unsigned char cData;

	tm2_NRF24L01_CSN_LOW( nrf_config );
	//TM_SPI_Send(NRF24L01_SPI, NRF24L01_READ_REGISTER_MASK(reg));
	cData = NRF24L01_READ_REGISTER_MASK( reg );
	HAL_SPI_Transmit( nrf_config->SPI, &cData, 1, 100 );
	//value = TM_SPI_Send(NRF24L01_SPI, NRF24L01_NOP_MASK);
	cData = NRF24L01_NOP_MASK;
	HAL_SPI_TransmitReceive( nrf_config->SPI, &cData, &value, 1, 100 );
	tm2_NRF24L01_CSN_HIGH( nrf_config );

	return value;
}

void tm2_NRF24L01_ReadRegisterMulti( NRF24L01_config_TypeDef *nrf_config, uint8_t reg, uint8_t* rxData, uint8_t count ) {
	unsigned char cData;
	unsigned char txData[ count ];

	for( int loop = 0 ; loop < count; loop ++ ) {
		txData[ loop ] = NRF24L01_NOP_MASK;
	}
	tm2_NRF24L01_CSN_LOW( nrf_config );
	//TM_SPI_Send(NRF24L01_SPI, NRF24L01_READ_REGISTER_MASK( reg ));
	cData = NRF24L01_READ_REGISTER_MASK( reg );
	HAL_SPI_Transmit( nrf_config->SPI, &cData, 1, 100 );
	//TM_SPI_ReadMulti( nrf_config, data, NRF24L01_NOP_MASK, count );
	HAL_SPI_TransmitReceive( nrf_config->SPI, txData, rxData, count, 100 );
	tm2_NRF24L01_CSN_HIGH( nrf_config );
}

void tm2_NRF24L01_WriteRegister( NRF24L01_config_TypeDef *nrf_config, uint8_t reg, uint8_t value ) {
	unsigned char cData;

	tm2_NRF24L01_CSN_LOW( nrf_config );
	//TM_SPI_Send(NRF24L01_SPI, NRF24L01_WRITE_REGISTER_MASK(reg));
	cData = NRF24L01_WRITE_REGISTER_MASK( reg );
	HAL_SPI_Transmit( nrf_config->SPI, &cData, 1, 100 );
	//TM_SPI_Send(NRF24L01_SPI, value);
	HAL_SPI_Transmit( nrf_config->SPI, &value, 1, 100 );
	tm2_NRF24L01_CSN_HIGH( nrf_config );
}

void tm2_NRF24L01_WriteRegisterMulti( NRF24L01_config_TypeDef *nrf_config, uint8_t reg, uint8_t *data, uint8_t count ) {
	unsigned char cData;

	tm2_NRF24L01_CSN_LOW( nrf_config );
	//TM_SPI_Send(NRF24L01_SPI, NRF24L01_WRITE_REGISTER_MASK( reg ));
	cData = NRF24L01_WRITE_REGISTER_MASK( reg );
	HAL_SPI_Transmit( nrf_config->SPI, &cData, 1, 100 );
	//TM_SPI_WriteMulti(NRF24L01_SPI, data, count);
	HAL_SPI_Transmit( nrf_config->SPI, data, count, 100 );
	tm2_NRF24L01_CSN_HIGH( nrf_config );
}

void tm2_NRF24L01_PowerUpTx( NRF24L01_config_TypeDef *nrf_config ) {
	tm2_NRF24L01_Clear_Interrupts( nrf_config );
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_CONFIG, NRF24L01_CONFIG | (0 << NRF24L01_PRIM_RX) | (1 << NRF24L01_PWR_UP));
}

void tm2_NRF24L01_PowerUpRx( NRF24L01_config_TypeDef *nrf_config ) {
	/* Disable RX/TX mode */
	tm2_NRF24L01_CE_LOW( nrf_config );
	/* Clear RX buffer */
	tm2_NRF24L01_FLUSH_RX( nrf_config );
	/* Clear interrupts */
	tm2_NRF24L01_Clear_Interrupts( nrf_config );
	/* Setup RX mode */
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_CONFIG, NRF24L01_CONFIG | 1 << NRF24L01_PWR_UP | 1 << NRF24L01_PRIM_RX);
	/* Start listening */
	tm2_NRF24L01_CE_HIGH( nrf_config );
}

void tm2_NRF24L01_PowerDown( NRF24L01_config_TypeDef *nrf_config ) {
	tm2_NRF24L01_CE_LOW( nrf_config );
	tm2_NRF24L01_WriteBit( nrf_config, NRF24L01_REG_CONFIG, NRF24L01_PWR_UP, 0 );
}

void tm2_NRF24L01_Transmit( NRF24L01_config_TypeDef *nrf_config, uint8_t *data ) {
	uint8_t count = nrf_config->payload_len;
	unsigned char cData;

	/* Chip enable put to low, disable it */
	tm2_NRF24L01_CE_LOW( nrf_config );

	/* Go to power up tx mode */
	tm2_NRF24L01_PowerUpTx( nrf_config );

	/* Clear TX FIFO from NRF24L01+ */
	tm2_NRF24L01_FLUSH_TX( nrf_config );

	/* Send payload to nRF24L01+ */
	tm2_NRF24L01_CSN_LOW( nrf_config );
	/* Send write payload command */
	//TM_SPI_Send(NRF24L01_SPI, NRF24L01_W_TX_PAYLOAD_MASK);
	cData = NRF24L01_W_TX_PAYLOAD_MASK;
	HAL_SPI_Transmit( nrf_config->SPI, &cData, 1, 100 );
	/* Fill payload with data*/
	//TM_SPI_WriteMulti(NRF24L01_SPI, data, count);
	HAL_SPI_Transmit( nrf_config->SPI, data, count, 100 );
	/* Disable SPI */
	tm2_NRF24L01_CSN_HIGH( nrf_config );

	/* Send data! */
	tm2_NRF24L01_CE_HIGH( nrf_config );
}

void tm2_NRF24L01_GetData( NRF24L01_config_TypeDef *nrf_config, uint8_t* data ) {
	unsigned char cData;

	/* Pull down chip select */
	tm2_NRF24L01_CSN_LOW( nrf_config );
	/* Send read payload command*/
	//TM_SPI_Send(NRF24L01_SPI, NRF24L01_R_RX_PAYLOAD_MASK);
	cData = NRF24L01_R_RX_PAYLOAD_MASK;
	HAL_SPI_Transmit( nrf_config->SPI, &cData, 1, 100 );
	/* Read payload */
	//TM_SPI_SendMulti(NRF24L01_SPI, data, data, TM_NRF24L01_Struct.PayloadSize);
	HAL_SPI_TransmitReceive( nrf_config->SPI, data, data, nrf_config->payload_len, 100 );
	/* Pull up chip select */
	tm2_NRF24L01_CE_HIGH( nrf_config );

	/* Reset status register, clear RX_DR interrupt flag */
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_STATUS, (1 << NRF24L01_RX_DR));
}

uint8_t tm2_NRF24L01_DataReady( NRF24L01_config_TypeDef *nrf_config ) {
	uint8_t status = tm2_NRF24L01_GetStatus( nrf_config );

	if (NRF24L01_CHECK_BIT(status, NRF24L01_RX_DR)) {
		return 1;
	}
	return !tm2_NRF24L01_RxFifoEmpty( nrf_config );
}

uint8_t tm2_NRF24L01_RxFifoEmpty( NRF24L01_config_TypeDef *nrf_config ) {
	uint8_t reg = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_FIFO_STATUS );
	return NRF24L01_CHECK_BIT( reg, NRF24L01_RX_EMPTY );
}

uint8_t tm2_NRF24L01_GetStatus( NRF24L01_config_TypeDef *nrf_config ) {
	uint8_t status;
	unsigned char cData;

	tm2_NRF24L01_CSN_LOW( nrf_config );
	/* First received byte is always status register */
	//status = TM_SPI_Send(NRF24L01_SPI, NRF24L01_NOP_MASK);
	cData = NRF24L01_NOP_MASK;
	HAL_SPI_TransmitReceive( nrf_config->SPI, &cData, &status, 1, 100 );
	/* Pull up chip select */
	tm2_NRF24L01_CSN_HIGH( nrf_config );

	return status;
}

TM_NRF24L01_Transmit_Status_t tm2_NRF24L01_GetTransmissionStatus( NRF24L01_config_TypeDef *nrf_config ) {
	uint8_t status = tm2_NRF24L01_GetStatus( nrf_config );
	if( NRF24L01_CHECK_BIT(status, NRF24L01_TX_DS )) {
		/* Successfully sent */
		return TM_NRF24L01_Transmit_Status_Ok;
	} else if( NRF24L01_CHECK_BIT( status, NRF24L01_MAX_RT )) {
		/* Message lost */
		return TM_NRF24L01_Transmit_Status_Lost;
	}

	/* Still sending */
	return TM_NRF24L01_Transmit_Status_Sending;
}

void tm2_NRF24L01_SoftwareReset( NRF24L01_config_TypeDef *nrf_config ) {
	uint8_t data[5];

	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_CONFIG, 	 NRF24L01_REG_DEFAULT_VAL_CONFIG );
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_EN_AA,		 NRF24L01_REG_DEFAULT_VAL_EN_AA );
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_EN_RXADDR,  NRF24L01_REG_DEFAULT_VAL_EN_RXADDR );
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_SETUP_AW, 	 NRF24L01_REG_DEFAULT_VAL_SETUP_AW );
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_SETUP_RETR, NRF24L01_REG_DEFAULT_VAL_SETUP_RETR );
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_RF_CH, 	 NRF24L01_REG_DEFAULT_VAL_RF_CH );
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_RF_SETUP, 	 NRF24L01_REG_DEFAULT_VAL_RF_SETUP );
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_STATUS, 	 NRF24L01_REG_DEFAULT_VAL_STATUS );
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_OBSERVE_TX, NRF24L01_REG_DEFAULT_VAL_OBSERVE_TX );
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_RPD, 		 NRF24L01_REG_DEFAULT_VAL_RPD );

	//P0
	data[0] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_0;
	data[1] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_1;
	data[2] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_2;
	data[3] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_3;
	data[4] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_4;
	tm2_NRF24L01_WriteRegisterMulti( nrf_config, NRF24L01_REG_RX_ADDR_P0, data, 5 );

	//P1
	data[0] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_0;
	data[1] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_1;
	data[2] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_2;
	data[3] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_3;
	data[4] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_4;
	tm2_NRF24L01_WriteRegisterMulti( nrf_config, NRF24L01_REG_RX_ADDR_P1, data, 5 );

	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_RX_ADDR_P2, NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P2 );
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_RX_ADDR_P3, NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P3 );
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_RX_ADDR_P4, NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P4 );
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_RX_ADDR_P5, NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P5 );

	//TX
	data[0] = NRF24L01_REG_DEFAULT_VAL_TX_ADDR_0;
	data[1] = NRF24L01_REG_DEFAULT_VAL_TX_ADDR_1;
	data[2] = NRF24L01_REG_DEFAULT_VAL_TX_ADDR_2;
	data[3] = NRF24L01_REG_DEFAULT_VAL_TX_ADDR_3;
	data[4] = NRF24L01_REG_DEFAULT_VAL_TX_ADDR_4;
	tm2_NRF24L01_WriteRegisterMulti( nrf_config, NRF24L01_REG_TX_ADDR, data, 5 );

	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_RX_PW_P0, 	  NRF24L01_REG_DEFAULT_VAL_RX_PW_P0 );
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_RX_PW_P1, 	  NRF24L01_REG_DEFAULT_VAL_RX_PW_P1 );
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_RX_PW_P2, 	  NRF24L01_REG_DEFAULT_VAL_RX_PW_P2 );
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_RX_PW_P3, 	  NRF24L01_REG_DEFAULT_VAL_RX_PW_P3 );
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_RX_PW_P4, 	  NRF24L01_REG_DEFAULT_VAL_RX_PW_P4 );
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_RX_PW_P5, 	  NRF24L01_REG_DEFAULT_VAL_RX_PW_P5 );
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_FIFO_STATUS, NRF24L01_REG_DEFAULT_VAL_FIFO_STATUS );
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_DYNPD, 	  NRF24L01_REG_DEFAULT_VAL_DYNPD );
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_FEATURE, 	  NRF24L01_REG_DEFAULT_VAL_FEATURE );
}

uint8_t tm2_NRF24L01_GetRetransmissionsCount( NRF24L01_config_TypeDef *nrf_config ) {
	/* Low 4 bits */
	return tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_OBSERVE_TX ) & 0x0F;
}

void tm2_NRF24L01_SetChannel( NRF24L01_config_TypeDef *nrf_config ) {
	if( nrf_config->radio_channel <= 125 ) {
		/* Write channel */
		tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_RF_CH, nrf_config->radio_channel );
	}
}

uint8_t tm2_NRF24L01_SetRF( NRF24L01_config_TypeDef *nrf_config ) {
	uint8_t tmp = 0;
	uint8_t control_value;

	if (nrf_config->baud_rate == TM_NRF24L01_DataRate_2M) {
		tmp |= 1 << NRF24L01_RF_DR_HIGH;
	} else if (nrf_config->baud_rate == TM_NRF24L01_DataRate_250k) {
		tmp |= 1 << NRF24L01_RF_DR_LOW;
	}
	/* If 1Mbps, all bits set to 0 */

	if (nrf_config->output_power == TM_NRF24L01_OutputPower_0dBm) {
		tmp |= 3 << NRF24L01_RF_PWR;
	} else if (nrf_config->output_power == TM_NRF24L01_OutputPower_M6dBm) {
		tmp |= 2 << NRF24L01_RF_PWR;
	} else if (nrf_config->output_power == TM_NRF24L01_OutputPower_M12dBm) {
		tmp |= 1 << NRF24L01_RF_PWR;
	}

	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_RF_SETUP, tmp );
	control_value = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_RF_SETUP );
	if( control_value != tmp ) return 1;

	return 0;
}

//uint8_t TM_NRF24L01_Read_Interrupts(TM_NRF24L01_IRQ_t* IRQ) {
//	IRQ->Status = TM_NRF24L01_GetStatus();
//	return IRQ->Status;
//}

void tm2_NRF24L01_Clear_Interrupts( NRF24L01_config_TypeDef *nrf_config ) {
	tm2_NRF24L01_WriteRegister( nrf_config, NRF24L01_REG_STATUS, 0x70 );
}

void tm2_NRF24L01_ReadConfig( NRF24L01_config_TypeDef *nrf_config, uint8_t* result ) {
	uint8_t data[5];

	result[ 0 ] = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_CONFIG );
	result[ 1 ] = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_EN_AA );
	result[ 2 ] = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_EN_RXADDR );
	result[ 3 ] = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_SETUP_AW );
	result[ 4 ] = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_SETUP_RETR );
	result[ 5 ] = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_RF_CH );
	result[ 6 ] = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_RF_SETUP );
	result[ 7 ] = 65; //tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_STATUS );
	result[ 8 ] = 66; //tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_OBSERVE_TX );
	result[ 9 ] = 67; //tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_RPD );

	//P0
	tm2_NRF24L01_ReadRegisterMulti( nrf_config, NRF24L01_REG_RX_ADDR_P0, data, 5 );
	result[ 10 ] = data[ 0 ];
	result[ 11 ] = data[ 1 ];
	result[ 12 ] = data[ 2 ];
	result[ 13 ] = data[ 3 ];
	result[ 14 ] = data[ 4 ];

	//P1
	tm2_NRF24L01_ReadRegisterMulti( nrf_config, NRF24L01_REG_RX_ADDR_P1, data, 5 );
	result[ 15 ] = data[ 0 ];
	result[ 16 ] = data[ 1 ];
	result[ 17 ] = data[ 2 ];
	result[ 18 ] = data[ 3 ];
	result[ 19 ] = data[ 4 ];

	result[ 20 ] = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_RX_ADDR_P2 );
	result[ 21 ] = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_RX_ADDR_P3 );
	result[ 22 ] = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_RX_ADDR_P4 );
	result[ 23 ] = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_RX_ADDR_P5 );

	//TX
	tm2_NRF24L01_ReadRegisterMulti( nrf_config, NRF24L01_REG_TX_ADDR, data, 5 );
	result[ 24 ] = data[ 0 ];
	result[ 25 ] = data[ 1 ];
	result[ 26 ] = data[ 2 ];
	result[ 27 ] = data[ 3 ];
	result[ 28 ] = data[ 4 ];

	result[ 29 ] = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_DEFAULT_VAL_RX_PW_P0 );
	result[ 30 ] = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_DEFAULT_VAL_RX_PW_P1 );
	result[ 31 ] = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_DEFAULT_VAL_RX_PW_P2 );
	result[ 32 ] = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_DEFAULT_VAL_RX_PW_P3 );
	result[ 33 ] = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_DEFAULT_VAL_RX_PW_P4 );
	result[ 34 ] = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_DEFAULT_VAL_RX_PW_P5 );
	result[ 35 ] = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_DEFAULT_VAL_FIFO_STATUS );
	result[ 36 ] = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_DEFAULT_VAL_DYNPD );
	result[ 37 ] = tm2_NRF24L01_ReadRegister( nrf_config, NRF24L01_REG_DEFAULT_VAL_FEATURE );
}

