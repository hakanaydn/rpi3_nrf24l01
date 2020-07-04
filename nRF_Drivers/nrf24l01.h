/**
******************************************************************************
* @file         nrf24l01.h
* @authors      Hakan AYDIN
* @date         05-May-2019
* @version      V0.0.1
* @copyright    Copyright (c) 2019
* @brief        nrf24l01 Header File.
******************************************************************************/
#ifndef NRF24L01_H_
#define NRF24L01_H_

/*****************************************************************************/
/* INCLUDES                                                                  */
/*****************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <stdbool.h>

/*****************************************************************************/
/* DEFINITIONS                                                               */
/*****************************************************************************/
// nRF24L0 instruction definitions
#define nRF24_CMD_R_REGISTER       (unsigned char)0x00 // Register read
#define nRF24_CMD_W_REGISTER       (unsigned char)0x20 // Register write
#define nRF24_CMD_R_RX_PAYLOAD     (unsigned char)0x61 // Read RX payload
#define nRF24_CMD_W_TX_PAYLOAD     (unsigned char)0xA0 // Write TX payload
#define nRF24_CMD_FLUSH_TX         (unsigned char)0xE1 // Flush TX FIFO
#define nRF24_CMD_FLUSH_RX         (unsigned char)0xE2 // Flush RX FIFO
#define nRF24_CMD_REUSE_TX_PL      (unsigned char)0xE3 // Reuse TX payload
#define nRF24_CMD_LOCK_UNLOCK      (unsigned char)0x50 // Lock/unlock exclusive features
#define nRF24_CMD_NOP              (unsigned char)0xFF // No operation (used for reading status register)

// nRF24L0 register definitions
#define nRF24_REG_CONFIG           (unsigned char)0x00 // Configuration register
#define nRF24_REG_EN_AA            (unsigned char)0x01 // Enable "Auto acknowledgment"
#define nRF24_REG_EN_RXADDR        (unsigned char)0x02 // Enable RX addresses
#define nRF24_REG_SETUP_AW         (unsigned char)0x03 // Setup of address widths
#define nRF24_REG_SETUP_RETR       (unsigned char)0x04 // Setup of automatic retransmit
#define nRF24_REG_RF_CH            (unsigned char)0x05 // RF channel
#define nRF24_REG_RF_SETUP         (unsigned char)0x06 // RF setup register
#define nRF24_REG_STATUS           (unsigned char)0x07 // Status register
#define nRF24_REG_OBSERVE_TX       (unsigned char)0x08 // Transmit observe register
#define nRF24_REG_RPD              (unsigned char)0x09 // Received power detector
#define nRF24_REG_RX_ADDR_P0       (unsigned char)0x0A // Receive address data pipe 0
#define nRF24_REG_RX_ADDR_P1       (unsigned char)0x0B // Receive address data pipe 1
#define nRF24_REG_RX_ADDR_P2       (unsigned char)0x0C // Receive address data pipe 2
#define nRF24_REG_RX_ADDR_P3       (unsigned char)0x0D // Receive address data pipe 3
#define nRF24_REG_RX_ADDR_P4       (unsigned char)0x0E // Receive address data pipe 4
#define nRF24_REG_RX_ADDR_P5       (unsigned char)0x0F // Receive address data pipe 5
#define nRF24_REG_TX_ADDR          (unsigned char)0x10 // Transmit address
#define nRF24_REG_RX_PW_P0         (unsigned char)0x11 // Number of bytes in RX payload in data pipe 0
#define nRF24_REG_RX_PW_P1         (unsigned char)0x12 // Number of bytes in RX payload in data pipe 1
#define nRF24_REG_RX_PW_P2         (unsigned char)0x13 // Number of bytes in RX payload in data pipe 2
#define nRF24_REG_RX_PW_P3         (unsigned char)0x14 // Number of bytes in RX payload in data pipe 3
#define nRF24_REG_RX_PW_P4         (unsigned char)0x15 // Number of bytes in RX payload in data pipe 4
#define nRF24_REG_RX_PW_P5         (unsigned char)0x16 // Number of bytes in RX payload in data pipe 5
#define nRF24_REG_FIFO_STATUS      (unsigned char)0x17 // FIFO status register
#define nRF24_REG_DYNPD            (unsigned char)0x1C // Enable dynamic payload length
#define nRF24_REG_FEATURE          (unsigned char)0x1D // Feature register

// Register bits definitions
#define nRF24_CONFIG_PRIM_RX       (unsigned char)0x01 // PRIM_RX bit in CONFIG register
#define nRF24_CONFIG_PWR_UP        (unsigned char)0x02 // PWR_UP bit in CONFIG register
#define nRF24_FLAG_RX_DR           (unsigned char)0x40 // RX_DR bit (data ready RX FIFO interrupt)
#define nRF24_FLAG_TX_DS           (unsigned char)0x20 // TX_DS bit (data sent TX FIFO interrupt)
#define nRF24_FLAG_MAX_RT          (unsigned char)0x10 // MAX_RT bit (maximum number of TX retransmits interrupt)

// Register masks definitions
#define nRF24_MASK_REG_MAP         (unsigned char)0x1F // Mask bits[4:0] for CMD_RREG and CMD_WREG commands
#define nRF24_MASK_CRC             (unsigned char)0x0C // Mask for CRC bits [3:2] in CONFIG register
#define nRF24_MASK_STATUS_IRQ      (unsigned char)0x70 // Mask for all IRQ bits in STATUS register
#define nRF24_MASK_RF_PWR          (unsigned char)0x06 // Mask RF_PWR[2:1] bits in RF_SETUP register
#define nRF24_MASK_RX_P_NO         (unsigned char)0x0E // Mask RX_P_NO[3:1] bits in STATUS register
#define nRF24_MASK_DATARATE        (unsigned char)0x28 // Mask RD_DR_[5,3] bits in RF_SETUP register
#define nRF24_MASK_EN_RX           (unsigned char)0x3F // Mask ERX_P[5:0] bits in EN_RXADDR register
#define nRF24_MASK_RX_PW           (unsigned char)0x3F // Mask [5:0] bits in RX_PW_Px register
#define nRF24_MASK_RETR_ARD        (unsigned char)0xF0 // Mask for ARD[7:4] bits in SETUP_RETR register
#define nRF24_MASK_RETR_ARC        (unsigned char)0x0F // Mask for ARC[3:0] bits in SETUP_RETR register
#define nRF24_MASK_RXFIFO          (unsigned char)0x03 // Mask for RX FIFO status bits [1:0] in FIFO_STATUS register
#define nRF24_MASK_TXFIFO          (unsigned char)0x30 // Mask for TX FIFO status bits [5:4] in FIFO_STATUS register
#define nRF24_MASK_PLOS_CNT        (unsigned char)0xF0 // Mask for PLOS_CNT[7:4] bits in OBSERVE_TX register
#define nRF24_MASK_ARC_CNT         (unsigned char)0x0F // Mask for ARC_CNT[3:0] bits in OBSERVE_TX register

// Fake address to test transceiver presence (5 bytes long)
#define nRF24_TEST_ADDR            "nRF24"

// Retransmit delay
enum {
	nRF24_ARD_NONE   = 0x00, // Dummy value for case when retransmission is not used
	nRF24_ARD_250us  = 0x00,
	nRF24_ARD_500us  = 0x01,
	nRF24_ARD_750us  = 0x02,
	nRF24_ARD_1000us = 0x03,
	nRF24_ARD_1250us = 0x04,
	nRF24_ARD_1500us = 0x05,
	nRF24_ARD_1750us = 0x06,
	nRF24_ARD_2000us = 0x07,
	nRF24_ARD_2250us = 0x08,
	nRF24_ARD_2500us = 0x09,
	nRF24_ARD_2750us = 0x0A,
	nRF24_ARD_3000us = 0x0B,
	nRF24_ARD_3250us = 0x0C,
	nRF24_ARD_3500us = 0x0D,
	nRF24_ARD_3750us = 0x0E,
	nRF24_ARD_4000us = 0x0F
};

// Data rate
enum {
	nRF24_DR_250kbps = 0x20, // 250kbps data rate
	nRF24_DR_1Mbps   = 0x00, // 1Mbps data rate
	nRF24_DR_2Mbps   = 0x08  // 2Mbps data rate
};

// RF output power in TX mode
enum {
	nRF24_TXPWR_18dBm = 0x00, // -18dBm
	nRF24_TXPWR_12dBm = 0x02, // -12dBm
	nRF24_TXPWR_6dBm  = 0x04, //  -6dBm
	nRF24_TXPWR_0dBm  = 0x06  //   0dBm
};

// CRC encoding scheme
enum {
	nRF24_CRC_off   = 0x00, // CRC disabled
	nRF24_CRC_1byte = 0x08, // 1-byte CRC
	nRF24_CRC_2byte = 0x0c  // 2-byte CRC
};

// nRF24L01 power control
enum {
	nRF24_PWR_UP   = 0x02, // Power up
	nRF24_PWR_DOWN = 0x00  // Power down
};

// Transceiver mode
enum {
	nRF24_MODE_RX = 0x01, // PRX
	nRF24_MODE_TX = 0x00  // PTX
};

// Enumeration of RX pipe addresses and TX address
enum {
	nRF24_PIPE0  = 0x00, // pipe0
	nRF24_PIPE1  = 0x01, // pipe1
	nRF24_PIPE2  = 0x02, // pipe2
	nRF24_PIPE3  = 0x03, // pipe3
	nRF24_PIPE4  = 0x04, // pipe4
	nRF24_PIPE5  = 0x05, // pipe5
	nRF24_PIPETX = 0x06  // TX address (not a pipe in fact)
};

// State of auto acknowledgment for specified pipe
enum {
	nRF24_AA_OFF = 0x00,
	nRF24_AA_ON  = 0x01
};

// Status of the RX FIFO
enum {
	nRF24_STATUS_RXFIFO_DATA  = 0x00, // The RX FIFO contains data and available locations
	nRF24_STATUS_RXFIFO_EMPTY = 0x01, // The RX FIFO is empty
	nRF24_STATUS_RXFIFO_FULL  = 0x02, // The RX FIFO is full
	nRF24_STATUS_RXFIFO_ERROR = 0x03  // Impossible state: RX FIFO cannot be empty and full at the same time
};

// Status of the TX FIFO
enum {
	nRF24_STATUS_TXFIFO_DATA  = 0x00, // The TX FIFO contains data and available locations
	nRF24_STATUS_TXFIFO_EMPTY = 0x01, // The TX FIFO is empty
	nRF24_STATUS_TXFIFO_FULL  = 0x02, // The TX FIFO is full
	nRF24_STATUS_TXFIFO_ERROR = 0x03  // Impossible state: TX FIFO cannot be empty and full at the same time
};

// Result of RX FIFO reading
typedef enum {
	nRF24_RX_PIPE0  = 0x00, // Packet received from the PIPE#0
	nRF24_RX_PIPE1  = 0x01, // Packet received from the PIPE#1
	nRF24_RX_PIPE2  = 0x02, // Packet received from the PIPE#2
	nRF24_RX_PIPE3  = 0x03, // Packet received from the PIPE#3
	nRF24_RX_PIPE4  = 0x04, // Packet received from the PIPE#4
	nRF24_RX_PIPE5  = 0x05, // Packet received from the PIPE#5
	nRF24_RX_EMPTY  = 0xff  // The RX FIFO is empty
} nRF24_RXResult;

typedef enum {
	RX_DataReady = 0x00,
	TX_Ack,
	TX_Fail,
	nRF_NONE
}nRF24_IRQ_Status;

typedef union {
	struct
	{
		unsigned char sTX_FULL : 1;
		unsigned char sRX_P_NO : 3;
		unsigned char sMAX_RT  : 1;
		unsigned char sTX_DS   : 1;
		unsigned char sRX_DR   : 1;
		unsigned char sRsvd    : 1;
	}Bits;
	unsigned char AllPacket;
}nRF24_IRQ_StatusPacket;

typedef struct {

	nRF24_IRQ_StatusPacket tPacket;
	nRF24_IRQ_Status eStatus;

}nRF24_Status;

// Addresses of the RX_PW_P# registers
static const unsigned char nRF24_RX_PW_PIPE[6] = {
		nRF24_REG_RX_PW_P0,
		nRF24_REG_RX_PW_P1,
		nRF24_REG_RX_PW_P2,
		nRF24_REG_RX_PW_P3,
		nRF24_REG_RX_PW_P4,
		nRF24_REG_RX_PW_P5
};

// Addresses of the address registers
static const unsigned char nRF24_ADDR_REGS[7] = {
		nRF24_REG_RX_ADDR_P0,
		nRF24_REG_RX_ADDR_P1,
		nRF24_REG_RX_ADDR_P2,
		nRF24_REG_RX_ADDR_P3,
		nRF24_REG_RX_ADDR_P4,
		nRF24_REG_RX_ADDR_P5,
		nRF24_REG_TX_ADDR
};

// Function prototypes
bool nRF24_Init(void);
unsigned char nRF24_Check(void);

void nRF24_SetPowerMode(unsigned char mode);
void nRF24_SetOperationalMode(unsigned char mode);
void nRF24_SetRFChannel(unsigned char channel);
void nRF24_SetAutoRetr(unsigned char ard, unsigned char arc);
void nRF24_SetAddrWidth(unsigned char addr_width);
void nRF24_SetAddr(unsigned char pipe, const unsigned char *addr);
void nRF24_SetTXPower(unsigned char tx_pwr);
void nRF24_SetDataRate(unsigned char data_rate);
void nRF24_SetCRCScheme(unsigned char scheme);
void nRF24_SetRXPipe(unsigned char pipe, unsigned char aa_state, unsigned char payload_len);
void nRF24_ClosePipe(unsigned char pipe);
void nRF24_EnableAA(unsigned char pipe);
void nRF24_DisableAA(unsigned char pipe);
void nRF24_spi_begin(unsigned char cs);

unsigned char nRF24_GetStatus(void);
unsigned char nRF24_GetIRQFlags(void);
unsigned char nRF24_GetStatus_RXFIFO(void);
unsigned char nRF24_GetStatus_TXFIFO(void);
unsigned char nRF24_GetRXSource(void);
unsigned char nRF24_GetRetransmitCounters(void);

void nRF24_ResetPLOS(void);
void nRF24_FlushTX(void);
void nRF24_FlushRX(void);
void nRF24_ClearIRQFlags(void);

void nRF24_CE_H (void);


void nRF24_WritePayload(unsigned char *pBuf, unsigned char length);
nRF24_RXResult nRF24_ReadPayload(unsigned char *pBuf, unsigned char *length);

nRF24_Status nRF24_IRQ_Handler(void);

#endif /* NRF24L01_H_ */
/*****************************************************************************/
/* END OF FILE */
/*****************************************************************************/
