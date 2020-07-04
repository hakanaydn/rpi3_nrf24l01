/**
******************************************************************************
* @file         nRF24L01.c
* @authors      Hakan AYDIN
* @date         27-Arpil-2019
* @version      V0.0.1
* @copyright    Copyright (c) 2019
* @brief        AudioRecorder Header File.
******************************************************************************
*/
/*****************************************************************************/
/* INCLUDES 																																 */
/*****************************************************************************/
#include "nrf24l01.h"
#include "../BCM/bcm2835.h"
#include "../COMMON/mxconstants.h"

/*****************************************************************************/
/* STRUCTURE/ENUM DEFINITIONS                                                */
/*****************************************************************************/

/*****************************************************************************/
/* STRUCT/ENUM                                                               */
/*****************************************************************************/

/*****************************************************************************/
/* GLOBALS VARIABLES                                                         */
/*****************************************************************************/

/*****************************************************************************/
/* STATICS FUNCTIONS 																												 */
/*****************************************************************************/
static unsigned char nRF24_LL_RW(unsigned char data);
static void nRF24_CSN_L(void);
static void nRF24_CSN_H(void);
static void nRF24_CE_L (void);


/*****************************************************************************/
/* FUNCTIONS 																																 */
/*****************************************************************************/
/**
  * @brief
  * @param
  * @retval
  */
static void nRF24_CSN_L(void)
{
    //HAL_GPIO_WritePin(nRF_CS_GPIO_Port, nRF_CS_Pin, GPIO_PIN_RESET);
    bcm2835_gpio_write(nRF24_CS, LOW);
}

/**
  * @brief
  * @param
  * @retval
  */
static void nRF24_CSN_H(void)
{
    // HAL_GPIO_WritePin(nRF_CS_GPIO_Port, nRF_CS_Pin, GPIO_PIN_SET);
    bcm2835_gpio_write(nRF24_CS, HIGH);
}

/**
  * @brief
  * @param
  * @retval
  */
static void nRF24_CE_L (void)
{
   // HAL_GPIO_WritePin(nRF_CE_GPIO_Port, nRF_CE_Pin, GPIO_PIN_RESET)
    bcm2835_gpio_write(nRF24_CE, LOW);
}

/**
  * @brief
  * @param
  * @retval
  */

void nRF24_CE_H (void)
{
   // HAL_GPIO_WritePin(nRF_CE_GPIO_Port, nRF_CE_Pin, GPIO_PIN_SET)
   bcm2835_gpio_write(nRF24_CE, HIGH);
}

/**
  * @brief
  * @param
  * @retval
  */
static unsigned char nRF24_LL_RW(unsigned char data)
{
	unsigned char rx_=0;
  rx_ = bcm2835_spi_transfer(data);
	return rx_;
}

/**
  * @brief
  * @param
  * @retval
  */
static unsigned char nRF24_ReadReg(unsigned char reg) {
	unsigned char value;

	nRF24_CSN_L();
	nRF24_LL_RW(reg & nRF24_MASK_REG_MAP);
	value = nRF24_LL_RW(nRF24_CMD_NOP);
	nRF24_CSN_H();

	return value;
}

/**
  * @brief
  * @param
  * @retval
  */
static void nRF24_WriteReg(unsigned char reg, unsigned char value) {
	nRF24_CSN_L();
	if (reg < nRF24_CMD_W_REGISTER) {
		// This is a register access
		nRF24_LL_RW(nRF24_CMD_W_REGISTER | (reg & nRF24_MASK_REG_MAP));
		nRF24_LL_RW(value);
	} else {
		// This is a single byte command or future command/register
		nRF24_LL_RW(reg);
		if ((reg != nRF24_CMD_FLUSH_TX) && (reg != nRF24_CMD_FLUSH_RX) && \
				(reg != nRF24_CMD_REUSE_TX_PL) && (reg != nRF24_CMD_NOP)) {
			// Send register value
			nRF24_LL_RW(value);
		}
	}
	nRF24_CSN_H();
}

/**
  * @brief
  * @param
  * @retval
  */
static void nRF24_ReadMBReg(unsigned char reg, unsigned char *pBuf, unsigned char count) {
	nRF24_CSN_L();
	nRF24_LL_RW(reg);
	while (count--) {
		*pBuf++ = nRF24_LL_RW(nRF24_CMD_NOP);
	}
	nRF24_CSN_H();
}

/**
  * @brief
  * @param
  * @retval
  */
static void nRF24_WriteMBReg(unsigned char reg, unsigned char *pBuf, unsigned char count) {
	nRF24_CSN_L();
	nRF24_LL_RW(reg);
	while (count--) {
		nRF24_LL_RW(*pBuf++);
	}
	nRF24_CSN_H();
}

/**
  * @brief
  * @param
  * @retval
  */

  void nRF24_spi_begin(uint8_t cs)
  {
      volatile uint32_t* paddr = bcm2835_spi0 + BCM2835_SPI0_CS/4;

      // Set the SPI0 pins to the Alt 0 function to enable SPI0 access on them
  		// except if we need custom Chip Select Pin
  		printf("bcm2835_spi_begin -> spi_custom_cs = %d \n",cs );

  		bcm2835_gpio_fsel(cs, BCM2835_GPIO_FSEL_OUTP);
  		bcm2835_gpio_write(cs, HIGH);

  		// Classic pin, hardware driven
      bcm2835_gpio_fsel(RPI_GPIO_P1_21, BCM2835_GPIO_FSEL_ALT0); // MISO
      bcm2835_gpio_fsel(RPI_GPIO_P1_19, BCM2835_GPIO_FSEL_ALT0); // MOSI
      bcm2835_gpio_fsel(RPI_GPIO_P1_23, BCM2835_GPIO_FSEL_ALT0); // CLK

      // Set the SPI CS register to the some sensible defaults
      bcm2835_peri_write(paddr, 0); // All 0s

      // Clear TX and RX fifos
      bcm2835_peri_write_nb(paddr, BCM2835_SPI0_CS_CLEAR);
}


/**
  * @brief
  * @param
  * @retval
  */
bool nRF24_Init(void)
{
  //F24 radio(RPI_V2_GPIO_P1_15, RPI_V2_GPIO_P1_26, BCM2835_SPI_SPEED_8MHZ);

  // Init BCM2835 chipset for talking with us
  if (!bcm2835_init())
  {
      return false;
  }

  // Initialise the CE pin of NRF24 (chip enable)
  bcm2835_gpio_fsel(nRF24_CE, BCM2835_GPIO_FSEL_OUTP);

  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
  bcm2835_spi_setClockSpeed(nRF24_SPI_Speed);
  nRF24_spi_begin(nRF24_CS);

  nRF24_CE_L();
  nRF24_CSN_H();

  delay(1);

	// Write to registers their initial values
	nRF24_WriteReg(nRF24_REG_CONFIG, 0x08);
	nRF24_WriteReg(nRF24_REG_EN_AA, 0x3F);
	nRF24_WriteReg(nRF24_REG_EN_RXADDR, 0x03);
	nRF24_WriteReg(nRF24_REG_SETUP_AW, 0x03);
	nRF24_WriteReg(nRF24_REG_SETUP_RETR, 0x03);
	nRF24_WriteReg(nRF24_REG_RF_CH, 0x02);
	nRF24_WriteReg(nRF24_REG_RF_SETUP, 0x0E);
	nRF24_WriteReg(nRF24_REG_STATUS, 0x00);
	nRF24_WriteReg(nRF24_REG_RX_PW_P0, 0x00);
	nRF24_WriteReg(nRF24_REG_RX_PW_P1, 0x00);
	nRF24_WriteReg(nRF24_REG_RX_PW_P2, 0x00);
	nRF24_WriteReg(nRF24_REG_RX_PW_P3, 0x00);
	nRF24_WriteReg(nRF24_REG_RX_PW_P4, 0x00);
	nRF24_WriteReg(nRF24_REG_RX_PW_P5, 0x00);
	nRF24_WriteReg(nRF24_REG_DYNPD, 0x00);
	nRF24_WriteReg(nRF24_REG_FEATURE, 0x00);

	// Clear the FIFO's
	nRF24_FlushRX();
	nRF24_FlushTX();

	// Clear any pending interrupt flags
	nRF24_ClearIRQFlags();

	// Deassert CSN pin (chip release)
	nRF24_CSN_H();

  return true;
}

/**
  * @brief
  * @param
  * @retval
  */
unsigned char nRF24_Check(void) {
	unsigned char rxbuf[5];
	unsigned char i;
	unsigned char *ptr = (unsigned char *)nRF24_TEST_ADDR;

	// Write test TX address and read TX_ADDR register
	nRF24_WriteMBReg(nRF24_CMD_W_REGISTER | nRF24_REG_TX_ADDR, ptr, 5);
	nRF24_ReadMBReg(nRF24_CMD_R_REGISTER | nRF24_REG_TX_ADDR, rxbuf, 5);


  printf("%c %c %c %c %c \n",rxbuf[0],rxbuf[1] ,rxbuf[2] ,rxbuf[3] ,rxbuf[4]  );

	// Compare buffers, return error on first mismatch
	for (i = 0; i < 5; i++) {
		if (rxbuf[i] != *ptr++) return 0;
	}

	return 1;
}

/**
  * @brief
  * @param
  * @retval
  */
void nRF24_FlushTX(void) {
	nRF24_WriteReg(nRF24_CMD_FLUSH_TX, nRF24_CMD_NOP);
}

/**
  * @brief
  * @param
  * @retval
  */
void nRF24_FlushRX(void) {
	nRF24_WriteReg(nRF24_CMD_FLUSH_RX, nRF24_CMD_NOP);
}

/**
  * @brief
  * @param
  * @retval
  */
void nRF24_ClearIRQFlags(void) {
	uint8_t reg;

	// Clear RX_DR, TX_DS and MAX_RT bits of the STATUS register
	reg  = nRF24_ReadReg(nRF24_REG_STATUS);
	reg |= nRF24_MASK_STATUS_IRQ;
	nRF24_WriteReg(nRF24_REG_STATUS, reg);
}

/**
  * @brief
  * @param
  * @retval
  */
void nRF24_SetPowerMode(unsigned char mode) {
	unsigned char reg;

	reg = nRF24_ReadReg(nRF24_REG_CONFIG);
	if (mode == nRF24_PWR_UP) {
		// Set the PWR_UP bit of CONFIG register to wake the transceiver
		// It goes into Stanby-I mode with consumption about 26uA
		reg |= nRF24_CONFIG_PWR_UP;
	} else {
		// Clear the PWR_UP bit of CONFIG register to put the transceiver
		// into power down mode with consumption about 900nA
		reg &= ~nRF24_CONFIG_PWR_UP;
	}
	nRF24_WriteReg(nRF24_REG_CONFIG, reg);
}

/**
  * @brief
  * @param
  * @retval
  */
void nRF24_SetOperationalMode(unsigned char mode) {
	unsigned char reg;

	// Configure PRIM_RX bit of the CONFIG register
	reg  = nRF24_ReadReg(nRF24_REG_CONFIG);
	reg &= ~nRF24_CONFIG_PRIM_RX;
	reg |= (mode & nRF24_CONFIG_PRIM_RX);
	nRF24_WriteReg(nRF24_REG_CONFIG, reg);
}

/**
  * @brief
  * @param
  * @retval
  */
void nRF24_SetCRCScheme(unsigned char scheme) {
	unsigned char reg;

	// Configure EN_CRC[3] and CRCO[2] bits of the CONFIG register
	reg  = nRF24_ReadReg(nRF24_REG_CONFIG);
	reg &= ~nRF24_MASK_CRC;
	reg |= (scheme & nRF24_MASK_CRC);
	nRF24_WriteReg(nRF24_REG_CONFIG, reg);
}

/**
  * @brief
  * @param
  * @retval
  */
void nRF24_SetAddr(unsigned char pipe, const unsigned char *addr) {
	unsigned char addr_width;

// Set static RX address for a specified pipe
// input:
//   pipe - pipe to configure address, one of nRF24_PIPEx values
//   addr - pointer to the buffer with address
// note: pipe can be a number from 0 to 5 (RX pipes) and 6 (TX pipe)
// note: buffer length must be equal to current address width of transceiver
// note: for pipes[2..5] only first byte of address will be written because
//       other bytes of address equals to pipe1
// note: for pipes[2..5] only first byte of address will be written because
//       pipes 1-5 share the four most significant address bytes

	// RX_ADDR_Px register
	switch (pipe) {
		case nRF24_PIPETX:
		case nRF24_PIPE0:
		case nRF24_PIPE1:
			// Get address width
			addr_width = nRF24_ReadReg(nRF24_REG_SETUP_AW) + 1;
			// Write address in reverse order (LSByte first)
			addr += addr_width;
			nRF24_CSN_L();
			nRF24_LL_RW(nRF24_CMD_W_REGISTER | nRF24_ADDR_REGS[pipe]);
			do {
				nRF24_LL_RW(*addr--);
			} while (addr_width--);
			nRF24_CSN_H();
			break;
		case nRF24_PIPE2:
		case nRF24_PIPE3:
		case nRF24_PIPE4:
		case nRF24_PIPE5:
			// Write address LSBbyte (only first byte from the addr buffer)
			nRF24_WriteReg(nRF24_ADDR_REGS[pipe], *addr);
			break;
		default:
			// Incorrect pipe number -> do nothing
			break;
	}
}


/**
  * @brief
  * @param
  * @retval
  */
void nRF24_SetTXPower(unsigned char tx_pwr) {
	unsigned char reg;
// Configure RF output power in TX mode
// input:
//   tx_pwr - RF output power, one of nRF24_TXPWR_xx values

	// Configure RF_PWR[2:1] bits of the RF_SETUP register
	reg  = nRF24_ReadReg(nRF24_REG_RF_SETUP);
	reg &= ~nRF24_MASK_RF_PWR;
	reg |= tx_pwr;
	nRF24_WriteReg(nRF24_REG_RF_SETUP, reg);
}


/**
  * @brief
  * @param
  * @retval
  */
void nRF24_SetDataRate(unsigned char data_rate) {
	unsigned char reg;
// Configure transceiver data rate
// input:
//   data_rate - data rate, one of nRF24_DR_xx values
	// Configure RF_DR_LOW[5] and RF_DR_HIGH[3] bits of the RF_SETUP register
	reg  = nRF24_ReadReg(nRF24_REG_RF_SETUP);
	reg &= ~nRF24_MASK_DATARATE;
	reg |= data_rate;
	nRF24_WriteReg(nRF24_REG_RF_SETUP, reg);
}

/**
  * @brief
  * @param
  * @retval
  */
void nRF24_SetRXPipe(unsigned char pipe, unsigned char aa_state, unsigned char payload_len) {
	unsigned char reg;
// Configure a specified RX pipe
// input:
//   pipe - number of the RX pipe, value from 0 to 5
//   aa_state - state of auto acknowledgment, one of nRF24_AA_xx values
//   payload_len - payload length in bytes
	// Enable the specified pipe (EN_RXADDR register)
	reg = (nRF24_ReadReg(nRF24_REG_EN_RXADDR) | (1 << pipe)) & nRF24_MASK_EN_RX;
	nRF24_WriteReg(nRF24_REG_EN_RXADDR, reg);

	// Set RX payload length (RX_PW_Px register)
	nRF24_WriteReg(nRF24_RX_PW_PIPE[pipe], payload_len & nRF24_MASK_RX_PW);

	// Set auto acknowledgment for a specified pipe (EN_AA register)
	reg = nRF24_ReadReg(nRF24_REG_EN_AA);
	if (aa_state == nRF24_AA_ON) {
		reg |=  (1 << pipe);
	} else {
		reg &= ~(1 << pipe);
	}
	nRF24_WriteReg(nRF24_REG_EN_AA, reg);
}

/**
  * @brief
  * @param
  * @retval
  */
void nRF24_ClosePipe(unsigned char pipe) {
	unsigned char reg;
// Disable specified RX pipe
// input:
//   PIPE - number of RX pipe, value from 0 to 5
	reg  = nRF24_ReadReg(nRF24_REG_EN_RXADDR);
	reg &= ~(1 << pipe);
	reg &= nRF24_MASK_EN_RX;
	nRF24_WriteReg(nRF24_REG_EN_RXADDR, reg);
}


/**
  * @brief
  * @param
  * @retval
  */
void nRF24_EnableAA(unsigned char pipe) {
	unsigned char reg;
// Enable the auto retransmit (a.k.a. enhanced ShockBurst) for the specified RX pipe
// input:
//   pipe - number of the RX pipe, value from 0 to 5
	// Set bit in EN_AA register
	reg  = nRF24_ReadReg(nRF24_REG_EN_AA);
	reg |= (1 << pipe);
	nRF24_WriteReg(nRF24_REG_EN_AA, reg);
}

/**
  * @brief
  * @param
  * @retval
  */
void nRF24_DisableAA(unsigned char pipe) {
	unsigned char reg;
// Disable the auto retransmit (a.k.a. enhanced ShockBurst) for one or all RX pipes
// input:
//   pipe - number of the RX pipe, value from 0 to 5, any other value will disable AA for all RX pipes
	if (pipe > 5) {
		// Disable Auto-ACK for ALL pipes
		nRF24_WriteReg(nRF24_REG_EN_AA, 0x00);
	} else {
		// Clear bit in the EN_AA register
		reg  = nRF24_ReadReg(nRF24_REG_EN_AA);
		reg &= ~(1 << pipe);
		nRF24_WriteReg(nRF24_REG_EN_AA, reg);
	}
}


/**
  * @brief
  * @param
  * @retval
  */
unsigned char nRF24_GetStatus(void) {
	// Get value of the STATUS register
// return: value of STATUS register
	return nRF24_ReadReg(nRF24_REG_STATUS);
}

/**
  * @brief
  * @param
  * @retval
  */
unsigned char nRF24_GetIRQFlags(void) {
	// Get pending IRQ flags
// return: current status of RX_DR, TX_DS and MAX_RT bits of the STATUS register
	return (nRF24_ReadReg(nRF24_REG_STATUS) & nRF24_MASK_STATUS_IRQ);
}

/**
  * @brief
  * @param
  * @retval
  */
unsigned char nRF24_GetStatus_RXFIFO(void) {
	// Get status of the RX FIFO
// return: one of the nRF24_STATUS_RXFIFO_xx values
	return (nRF24_ReadReg(nRF24_REG_FIFO_STATUS) & nRF24_MASK_RXFIFO);
}

/**
  * @brief
  * @param
  * @retval
  */
unsigned char nRF24_GetStatus_TXFIFO(void) {

// Get status of the TX FIFO
// return: one of the nRF24_STATUS_TXFIFO_xx values
// note: the TX_REUSE bit ignored
	return ((nRF24_ReadReg(nRF24_REG_FIFO_STATUS) & nRF24_MASK_TXFIFO) >> 4);
}

/**
  * @brief
  * @param
  * @retval
  */
unsigned char nRF24_GetRXSource(void) {
	// Get pipe number for the payload available for reading from RX FIFO
// return: pipe number or 0x07 if the RX FIFO is empty
	return ((nRF24_ReadReg(nRF24_REG_STATUS) & nRF24_MASK_RX_P_NO) >> 1);
}


/**
  * @brief
  * @param
  * @retval
  */
unsigned char nRF24_GetRetransmitCounters(void) {
	// Get auto retransmit statistic
// return: value of OBSERVE_TX register which contains two counters encoded in nibbles:
//   high - lost packets count (max value 15, can be reseted by write to RF_CH register)
//   low  - retransmitted packets count (max value 15, reseted when new transmission starts)
	return (nRF24_ReadReg(nRF24_REG_OBSERVE_TX));
}


/**
  * @brief
  * @param
  * @retval
  */
void nRF24_ResetPLOS(void) {
	unsigned char reg;
// Reset packet lost counter (PLOS_CNT bits in OBSERVER_TX register)
	// The PLOS counter is reset after write to RF_CH register
	reg = nRF24_ReadReg(nRF24_REG_RF_CH);
	nRF24_WriteReg(nRF24_REG_RF_CH, reg);
}


/**
  * @brief
  * @param
  * @retval
  */
void nRF24_WritePayload(unsigned char *pBuf, unsigned char length) {
	// Write TX payload
// input:
//   pBuf - pointer to the buffer with payload data
//   length - payload length in bytes
	nRF24_WriteMBReg(nRF24_CMD_W_TX_PAYLOAD, pBuf, length);
}


/**
  * @brief
  * @param
  * @retval
  */
nRF24_RXResult nRF24_ReadPayload(unsigned char *pBuf, unsigned char *length) {
	unsigned char pipe;

	// Read top level payload available in the RX FIFO
	// input:
	//   pBuf - pointer to the buffer to store a payload data
	//   length - pointer to variable to store a payload length
	// return: one of nRF24_RX_xx values
	//   nRF24_RX_PIPEX - packet has been received from the pipe number X
	//   nRF24_RX_EMPTY - the RX FIFO is empty

	// Extract a payload pipe number from the STATUS register
	pipe = (nRF24_ReadReg(nRF24_REG_STATUS) & nRF24_MASK_RX_P_NO) >> 1;

	// RX FIFO empty?
	if (pipe < 6) {
		// Get payload length
		*length = nRF24_ReadReg(nRF24_RX_PW_PIPE[pipe]);

		// Read a payload from the RX FIFO
		if (*length) {
			nRF24_ReadMBReg(nRF24_CMD_R_RX_PAYLOAD, pBuf, *length);
		}

		return ((nRF24_RXResult)pipe);
	}

	// The RX FIFO is empty
	*length = 0;

	return nRF24_RX_EMPTY;
}

/**
  * @brief
  * @param
  * @retval
  */
void nRF24_SetAddrWidth(unsigned char addr_width) {
	// Set of address widths
// input:
//   addr_width - RX/TX address field width, value from 3 to 5
// note: this setting is common for all pipes
	nRF24_WriteReg(nRF24_REG_SETUP_AW, addr_width - 2);
}

/**
  * @brief
  * @param
  * @retval
  */
void nRF24_SetRFChannel(unsigned char channel) {
	// Set frequency channel
// input:
//   channel - radio frequency channel, value from 0 to 127
// note: frequency will be (2400 + channel)MHz
// note: PLOS_CNT[7:4] bits of the OBSERVER_TX register will be reset
	nRF24_WriteReg(nRF24_REG_RF_CH, channel);
}

/**
  * @brief
  * @param
  * @retval
  */

nRF24_Status nRF24_IRQ_Handler(void)
{
	nRF24_Status data;

	data.eStatus = nRF_NONE;
	data.tPacket.AllPacket = nRF24_GetStatus();

	if(data.tPacket.Bits.sRX_DR == 1)
	{
		data.eStatus = RX_DataReady;
	}
	return (data);
}

/*****************************************************************************/
/* END OF FILE 																															 */
/*****************************************************************************/
