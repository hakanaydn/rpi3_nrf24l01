#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <sys/types.h>
#include <pthread.h>
#include <sys/time.h>

#include "BCM/bcm2835.h"
#include "nRF_Drivers/nrf24l01.h"


// Blinks on RPi pin GPIO 11
#define PIN RPI_GPIO_P1_11

void *pReadNRFFunc(void *parameters)
{
	
  nRF24_Status data;
  nRF24_RXResult pipe;
  clock_t t;

  unsigned int xData=0;
  unsigned char nRF24_payload[32];
  unsigned char payload_length;

  bool lMeasureTimeFlag = false;
  double time_taken = 0;
  while (1)
  {
      if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY)
      {
        if(lMeasureTimeFlag == false)
        {
	  t = clock();
	  lMeasureTimeFlag  = true;
        }

        nRF24_ClearIRQFlags();

        pipe = nRF24_ReadPayload(nRF24_payload, &payload_length);

        xData++;

        if(nRF24_payload[0] == 128)
        {
           t = clock() - t;

            printf("PAYLOAD:> ");
            for(uint8_t lcnt = 0 ; lcnt < payload_length; lcnt++)
            {
              printf("%d ",nRF24_payload[lcnt] );
            }
            printf("<\n" );


 	   time_taken = ((double)t)/CLOCKS_PER_SEC; // in seconds

           printf("Time taken by program is :  %f\n",time_taken );
           time_taken = 0;
           lMeasureTimeFlag  = false;
        } 
      }
  }

  while (1)
  {
    data = nRF24_IRQ_Handler();

    if(data.eStatus == RX_DataReady)
    {
      if(lMeasureTimeFlag == false)
      {
          t = clock();
          lMeasureTimeFlag  = true;
      }

      xData++;

      pipe = nRF24_ReadPayload(nRF24_payload, &payload_length);
      nRF24_ClearIRQFlags();

      if(nRF24_payload[0] == 128)
      {
           t = clock() - t;
          printf("PAYLOAD:> ");
      		for(uint8_t lcnt = 0 ; lcnt < payload_length; lcnt++)
      		{
            printf("%d ",nRF24_payload[lcnt] );
      		}
          printf("<\n" );

		printf("%d %d\n",pipe,xData);
          time_taken = ((double)t)/CLOCKS_PER_SEC; // in seconds

          printf("Time taken by program is :  %f\n",time_taken );
          xData = 0;
          time_taken = 0;
          lMeasureTimeFlag  = false;
      }


    }
  }
}

void *pLedFunc(void *parameters)
{
  // Set the pin to be an output
  bcm2835_gpio_fsel(PIN, BCM2835_GPIO_FSEL_OUTP);

  while (1)
  {
  	// Turn it on
  	bcm2835_gpio_write(PIN, HIGH);
  	// wait a bit
  	delay(100);
  	// turn it off
  	bcm2835_gpio_write(PIN, LOW);
  	// wait a bit
  	delay(100);
  }
}

void vInit(void)
{
  nRF24_Init();

  if(nRF24_Check() == 1)
  {
      printf("--- CHECK OK --- \n");
  }
  else
  {
      printf("--- CHECK ERROR --- \n");
  }

// This is simple receiver with one RX pipe:
	//   - pipe#1 address: '0xE7 0x1C 0xE3'
	//   - payload: 5 bytes
	//   - RF channel: 115 (2515MHz)
	//   - data rate: 250kbps (minimum possible, to increase reception reliability)
	//   - CRC scheme: 2 byte

    // The transmitter sends a 5-byte packets to the address '0xE7 0x1C 0xE3' without Auto-ACK (ShockBurst disabled)

    // Disable ShockBurst for all RX pipes
    nRF24_DisableAA(0xFF);

    // Set RF channel
    nRF24_SetRFChannel(115);

    // Set data rate
    nRF24_SetDataRate(nRF24_DR_2Mbps);

    // Set CRC scheme
    nRF24_SetCRCScheme(nRF24_CRC_2byte);

    // Set address width, its common for all pipes (RX and TX)
    nRF24_SetAddrWidth(3);

    // Configure RX PIPE#1
    static const uint8_t nRF24_ADDR[] = { 0xE7, 0x1C, 0xE3 };
    nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR); // program address for RX pipe #1
    nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_OFF, 32); // Auto-ACK: disabled, payload length: 5 bytes

    // Set operational mode (PRX == receiver)
    nRF24_SetOperationalMode(nRF24_MODE_RX);

    // Wake the transceiver
    nRF24_SetPowerMode(nRF24_PWR_UP);

    // Put the transceiver to the RX mode
    nRF24_CE_H();
}

int main(int argc, char const *argv[]) {

  pthread_t readNrf_t, ledTest_t;

  printf("--- CREATE TREADS --- \n");

  vInit();

  pthread_create(&readNrf_t,NULL,&pReadNRFFunc,NULL);
  pthread_create(&ledTest_t,NULL,&pLedFunc,NULL);

  pthread_join(readNrf_t, NULL);
  pthread_join(ledTest_t, NULL);

  return 0;
}
