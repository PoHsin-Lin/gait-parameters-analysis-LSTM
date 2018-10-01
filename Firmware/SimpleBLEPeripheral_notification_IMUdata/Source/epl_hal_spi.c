#include <ioCC2541.h>
#include "epl_hal_spi.h"

static uint8 SPI_MODE;

/** \brief	Write one byte to SPI interface
*
* Write one byte to SPI interface
*
* \param[in]       write
*     Value to write
*/
void spiWriteByte(uint8 write)
{
  U1CSR &= ~0x02;                 /* Clear TX_BYTE */
  U1DBUF = write;
  while (!(U1CSR & 0x02));        /* Wait for TX_BYTE to be set */
}

/** \brief	Read one byte from SPI interface
*
* Read one byte from SPI interface
*
* \param[in]       read
*     Read out value
* \param[in]       write
*     Value to write
*/
void spiReadByte(int8 *read, uint8 write)
{
  U1CSR &= ~0x02;                 /* Clear TX_BYTE */
  U1DBUF = write;
  
  if( SPI_MODE == SPI_MASTER )
  {
    while (!(U1CSR & 0x02));        /* Master Mdoe Wait for TX_BYTE to be clear */
  }
  else
  {
    while(!(U1CSR&0x04));           /* Slave Mdoe Wait for RX_BYTE to be clear */
  }
  *read = U1DBUF;
}

/**
*
*       @brief Init SPI interface
*
*/
void spiInit(uint8 MODE){
  
  SPI_MODE = MODE; 
  //*** Setup the SPI interface ***
  switch( SPI_MODE )
  {
  case SPI_MASTER:      
    U1CSR &= ~0xA0;   // SPI Master Mode
    
    /* Negative clock polarity, Phase: data out on CPOL -> CPOL-inv
    *                                 data in on CPOL-inv -> CPOL 
    * MSB first*/
    U1GCR = 0x20;
    /* SCK frequency = 1MHz */
    U1GCR |= 0x0F;
    U1BAUD = 0x00;
    
    /**** Setup USART 1 SPI at alternate location 2 ***/
    /* USART 1 at alternate location 2 */
    PERCFG |= 0x02;
    /* Peripheral function on SCK, MISO and MOSI (P1_5-7) */
    P1SEL |= 0xE0;
	
	
    /* Set P1_4 to a GPIO output for SSN */
    P1SEL &= 0xEF; /* 1110 1111 , 0 for GPIO */
    P1DIR |=0x10;  /* 0001 0000 , 1 for Output */
	
    CS = 0;
    
    break;
  case SPI_SLAVE:      
    U1CSR &= ~0x80;   /* SPI SLAVE Mode 0111 1111 */
    U1CSR |= 0x20;
    
    /* Negative clock polarity, Phase: data out on CPOL -> CPOL-inv
    *                                 data in on CPOL-inv -> CPOL
    * MSB first*/
    U1GCR = 0x20;

    
    /**** Setup USART 1 SPI at alternate location 2 ***/
    /* USART 1 at alternate location 2 */
    PERCFG |= 0x02;
    /* Peripheral function on SSN, SCK, MISO and MOSI (P1_4-7) */
    P1SEL |= 0xF0;
    
    /*****Slave Rx complete Interrupt enable setting*****/
    URX1IF = 0x00;
    URX1IE = 0x01;
    U1DBUF = 0x18;
    EA     = 1;       // enable all interrupt
    
    break;
  default :
    return ;
  };
  
}

/***************************************************************************************************
*                                    INTERRUPT SERVICE ROUTINE
***************************************************************************************************/
/**
* @brief   SPI 1 slave RX sevice route
*
* @param	None
*
* @return 	None
*/
// HAL_ISR_FUNCTION( halSPI1RxIsr, URX1_VECTOR )
// {
  // static uint8 read = 0x00;
  
  // spiReadByte( &read[read_idx], 0xF0);
 
// }

