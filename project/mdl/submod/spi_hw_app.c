#include <stdint.h>

#ifdef STM32G0
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_spi.h"  
#include "stm32g0xx_ll_dma.h"
#endif /* STM32G0 */

#ifdef STM32F4
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_spi.h"  
#include "stm32f4xx_ll_dma.h"
#endif /* STM32F4 */

#ifdef STM32L4
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_spi.h"  
#include "stm32l4xx_ll_dma.h"
#endif /* STM32L4 */

#ifdef STM32F7
#include "stm32f7xx_ll_bus.h"
#include "stm32f7xx_ll_rcc.h"
#include "stm32f7xx_ll_gpio.h"
#include "stm32f7xx_ll_spi.h"  
#include "stm32f7xx_ll_dma.h"
#endif /* STM32F7 */


#include "spi_hw_app.h"

#include "system_init.h"
//-----------------------------------------------------------------------------

static void spi_pins_init(void);
static void spi_port_init(void);
static void spi_dma_init(void);
static void spi_dma_interrupts_enable(void);
static void spi_delay(uint32_t period);

void set_SPI_addr (void);

void set_SPI_buf_len (uint32_t data_len);

void set_SPI_buf_addr (uint32_t * addr);



#define CMD_LED_ON  0x0A01
#define CMD_LED_OFF 0x0A02
#define CMD_TRANSFER_DATA ((uint16_t)(0x0103))

uint16_t spi_tx_buffer[1] = {0x0103};
uint16_t spi_rx_buffer[1];

uint16_t Buf_SPI[CCD_SZ] = {0};

uint8_t receive_spec_resp = 0;

//-----------------------------------------------------------------------------

void spi_init_all(void)
{
  spi_pins_init();
  spi_port_init();
  spi_dma_init();
  spi_dma_interrupts_enable();
}

//-----------------------------------------------------------------------------

static void spi_pins_init(void)
{
  /*
  PE2	SPI4_SCK	n/a	Alternate Function Push Pull	No pull-up and no pull-down	Very High	n/a		false
  PE5	SPI4_MISO	n/a	Alternate Function Push Pull	No pull-up and no pull-down	Very High	n/a		false
  PE6	SPI4_MOSI	n/a	Alternate Function Push Pull	No pull-up and no pull-down	Very High	n/a		false
  */
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
  
  //--- SPI1 GPIO Configuration ---//
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  //---- PE2 --> SPI4_SCK ----//
  //---- PE5 --> SPI4_MISO ----//
  //---- PE6 --> SPI4_MOSI  ----//
  
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_5|LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;             // AF5 P.95 DT
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct); 
 
}

//------------------------------------------------------------------------------
// Spi2 slave mode RX Only
//------------------------------------------------------------------------------

static void spi_port_init(void)
{
  //--- SPI 4 INIT ---//
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI4);
  
  //--- SPI4  configuration ---//
  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;        // Change !!! LL_SPI_SIMPLEX_RX;         // SPIx->CR1 -- RX Only
  SPI_InitStruct.Mode =LL_SPI_MODE_MASTER;                      // Change !!! LL_SPI_MODE_SLAVE; // Slave no (MSTR | SSI)
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;            // Data - 16 bit (CR2 DS)
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;                         // SS not used
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16;      //  8 ??? Mhz               LL_SPI_BAUDRATEPRESCALER_DIV4;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;// NO CRC
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI4, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI4, LL_SPI_PROTOCOL_MOTOROLA);                   // CR2, SPI_CR2_FRF
  LL_SPI_EnableNSSPulseMgt(SPI4);                               // CR2, SPI_CR2_NSSP (Only in Master Mode)
  
  //--- SPI Interrupts Enable ---//
  //LL_SPI_SetRxFIFOThreshold(SPI4, LL_SPI_RX_FIFO_TH_HALF);              // !!! New Change
  //LL_SPI_EnableIT_TXE(SPI4);                                            // !!! New Change
  //NVIC_SetPriority(SPI4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),4, 0)); 
  //NVIC_EnableIRQ(SPI4_IRQn);
  
  //--- DMA RX Enable ---//
  //LL_SPI_EnableDMAReq_TX(SPI4);
  //LL_SPI_EnableDMAReq_RX(SPI4);
  //--- SPI 4 Enable ---//
  LL_SPI_Enable(SPI4);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------

static void spi_dma_init(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

  //LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_0, (uint32_t) Buf_ADC);
  //LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, (uint32_t) (CCD - 2));
    
  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);   // From SPI to Buf
  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_0, LL_DMA_PRIORITY_MEDIUM);
  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);                                 // Circular LL_DMA_MODE_CIRCULAR 
  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_HALFWORD);
  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_HALFWORD);
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_0, LL_DMA_CHANNEL_4);
  
  
  //// DMA Tramsmit 
  
  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_1, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);   // From Buf  to SPI 
  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_1, LL_DMA_PRIORITY_MEDIUM);
  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_1, LL_DMA_MODE_NORMAL);                                 // Circular LL_DMA_MODE_CIRCULAR 
  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_1, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_1, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_HALFWORD);
  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_1, LL_DMA_MDATAALIGN_HALFWORD);
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_1, LL_DMA_CHANNEL_4);
  
  
  //DMA1_Channel1->CCR = 0;
  
  // | DMA_CCR_TEIE | DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_PSIZE_0  | DMA_CCR_MSIZE_0);
  //DMA1_Channel1->CCR |= (DMA_CCR_TCIE | DMA_CCR_TEIE | DMA_CCR_EN);  
  
  //DMAMUX
  // SPI2_RX -- DMAMUX input = 18
  //LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_0, LL_DMAMUX_REQ_SPI2_RX);
  
  
}

//------------------------------------------------------------------------------

static void spi_dma_interrupts_enable(void)
{
  DMA2_Stream0->CR |= DMA_SxCR_TCIE;    // Transfer Complete interrupt enable 
  NVIC_SetPriority(DMA2_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),3, 0)); 
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

//------------------------------------------------------------------------------
// DMA addreses init
//------------------------------------------------------------------------------

inline void set_SPI_addr (void)
{
  uint32_t spi_addr;
  spi_addr = LL_SPI_DMA_GetRegAddr(SPI4);
  LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_0, spi_addr);
}

//------------------------------------------------------------------------------

inline void set_SPI_buf_len (uint32_t data_len)
{
  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, data_len);
}

//------------------------------------------------------------------------------

inline void set_SPI_buf_addr (uint32_t * addr)
{
  LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_0, (uint32_t) addr);
}

//------------------------------------------------------------------------------
// DMA Buffer RX

void spi_dma_buf (uint32_t * addr, uint32_t data_len)
{
  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);
  
  set_SPI_addr ();
  set_SPI_buf_len (data_len);
  set_SPI_buf_addr (addr);
  
  LL_SPI_EnableDMAReq_RX(SPI4);
  
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
}

//------------------------------------------------------------------------------
// DMA Buffer TX

void spi_dma_TX (uint32_t * addr, uint32_t data_len)
{
  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_1);
  
  set_SPI_addr ();
  set_SPI_buf_len (data_len);
  set_SPI_buf_addr (addr);
  
  LL_SPI_EnableDMAReq_TX(SPI4);
  
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);
}


//------------------------------------------------------------------------------

static void spi_delay(uint32_t period)
{
  for(uint32_t a = 0; a < period; a++)
  {
    for(uint16_t b = 0; b < 128; b++) { __NOP(); }
  }
}

//------------------------------------------------------------------------------


void get_spectrum_response(void)
{
 
  SPI4->CR1 &= ~LL_SPI_SIMPLEX_RX;
  LL_SPI_Enable(SPI4);
  
  //-- send resp to STM32f302 by SPI --//
  //spi_dma_TX ((uint32_t *)&spi_tx_buffer[0], 1);
  
  while (!LL_SPI_IsActiveFlag_TXE(SPI4));  // ???? ?????????? SPI
  LL_SPI_TransmitData16(SPI4, CMD_TRANSFER_DATA);
  //system_delay(1);
  spi_delay(100);
 
  //LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_1);
  //LL_SPI_DisableDMAReq_TX(SPI4);

  //--- OFF SPI ---///
  LL_SPI_Disable(SPI4);
  //--- Receive only SPI--//
  SPI4->CR1 |= LL_SPI_SIMPLEX_RX;
  //--- Enable SPI ---///
  LL_SPI_Enable(SPI4);
  //--- Start DMA Receive ONLY ---///
  spi_dma_buf ((uint32_t *)&Buf_SPI[0], CCD_SZ);
}

//------------------------------------------------------------------------------
// INTERRUPTS
//------------------------------------------------------------------------------

void DMA2_Stream0_IRQHandler(void) // in spi.c, in config
{
  //---- IF transfer complete ---//
  if(LL_DMA_IsActiveFlag_TC0(DMA2))     // DMA2 Stream0
  {
    LL_DMA_ClearFlag_TC0(DMA2);
    LL_DMA_ClearFlag_HT0(DMA2);
    LL_DMA_ClearFlag_TE0(DMA2);
    //spi_handler(); // Extern function
  }
  else
  {
    // clear flags
    LL_DMA_ClearFlag_TC0(DMA2);
    LL_DMA_ClearFlag_HT0(DMA2);
    LL_DMA_ClearFlag_TE0(DMA2);  // Transfer error
  }   
  //spi_handler(); // Extern function --- if transfer complete
  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);
    //--- OFF SPI ---///
  LL_SPI_Disable(SPI4);
  LL_SPI_DisableDMAReq_RX(SPI4);
  
  send_spectrum_buffer(); // Extern function --- if transfer complete
 

  //--- Full Duplex SPI--//
  //SPI4->CR1 &= ~LL_SPI_SIMPLEX_RX;
  
  //--- Enable SPI ---///
  //LL_SPI_Enable(SPI4);
  
 
}


//-----------------------------------------------------------------------------

void DMA2_Stream1_IRQHandler(void) 
{
    if(LL_DMA_IsActiveFlag_TC1(DMA2)) 
    {
        LL_DMA_ClearFlag_TC1(DMA2);
        LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_1);
        LL_SPI_DisableDMAReq_TX(SPI4);

        // Disable SPI
        LL_SPI_Disable(SPI4); 
    }
}


//-----------------------------------------------------------------------------

/*
void SPI4_IRQHandler(void)
{       //--- if TX is complete ---//
  if (LL_SPI_IsActiveFlag_TXE(SPI4)) 
  {
    LL_SPI_ReceiveData16(SPI4);
    if (receive_spec_resp == 1)
    {
      receive_spec_resp = 0;
      //-- enable DMA receive --//
      spi_dma_buf ((uint32_t *)&Buf_SPI[0], CCD_SZ);
    }
  }
}
*/

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------




//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------




//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------




//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
