#include <string.h>


#ifdef STM32F7
#include "stm32f7xx_ll_bus.h"
#include "stm32f7xx_ll_gpio.h"
#endif /* STM32F7 */

#include "app_ethernet.h"
#include "stm32_eth.h"
#include "stm32f767xx.h"

#define PHY_ADDRESS     0x0

#define SPEC_BUF_SIZE   4

//------ DEFINES -----------------------------------

#define TDES0_OWN       ((uint32_t) (0x1 << 31))
#define TDES0_LS        ((uint32_t) (0x1 << 29))
#define TDES0_FS        ((uint32_t) (0x1 << 28))
#define TDES0_DC        ((uint32_t) (0x1 << 27))
#define TDES0_TER       ((uint32_t) (0x1 << 21))
#define TDES0_TCH       ((uint32_t) (0x1 << 20))
#define TDES0_IHE       ((uint32_t) (0x1 << 16))
#define TDES0_ES        ((uint32_t) (0x1 << 15))
#define TDES0_FF        ((uint32_t) (0x1 << 13))
#define TDES0_IPE       ((uint32_t) (0x1 << 12))
#define TDES0_LCA       ((uint32_t) (0x1 << 11))
#define TDES0_NC        ((uint32_t) (0x1 << 10))
#define TDES0_UF        ((uint32_t) (0x1 << 1))

//--------------------------------------------------

uint16_t phy_status = 0;

uint32_t Value = 0;



#pragma data_alignment=4
ETH_DMADESCTypeDef  RxDscrTab[ETH_RXBUFNB];/* Ethernet Rx MA Descriptor */
#pragma data_alignment=4
ETH_DMADESCTypeDef  TxDscrTab[ETH_TXBUFNB];/* Ethernet Tx DMA Descriptor */
#pragma data_alignment=4
uint8_t Rx_Buf[ETH_RXBUFNB][ETH_RX_BUF_SIZE]; /* Ethernet Receive Buffer */
#pragma data_alignment=4
uint8_t Tx_Buf[ETH_TXBUFNB][ETH_TX_BUF_SIZE]; 

#pragma data_alignment=4
uint8_t Spectrum_Buf[SPEC_BUF_SIZE][ETH_TX_BUF_SIZE];


//------------------------------------------------------------------------------
//---- static  functions --------
//------------------------------------------------------------------------------

static void CopyBuf(uint8_t * Buf_A, uint8_t * Buf_B, uint32_t value);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

uint8_t ak = 0;

//--------------------------------------------------

//--- list of descriptors
volatile uint32_t RXDL[4] __attribute__((aligned(4)));
__attribute__((section(".sram1"), aligned(4))) volatile  uint32_t TXDL[8];

//--- Buffer
uint8_t RX_BUF[1532];   //1500 + 6*6(MAC) + 2(ethertype) + 4B(CRC) = 1518 B
uint8_t TX_BUF[214] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,      // MACd 
                       0x00, 0x80, 0xE1, 0x0A, 0x11, 0x07,      // MACs
                       0x08, 0x00,                              // IP
                       0x45, 0x00, 0x00, 0xC8,                  // IPv4 (1B), IHL (1B), Len (2B) 200 B
                       0x00, 0x01, 0x00, 0x00,                  // ID pack, flags (3b), offset(12b)
                       0x09, 0x02, 0x00, 0x00,                  // TTL(1B), Prot.(1B), Checksum (2B)
                       192,  168,  100,   157,                  // IP source
                       192,  168,  100,   1,                    // IP dest
                       0x00, 0x00, 0x00, 0x00,                  // Data
                       0x00, 0x00, 0x00, 0x00,
                       0x01, 0x02, 0x03, 0x04,
                       0x05, 0x06, 0x06, 0x07,
                       0x08, 0x09, 0x0A, 0x0B,
                       0x0C, 0x0D, 0x0E, 0x0F,
                       0x10, 0x11, 0x12, 0x13,
                       0x14, 0x15, 0x16, 0x17,
                       0x18, 0x19, 0x1A, 0x1B,
                       0x1C, 0x1D, 0x1E, 0x1F
                      }; 

__attribute__((aligned(4))) uint8_t TX_ARP[42] = {
                       0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,      // MACd
                       0x00, 0x80, 0xE1, 0x0A, 0x11, 0x07,      // MACs
                       0x08, 0x06,                              // EtherType
                       0x00, 0x01, 0x08, 0x00,                  // HTYPE, PTYPE
                       0x06, 0x04,                              // HLEN, PLEN
                       0x00, 0x01,                              // OPER
                       0x00, 0x80, 0xE1, 0x0A, 0x11, 0x07,      // MACs
                       192,  168,  100,   157,                  // IPs   28
                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00,      // MACd  32
                       192,  168,  100,   1,                    // IPd   38
                      };
                   

uint8_t MAC_PC[6] = {0};  // MAC-PC
uint8_t IP_PC[4] = {0};   // IP-PC

uint8_t MAC_STM[6] = {0x00, 0x80, 0xE1, 0x0A, 0x11, 0x07}; 
uint8_t IP_STM[4] = {192,  168,  100,   157};   

uint8_t TX_ARP_Resp[42] = {0};


//---------------------------------------------------------------------------- 
//---------------------------------------------------------------------------- 
//----------------------------------------------------------------------------
 
void ETH_fill_buffer(void)
{
  uint32_t cnt = 0;
  uint32_t ext_cnt = 0;
  
  for(cnt = 0; cnt<42; cnt++)
  {
    Tx_Buf[0][cnt] = TX_ARP[cnt];
  }
  
  for(cnt = 0; cnt<214; cnt++)
  {
    Tx_Buf[1][cnt] = TX_BUF[cnt];
  }
  
  //---- Fill Spectrum Buffer ----
  for(ext_cnt = 0; ext_cnt < SPEC_BUF_SIZE; ext_cnt++)
    for(cnt = 0; cnt < ETH_TX_BUF_SIZE; cnt++)
    {
      if (cnt < 40)
        Spectrum_Buf[ext_cnt][cnt] = TX_BUF[cnt];
      else if (cnt < 530)
        Spectrum_Buf[ext_cnt][cnt] = (uint8_t)( ((uint8_t)(cnt - 39)/8) * (ext_cnt + 1) );
    }
}


//---------------------------------------------------------------------------- 
//---------------------------------------------------------------------------- 
//----------------------------------------------------------------------------
//----- N E W  F U N C T I O N S --------------------------------------------- 
//----------------------------------------------------------------------------

/*
static void CopyBuf(uint8_t * Buf_A, const uint8_t * Buf_B, uint32_t init_val, uint32_t max_val)
{
  if (init_val <= max_val)
    for(uint32_t cnt = init_val; cnt < max_val; cnt++)
    {
        Buf_A[cnt] = Buf_B[cnt]; 
    }
}
*/

static void CopyBuf(uint8_t * Buf_A, uint8_t * Buf_B, uint32_t value)
{
  for(uint32_t cnt = 0; cnt < value; cnt++)
  {
    *(Buf_A + cnt) = *(Buf_B + cnt);
  }
}

//---------------------------------------------------------------------------- 
//---------------------------------------------------------------------------- 
//----------------------------------------------------------------------------


uint8_t DefinePackage(uint8_t * buf_input)
{
  if ( (buf_input[12] == 0x08) &&
       (buf_input[13] == 0x06) &&
       (buf_input[21] == 0x01) )        // ARP
  {
    if ( buf_input[38] == IP_STM[0] &&
         buf_input[39] == IP_STM[1] &&
         buf_input[40] == IP_STM[2] &&
         buf_input[41] == IP_STM[3] )
      return 1;
  }  
  return 0;
}

//---------------------------------------------------------------------------- 
//---------------------------------------------------------------------------- 
//----------------------------------------------------------------------------

void SendARPResponse(uint8_t * buf_input) 
{
    CopyBuf(TX_ARP_Resp, buf_input, 42); // Copy Buf to ARP Buf
       
    CopyBuf(MAC_PC, TX_ARP_Resp + 6, 6);        // Store MAC Destination 
    CopyBuf(TX_ARP_Resp, MAC_PC, 6);            // Replace MAC Destination 1
    CopyBuf(TX_ARP_Resp + 32, MAC_PC, 6);       // Replace MAC Destination 2
    CopyBuf(TX_ARP_Resp + 6, MAC_STM, 6);       // Replace STM32 MAC 
    
    CopyBuf(IP_PC, TX_ARP_Resp + 28, 4);        // Store IP Destination
    CopyBuf(TX_ARP_Resp + 38, IP_PC, 4);        // Replace IP Destination
    CopyBuf(TX_ARP_Resp + 28, IP_STM, 4);       // Replace IP STM32
    
    // Change ARP packet type (response)
    TX_ARP_Resp[21] = 0x02; //  ARP response (OPER = 2)
    
    // Sending an ARP reply
    ETH_DMATransmissionCmd(DISABLE);
    ETH_HandleTxPkt(&TX_ARP_Resp[0], 42);
    ETH_DMATransmissionCmd(ENABLE);
}

//---------------------------------------------------------------------------- 
//---------------------------------------------------------------------------- 
//----------------------------------------------------------------------------

void Set_MAC_Filter(void) 
{
    ETH->MACA1HR = (MAC_PC[0] << 8) | MAC_PC[1];  // The upper part of the MAC address
    ETH->MACA1LR = (MAC_PC[2] << 24) | (MAC_PC[3] << 16) | (MAC_PC[4] << 8) | MAC_PC[5]; // Lower part of MAC address
    
    // Filtering settings (only packets with MAC_PC and broadcast)
    ETH->MACFFR &= ~(ETH_MACFFR_PM);  // Disable receiving all packets (Promiscuous mode)
    ETH->MACFFR |= ETH_MACFFR_HPF | ETH_MACFFR_BFD;  // Enable filtering by MAC and broadcast packets
}



//---------------------------------------------------------------------------- 
//---------------------------------------------------------------------------- 
//----------------------------------------------------------------------------


//---------------------------------------------------------------------------- 
//---------------------------------------------------------------------------- 
//----------------------------------------------------------------------------





//// new prog
#ifdef NEW_INIT

void ETH_init(void)
{       
  //--- ETH deinit
  //RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_ETH_MAC, ENABLE);
  //RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_ETH_MAC, DISABLE);
  RCC->AHB1RSTR|=RCC_AHB1RSTR_ETHMACRST;
  RCC->AHB1RSTR&=~RCC_AHB1RSTR_ETHMACRST; 
  
  //---- Ethernet RCC -----
  RCC->AHB1ENR|=RCC_AHB1ENR_ETHMACRXEN|RCC_AHB1ENR_ETHMACTXEN|RCC_AHB1ENR_ETHMACEN;
  
  //--- software reset DMA ----
  ETH->DMABMR |= ETH_DMABMR_SR;
  
  //--- init parameters ETH and DMA
  ETH_InitTypeDef ETH_InitStructure;
  ETH_StructInit(&ETH_InitStructure);
  
  ETH_InitStructure.ETH_AutoNegotiation = ETH_AutoNegotiation_Enable;
  ETH_InitStructure.ETH_Speed = ETH_Speed_100M;
  ETH_InitStructure.ETH_LoopbackMode = ETH_LoopbackMode_Disable;
  ETH_InitStructure.ETH_Mode = ETH_Mode_FullDuplex;
  ETH_InitStructure.ETH_RetryTransmission = ETH_RetryTransmission_Disable;
  ETH_InitStructure.ETH_AutomaticPadCRCStrip = ETH_AutomaticPadCRCStrip_Disable;

  ETH_InitStructure.ETH_ReceiveAll = ETH_ReceiveAll_Enable;
  ETH_InitStructure.ETH_BroadcastFramesReception = ETH_BroadcastFramesReception_Enable;
  ETH_InitStructure.ETH_PromiscuousMode = ETH_PromiscuousMode_Disable;
  ETH_InitStructure.ETH_MulticastFramesFilter = ETH_MulticastFramesFilter_Perfect;
  ETH_InitStructure.ETH_UnicastFramesFilter = ETH_UnicastFramesFilter_Perfect;
  
  //-- NEW --
  ETH->MACFFR |= ETH_MACFFR_RA;  // Receive all
  //ETH->DMAOMR |= ETH_DMAOMR_RSF | ETH_DMAOMR_TSF; // store and forward
  //check this func (PHY)
  Value = ETH_Init(&ETH_InitStructure, PHY_ADDRESS);
  set_rmii();
  ETH->MACCR = (ETH_MACCR_FES | ETH_MACCR_DM | ETH_MACCR_RD | ETH_MACCR_APCS); // RM - P.1821

  
  //init DMA descriptors
  ETH_DMATxDescChainInit(TxDscrTab, &Tx_Buf[0][0], ETH_TXBUFNB);
  ETH_DMARxDescChainInit(RxDscrTab, &Rx_Buf[0][0], ETH_RXBUFNB);
  
  TxDscrTab[0].ControlBufferSize = 42;
  TxDscrTab[1].ControlBufferSize = 214;
   
  //--- ETH Enable
  ETH_Start();
  
  ETH_receive_pack();
  /*
  ETH->MACCR |= ETH_MACCR_TE;
  ETH->DMAOMR |= ETH_DMAOMR_FTF;
  ETH->MACCR |= ETH_MACCR_RE; 
  ETH->DMAOMR |= ETH_DMAOMR_ST;
  ETH->DMAOMR |= ETH_DMAOMR_SR;
  */

}
#endif





//---------------------------------------------------------------------------- 
//---------------------------------------------------------------------------- 
//---------------------------------------------------------------------------- 

void ETH_receive_pack(void)
{
  ETH_DMAReceptionCmd(DISABLE);
  ETH_DMAReceptionCmd(ENABLE); // SR bit set
}

void ETH_transmit_pack(void)
{
  if(ak == 0)
  {
    ETH_DMATransmissionCmd(DISABLE);
    ETH_HandleTxPkt(&Tx_Buf[0][0], 42);
    ETH_DMATransmissionCmd(ENABLE);
    ak ++;
  }
  else
  {
    ETH_DMATransmissionCmd(DISABLE);
    ETH_HandleTxPkt(&Tx_Buf[1][0], 214);
    ETH_DMATransmissionCmd(ENABLE);
    ak = 0;
  }
}


//---------------------------------------------------------------------------- 
//---------------------------------------------------------------------------- 
//----------------------------------------------------------------------------
  
void ETH_pins_init(void)
{
    //uint32_t w;
  //---- Enable RMII_PHY ---------
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  SYSCFG->PMC |= SYSCFG_PMC_MII_RMII_SEL;
  
  
  for (uint32_t i=0;i<3000;i++);
  
  //----------------------------------------------------------------------------
  //---- Ethernet pins 
  
  LL_AHB1_GRP1_EnableClock( LL_AHB1_GRP1_PERIPH_GPIOA );
  LL_AHB1_GRP1_EnableClock( LL_AHB1_GRP1_PERIPH_GPIOB );
  LL_AHB1_GRP1_EnableClock( LL_AHB1_GRP1_PERIPH_GPIOC );
  LL_AHB1_GRP1_EnableClock( LL_AHB1_GRP1_PERIPH_GPIOG );
  
  /**
  PA1	ETH_REF_CLK	n/a	Alternate Function Push Pull	No pull-up and no pull-down	Very High	n/a	RMII_REF_CLK [LAN8742A-CZ-TR_REFCLK0]	true
  PA2*  ETH_MDIO	n/a	Alternate Function Push Pull	No pull-up and no pull-down	Very High	n/a	RMII_MDIO [LAN8742A-CZ-TR_MDIO]	        true
  PA7	ETH_CRS_DV	n/a	Alternate Function Push Pull	No pull-up and no pull-down	Very High	n/a	RMII_CRS_DV [LAN8742A-CZ-TR_CRS_DV]	true
  
  PB13	ETH_TXD1	n/a	Alternate Function Push Pull	No pull-up and no pull-down	Very High	n/a	RMII_TXD1 [LAN8742A-CZ-TR_TXD1]	        true
  
  PC1*	ETH_MDC	        n/a	Alternate Function Push Pull	No pull-up and no pull-down	Very High	n/a	RMII_MDC [LAN8742A-CZ-TR_MDC]	        true
  PC4	ETH_RXD0	n/a	Alternate Function Push Pull	No pull-up and no pull-down	Very High	n/a	RMII_RXD0 [LAN8742A-CZ-TR_RXD0]	        true
  PC5	ETH_RXD1	n/a	Alternate Function Push Pull	No pull-up and no pull-down	Very High	n/a	RMII_RXD1 [LAN8742A-CZ-TR_RXD1]	        true
  
  PG11	ETH_TX_EN	n/a	Alternate Function Push Pull	No pull-up and no pull-down	Very High	n/a	RMII_TX_EN [LAN8742A-CZ-TR_TXEN]	true
  PG13	ETH_TXD0	n/a	Alternate Function Push Pull	No pull-up and no pull-down	Very High	n/a	RMII_TXD0 [LAN8742A-CZ-TR_TXD0]	        true
  */ 
    
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  // PORTA
  GPIO_InitStruct.Pin = (LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_7); 
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_11;        // AFIO
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);  // PORTA
  
  // PORTB
  GPIO_InitStruct.Pin = (LL_GPIO_PIN_13); 
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_11;        // AFIO
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);  // PORTB
  
  // PORTC
  GPIO_InitStruct.Pin = (LL_GPIO_PIN_1 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5); 
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_11;        // AFIO
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);  // PORTC
  
  // PORTG
  GPIO_InitStruct.Pin = (LL_GPIO_PIN_11 | LL_GPIO_PIN_13); 
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_11;        // AFIO
  LL_GPIO_Init(GPIOG, &GPIO_InitStruct);  // PORTG 
   
}


//---------------------------------------------------------------------------- 
//---------------------------------------------------------------------------- 
//---------------------------------------------------------------------------- 
//---------------------------------------------------------------------------- 
//---------------------------------------------------------------------------- 
//----------------------------------------------------------------------------
 
void RunTx(void)
{
  //------------------ none OSF MODE -------------------
  // Num of bytes - TDES1, FS in TDES0 (first descriptor), LS in TDES0 (last descriptor)
  // OWN bit set after setting up the corresponding data buffer; ST in - run state
  
  //ETH->DMABMR |= ETH_DMABMR_SR;  // Soft reset DMA
  //while (ETH->DMABMR & ETH_DMABMR_SR);  // Wait for reset to complete

  ETH->DMAOMR &= ~ETH_DMAOMR_ST;  // Stop DMA
  //ETH->DMASR |= ETH_DMASR_TBUS;  // Clear Transmit Buffer Unavailable
  ETH->DMATPDR = 0;  // Resume transmission

  //init list of descriptors and DMATDLAR register setup
  TXDL[0] &= ~TDES0_OWN; // Clear OWN bit before configuring
  TXDL[0] = TDES0_OWN | TDES0_LS | TDES0_FS | TDES0_TCH | TDES0_TER;
  TXDL[1] = 44; // Packet length
  TXDL[2] = (uint32_t)TX_ARP; // DMA buffer pointer
  TXDL[3] = (uint32_t)TXDL;   // Circular buffer
  //TXDL[3] = (uint32_t)TX_BUF;  
  
  //TXDL[3] = (uint32_t)&TXDL[4]; // Point to the next descriptor

  // Second descriptor (Dummy)
  //TXDL[4] = 0;
  //TXDL[5] = 0;  // Length = 0, so it won't send anything
  //TXDL[6] = 0;  // No buffer
  //TXDL[7] = (uint32_t)&TXDL[0]; // Circular buffer: point back to first descriptor
  
  
  //ETH->DMASR &= ~ETH_DMASR_TBUS;  // Clear Transmit Buffer Unavailable
  ETH->DMAOMR |= ETH_DMAOMR_TSF; /// ??
 
  ETH->DMATPDR = 0;
  //--- Enable DMA Tx
  ETH->DMAOMR |= ETH_DMAOMR_ST; // ETH_DMAOMR_SR;
      // Trigger Transmission Poll Demand
  
  
}

//---------------------------------------------------------------------------- 
//---------------------------------------------------------------------------- 
//---------------------------------------------------------------------------- 

void set_rmii(void)
{
   //-- Read reg 18
  ETH->MACMIIAR = ( (18 << 6) | ETH_MACMIIAR_CR_Div62 | 0x00 | 0x01);       // reg 18, MB
  while (ETH->MACMIIAR & 0x00000001) {};   //Busy
  for(uint32_t tm = 0; tm<3000; tm++) { };
  uint16_t result = (uint16_t)(0xFFFF & (ETH->MACMIIDR)); 
  
  //---- reset PHY
  ETH->MACMIIDR = 0x00008000;//soft restart data
  ETH->MACMIIAR = ( (0 << 6) | ETH_MACMIIAR_CR_Div62 | 0x02 | 0x01); // reg 0, MW, MB
  while (ETH->MACMIIAR & 0x00000001) {};   //Busy
   for(uint32_t tm = 0; tm<3000; tm++) { };
  
  //---- read reg_0 data while 
  while ((ETH->MACMIIDR&0x00008000) == 0x00008000)// reset is done?
  {//no
  ETH->MACMIIAR = ( (0 << 6) | ETH_MACMIIAR_CR_Div62 | 0x00 | 0x01); // reg 0, MB
  while (ETH->MACMIIAR&0x00000001) {};     //Busy
  for(uint32_t tm = 0; tm<3000; tm++) { };
  }
  
  //----- verify that the PHY link is established ------

  do {
      ETH->MACMIIAR = ((1 << 6) | ETH_MACMIIAR_CR_Div62 | 0x00 | 0x01); // Read PHY Status Register
      while (ETH->MACMIIAR & ETH_MACMIIAR_MB); // Wait for MII busy to clear
      for(uint32_t tm = 0; tm<500; tm++) { };
      phy_status = (uint16_t)(ETH->MACMIIDR);
  } while (!(phy_status & 0x0004));  // Bit 2: Link Status

}

//---------------------------------------------------------------------------- 
//---------------------------------------------------------------------------- 
//---------------------------------------------------------------------------- 


void Eth_init_old(void)
{ 
  //----------------------------------------------------------------------------
  
  //---- Ethernet RCC
  RCC->AHB1ENR|=RCC_AHB1ENR_ETHMACRXEN|RCC_AHB1ENR_ETHMACTXEN|RCC_AHB1ENR_ETHMACEN;
  
  //---- Reset Ethernet ???
  RCC->AHB1RSTR|=RCC_AHB1RSTR_ETHMACRST;
  RCC->AHB1RSTR&=~RCC_AHB1RSTR_ETHMACRST;    

  /*
    1. Write to ETH_DMABMR to set STM32F76xxx and STM32F77xxx bus access parameters.

    2. Write to the ETH_DMAIER register to mask unnecessary interrupt causes.

    3. The software driver creates the transmit and receive descriptor lists. 
    Then it writes to both the ETH_DMARDLAR and ETH_DMATDLAR registers, 
    providing the DMA with the start address of each list.

    4. Write to MAC Registers 1, 2, and 3 to choose the desired filtering options.

    5. Write to the MAC ETH_MACCR register to configure and enable the transmit and
    receive operating modes. The PS and DM bits are set based on the auto-negotiation
    result (read from the PHY).

    6. Write to the ETH_DMAOMR register to set bits 13 and 1 and start transmission and
    reception.

    7. The transmit and receive engines enter the running state and attempt to acquire
    descriptors from the respective descriptor lists. The receive and transmit engines then
    begin processing receive and transmit operations. The transmit and receive processes
    are independent of each other and can be started or stopped separately.
  */
  
  //---- Program reset
  ETH->DMABMR |= ETH_DMABMR_SR;
  while (ETH->DMABMR & ETH_DMABMR_SR);  // Wait for reset to complete
  
  
  //----------------------------------------------------------------------------
  //----------------------------------------------------------------------------
  //----- PHY init
  /*
  PHY ID 0x00
  1. Read Reg 18 (Data 60E0)
  2. Write Reg 0 - Data 0x8000 (Reset)
  3. Read Reg 0 while (data == 0x8000)
  4. Untill Reg0 data == 0x3000
------------------------------------
  Write OP
    1. write data in ETH_MACMIIDR
    2. Set PHY ID, RegAddr, MII_Write=1 and MII_Busy=1 in ETH_MACMIIAR
  Read OP
    1. Set PHY ID, RegAddr, MII_Write=0 and MII_Busy=1 in ETH_MACMIIAR
    2. Read ETH_MACMIIDR
  */
  
  

  set_rmii();
 
  
  //-- Fast eth 100 Mbit/s; Duplex mode;  Retry disable; Automatic Pad/CRC stripping ; ; ; ; ; ; ; 
  ETH->MACCR = (ETH_MACCR_FES | ETH_MACCR_DM | ETH_MACCR_RD | ETH_MACCR_APCS); // RM - P.1821
    
  ETH->MACFFR=ETH_MACFFR_RA|ETH_MACFFR_PAM|ETH_MACFFR_PM ;//all frames receive
  
  //---------------------------------------------------------------------------

  // DMABMR -bus access
  // DMARDLAR and ETH_ DMATDLAR
  // Write to MAC Registers 1, 2, and 3 to choose the desired filtering options
  // MAC ETH MACCR register to configure and enable the transmit and receive operating modes.
  // ETH DMAOMR register to set bits 13 and 1 and start transmission and reception
   
  
  //DMA descriptor status update before tX
  // OSF = 0, FB (fixed-length), PBL (maximum burst length) - ETH_DMABMR
  // EDFE - 0x0, PBL - 0x20, FB - 0x1, RDP - 0x02, USP - 0x1, AAB 0x1
  // FB, AAB, TTC(Threshold)
  ETH->DMABMR = ETH_DMABMR_FB | ETH_DMABMR_PBL_32Beat | ETH_DMABMR_AAB ; //  | ETH_DMABMR_RDP_32Beat | ETH_DMABMR_USP;
  ETH->DMAOMR = ETH_DMAOMR_TSF; //ETH_DMAOMR_RSF|ETH_DMAOMR_TSF;
   
  ETH->DMATDLAR = (uint32_t)(&TXDL[0]);    // Setup DMA descriptor
   
  //-- Enable Eth
  for(uint32_t tm = 0; tm<3000; tm++) { };
  ETH->MACCR =  ETH_MACCR_TE; // ETH_MACCR_RE
   
  //---------------------------------------------------------------------------
/*
  //init list of descriptors and DMARDLAR register setup
  ETH->DMARDLAR=(uint32_t)(&RXDL[0]);
  RXDL[0]= 0; // 0x80000000;//own=1
  RXDL[1]=0x000005fc;//rch=1, value RX_BUF=1532
  RXDL[2]=(uint32_t)(RX_BUF);
  RXDL[3]=(uint32_t)(RXDL);
  */
  //---------------------------------------------------------------------------
    
}