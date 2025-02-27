#ifndef _APP_ETHERNET_H_
#define _APP_ETHERNET_H_

#define NEW_INIT


//---- functions -------

void ETH_fill_buffer(void);
void ETH_pins_init(void);

#ifdef NEW_INIT
void ETH_init(void);
#endif

//-- new functions ---
uint8_t DefinePackage(uint8_t * buf_input);
void SendARPResponse(uint8_t * buf_input);
void Set_MAC_Filter(void);


void ETH_transmit_pack(void);
void ETH_receive_pack(void);

void set_rmii(void);

void RunTx(void);
void Eth_init_old(void);



#endif /* _APP_ETHERNET_H_ */



