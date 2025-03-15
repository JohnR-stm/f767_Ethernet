#ifndef _MDL_SPI_HW_APP_H_
#define _MDL_SPI_HW_APP_H_

void spi_init_all(void);
void spi_dma_buf (uint32_t * addr, uint32_t data_len);
void spi_dma_TX (uint32_t * addr, uint32_t data_len);
void get_spectrum_response(void);

extern void spi_handler(void);
extern void send_spectrum_buffer(void);

#define CCD_SZ                  ((uint16_t)3650)
#define CCD_SZx2                ((uint16_t) (CCD_SZ * 2))

#endif /*  _MDL_SPI_HW_APP_H_  */