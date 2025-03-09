#ifndef _MDL_SPI_HW_H_
#define _MDL_SPI_HW_H_

void spi_init_all(void);
void spi_dma_buf (uint32_t * addr, uint32_t data_len);

extern void spi_handler(void);

#endif /*  _MDL_SPI_HW_H_  */