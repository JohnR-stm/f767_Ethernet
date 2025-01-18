#include "ccd_timers_hw.h"
#include "ccd_adc_hw.h"
#include "ccd_spi_hw.h"

#include "app_ccd_control.h"


void init_all_ccd(void)
{
  ccd_timers_init();
  ccd_timers_start();
  
  ccd_adc_mdl_init();
  
  ccd_spi_init();
}