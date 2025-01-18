#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_system.h" // FLASH
#include "stm32f3xx_ll_utils.h" 
#include "stm32f3xx_ll_bus.h"

#include "system_init.h"


//-----------------------------------------------------------------------------
// PEREDELAT
//-----------------------------------------------------------------------------

void system_clock_config(void)
{
  //--- flash latency ---//
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {  }
  
  //--- HSE ---//
  LL_RCC_HSE_Enable();
  //--- Wait till HSE is ready ---//
  while(LL_RCC_HSE_IsReady() != 1)
  {  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_2, LL_RCC_PLL_MUL_16);
  LL_RCC_PLL_Enable();

  //--- Wait till PLL is ready ---//
  while(LL_RCC_PLL_IsReady() != 1)
  {  }
  
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

  //--- Wait till System clock is ready ---//
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {  }
  
  //--- timer delay in utils.h ---//
  LL_Init1msTick(64000000);
  LL_SetSystemCoreClock(64000000);
  //-- TIM1 ???--//
  LL_RCC_SetTIMClockSource(LL_RCC_TIM1_CLKSOURCE_PCLK2);
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------

void nvic_priority_config(void)
{
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),PEND_SV_PRIORITY, 0));
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),SYS_TICK_PRIORITY, 0));
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------

void system_delay(uint32_t delay)
{
  LL_mDelay(delay);
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


//-----------------------------------------------------------------------------
// PLL ENABLE
//-----------------------------------------------------------------------------


/*
void SystemClock_Config(void)
{
  

  /// HSI configuration and activation 
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /// Main PLL configuration and activation 
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_Enable();
  LL_RCC_PLL_EnableDomain_SYS();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  /// Set AHB prescaler
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /// Sysclk activation on the main PLL 
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /// Set APB1 prescaler
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(64000000);
  /// Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) 
  LL_SetSystemCoreClock(64000000);
}
*/

