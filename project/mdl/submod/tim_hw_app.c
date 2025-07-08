#include <stdint.h>



#ifdef STM32F3
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_gpio.h"
#include "stm32f3xx_ll_tim.h"
#endif


#ifdef STM32F7
#include "stm32f7xx_ll_bus.h"
#include "stm32f7xx_ll_gpio.h"
#include "stm32f7xx_ll_tim.h"
#endif /* STM32F7 */


#include "tim_hw_app.h"

#include "system_init.h"


//------------------------------------------------------------------------------

static void Tim_sync_init(void);

//------------------------------------------------------------------------------
//  ALL TIMERS INIT/START
//------------------------------------------------------------------------------

void ccd_timers_init(void)
{
  Tim_sync_init();
}

//------------------------------------------------------------------------------

void ccd_timers_start(void)
{
  LL_TIM_EnableCounter(TIM3); 
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
//  TIMERS INIT FUNCTIONS
//------------------------------------------------------------------------------
//  Master Clock -- TIM3
//------------------------------------------------------------------------------

static void Tim_sync_init(void)
{
  //--- TIM 16 INIT ---//
  
  //--- Peripheral clock enable ---//
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  //--- TIM16 interrupt Init ---//
  NVIC_SetPriority(TIM3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(TIM3_IRQn);
  
  //--- Base Init TIM ---//
  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  TIM_InitStruct.Prescaler = 128 - 1;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 21000 - 1;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  // TIM_InitStruct.RepetitionCounter = 0; // RCR reg
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  
  //--- PWM OC (CH1) -- PA6 ---//
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1); /// LL_TIM_CHANNEL_CH1N
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 11000 - 1;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  //TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  //TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);  //  Enable channel 1
  //TIM3->CCER |= TIM_CCER_CC1E;   /// Enable Ch1 (CC1E) /// | TIM_CCER_CC1P
  
 
  //--- TIM3 PA6 OUTPUT Configuration ---//  
  //LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2; /// PA6 -> TIM16_CH1 -- AF1 - P.44 DT
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  SET_BIT(TIM3->EGR, TIM_EGR_UG);
}


