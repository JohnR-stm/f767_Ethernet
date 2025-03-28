#ifndef _LED_CONFIG_H_
#define _LED_CONFIG_H_

#define LED_CLK_IsEnabled                       LL_AHB1_GRP1_IsEnabledClock
#define LED_CLK_Enable                          LL_AHB1_GRP1_EnableClock

#define LED_GREEN_BUS                           LL_AHB1_GRP1_PERIPH_GPIOB 
#define LED_GREEN_PORT                          GPIOB
#define LED_GREEN                               LL_GPIO_PIN_0

#endif /* _LED_CONFIG_H_ */