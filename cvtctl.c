/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Copyright (C) 2012 Karl Palsson <karlp@tweak.net.au>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#define PORT_LED GPIOA
#define PIN_LED1 GPIO3
#define PIN_LED2 GPIO2
/*
 * C15 LMH0344 BYPASS 
 * C14 LMH0344 MUTE
 * C13 LMH0344 CD [IN]
 *
 * B3 LMH0303 ENABLE
 *
 * B12 SFP LOS [IN]
 * B14 SFP TXDIS
 */

static void gpio_setup(void)
{
	/* Enable GPIOC clock. */
	/* Manually: */
	//RCC_AHBENR |= RCC_AHBENR_GPIOCEN;
	/* Using API functions: */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);


	/* Set GPIO8 (in GPIO port C) to 'output push-pull'. */
	/* Using API functions: */
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15); //LMH0344 BYPASS
  gpio_clear(GPIOC, GPIO15); //LMH0344 BYPASS

	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14); //LMH0344 MUTE
  gpio_clear(GPIOC, GPIO14); //LMH0344 MUTE

	gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO13);  //LMH0344 CD

	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3); //LMH0302 ENABLE

	gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO12);  //SFP LOS
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14); //SFP TXDIS

	gpio_mode_setup(PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED1);
	gpio_mode_setup(PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED2);
}


int main(void)
{
	int i;

	gpio_setup();

  while (1) {
    int s=0;
    
    if (gpio_get(GPIOC, GPIO13)) {
      //NO SDI SIGNAL
      gpio_set(GPIOB, GPIO14); //SFP TXDIS
      gpio_clear(PORT_LED, PIN_LED1);
    }else{
      //SDI SIGNAL PRESENT
      s|=1;
      gpio_clear(GPIOB, GPIO14); //SFP TXDIS
      gpio_set(PORT_LED, PIN_LED1);
    }
    if (gpio_get(GPIOB, GPIO12)) {
      //NO SFP Signal
      gpio_clear(GPIOB, GPIO3); //LMH0302 ENABLE
      gpio_clear(PORT_LED, PIN_LED2);
    }else{
      //SFP signal
      s|=1;
      gpio_set(PORT_LED, PIN_LED2);
      gpio_set(GPIOB, GPIO3); //LMH0302 ENABLE
    }

    if(!s){
      gpio_set(PORT_LED, PIN_LED2);
      gpio_set(PORT_LED, PIN_LED1);
      for (i = 0; i < 50000; i++) {	/* Wait a bit. */
        __asm__("nop");
      }
      gpio_clear(PORT_LED, PIN_LED2);
      gpio_clear(PORT_LED, PIN_LED1);
    }
    for (i = 0; i < 200000; i++) {	/* Wait a bit. */
      __asm__("nop");
    }
  }

	return 0;
}
