/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

// Helpful macros
#ifdef LED0
#define LED0_TOGGLE              digitalToggle(LED0_GPIO, LED0_PIN)
#ifndef LED0_INVERTED
#define LED0_OFF                 digitalHi(LED0_GPIO, LED0_PIN)
#define LED0_ON                  digitalLo(LED0_GPIO, LED0_PIN)
#else
#define LED0_OFF                 digitalLo(LED0_GPIO, LED0_PIN)
#define LED0_ON                  digitalHi(LED0_GPIO, LED0_PIN)
#endif // inverted
#else
#define LED0_TOGGLE              do {} while(0)
#define LED0_OFF                 do {} while(0)
#define LED0_ON                  do {} while(0)
#endif

#ifdef LED1
#define LED1_TOGGLE              digitalToggle(LED1_GPIO, LED1_PIN)
#ifndef LED1_INVERTED
#define LED1_OFF                 digitalHi(LED1_GPIO, LED1_PIN)
#define LED1_ON                  digitalLo(LED1_GPIO, LED1_PIN)
#else
#define LED1_OFF                 digitalLo(LED1_GPIO, LED1_PIN)
#define LED1_ON                  digitalHi(LED1_GPIO, LED1_PIN)
#endif // inverted
#else
#define LED1_TOGGLE              do {} while(0)
#define LED1_OFF                 do {} while(0)
#define LED1_ON                  do {} while(0)
#endif


#ifdef LED2
#define LED2_TOGGLE              digitalToggle(LED2_GPIO, LED2_PIN)
#ifndef LED2_INVERTED
#define LED2_OFF                 digitalHi(LED2_GPIO, LED2_PIN)
#define LED2_ON                  digitalLo(LED2_GPIO, LED2_PIN)
#else
#define LED2_OFF                 digitalLo(LED2_GPIO, LED2_PIN)
#define LED2_ON                  digitalHi(LED2_GPIO, LED2_PIN)
#endif // inverted
#else
#define LED2_TOGGLE              do {} while(0)
#define LED2_OFF                 do {} while(0)
#define LED2_ON                  do {} while(0)
#endif

#ifdef LED3
#define LED3_TOGGLE              digitalToggle(LED3_GPIO, LED3_PIN)
#ifndef LED3_INVERTED
#define LED3_OFF                 digitalLo(LED3_GPIO, LED3_PIN)
#define LED3_ON                  digitalHi(LED3_GPIO, LED3_PIN)
#else
#define LED3_OFF                 digitalLo(LED3_GPIO, LED3_PIN)
#define LED3_ON                  digitalHi(LED3_GPIO, LED3_PIN)
#endif // inverted
#else
#define LED3_TOGGLE              do {} while(0)
#define LED3_OFF                 do {} while(0)
#define LED3_ON                  do {} while(0)
#endif

#ifdef LED4
#define LED4_TOGGLE              digitalToggle(LED4_GPIO, LED4_PIN)
#ifndef LED4_INVERTED
#define LED4_OFF                 digitalLo(LED4_GPIO, LED4_PIN)
#define LED4_ON                  digitalHi(LED4_GPIO, LED4_PIN)
#else
#define LED4_OFF                 digitalLo(LED4_GPIO, LED4_PIN)
#define LED4_ON                  digitalHi(LED4_GPIO, LED4_PIN)
#endif // inverted
#else
#define LED4_TOGGLE              do {} while(0)
#define LED4_OFF                 do {} while(0)
#define LED4_ON                  do {} while(0)
#endif

void ledInit(void);
