/**
 ******************************************************************************
 * rc_pwm.h
 *
 * PWM-Ausgabe fuer das RC-Car (STM32L432KB):
 *   PA0 = TIM2_CH1 -> unidirektionales ESC 1,
 *   PA1 = TIM2_CH2 -> unidirektionales ESC 2,
 *   PA3 = TIM2_CH4 -> Servo (500 ... 2500 us = 0 ... 180 Grad)
 ******************************************************************************
 */
#ifndef RC_PWM_H
#define RC_PWM_H

#include <stdint.h>

void RC_PWM_Init(void);              /* TIM2 + GPIOs konfigurieren, PWM starten */
void RC_PWM_ArmESCs(void);           /* blockierend: ~3 s Neutral fuer Arming   */
void RC_PWM_SetDrive(int8_t steer, int8_t throttle); /* elektronisches Diff. */
void RC_PWM_SetThrottle(int8_t percent);  /* 0 ... +100, negative Werte = Stop */
void RC_PWM_SetSteer(int8_t percent);     /* -100 ... +100                      */
void RC_PWM_Neutral(void);           /* ESC Stop, Servo Mitte                  */

#endif /* RC_PWM_H */
