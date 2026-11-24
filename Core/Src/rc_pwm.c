/**
 ******************************************************************************
 * rc_pwm.c
 *
 * PWM-Erzeugung fuer das RC-Car auf dem STM32L432KB:
 *
 *   PA0 (AF1) = TIM2_CH1  ->  ESC 1 (BLDC-Motor 1)
 *   PA1 (AF1) = TIM2_CH2  ->  ESC 2 (BLDC-Motor 2)
 *   PA3 (AF1) = TIM2_CH4  ->  Lenkservo
 *
 * Signalform: Standard-RC-Servosignal
 *   Periode:    20 ms  (50 Hz)
 *   ESC:         1000 us (Stop) bis 2000 us (Vollgas, unidirektional)
 *   Servo:        500 us (0 Grad) bis 2500 us (180 Grad)
 *
 * Timer-Auslegung:
 *   TIM2 (32 Bit, APB1) wird per Prescaler auf 1 MHz Zaehltakt gebracht.
 *   Damit entspricht 1 Timer-Tick genau 1 us -> CCRx direkt in us setzbar.
 *   ARR = 19999 -> Periode 20000 us = 50 Hz.
 *
 * HINWEIS zum Timer-Handle:
 *   Die CubeMX-Konfiguration aus tim.c stellt htim2 und die GPIO-Konfiguration
 *   bereit. Dieses Modul startet nur die PWM-Kanaele und setzt deren Compare-
 *   Werte, damit TIM2 nur einen Initialisierungspfad besitzt.
 *
 * Einbindung:
 *   1. RC_PWM_Init() nach der Takt-Initialisierung aufrufen
 *   2. RC_PWM_ArmESCs() einmalig beim Start (ESC-Arming, ~3 s Neutral)
 *   3. RC_PWM_SetThrottle() / RC_PWM_SetSteer() aus RC_ApplyOutputs()
 *      in rc_receiver.c aufrufen (Werte -100 ... +100)
 ******************************************************************************
 */

#include "stm32l4xx_hal.h"
#include "tim.h"
#include "rc_pwm.h"
#include <stdint.h>

/* ------------------------- Konfiguration --------------------------------- */
#define ESC_STOP_US        1000U
#define ESC_MAX_US         2000U
#define SERVO_MIN_US       500U
#define SERVO_CENTER_US    1500U
#define SERVO_MAX_US       2500U


/* ========================================================================== */
/*  Initialisierung                                                           */
/* ========================================================================== */
void RC_PWM_Init(void)
{
    HAL_StatusTypeDef status_ch1;
    HAL_StatusTypeDef status_ch2;
    HAL_StatusTypeDef status_ch4;

    MX_TIM2_Init();
    RC_PWM_Neutral();
    status_ch1 = HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    status_ch2 = HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    status_ch4 = HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    if (status_ch1 != HAL_OK || status_ch2 != HAL_OK || status_ch4 != HAL_OK)
    {
        Error_Handler();
    }
}

/* ========================================================================== */
/*  Hilfsfunktionen                                                           */
/* ========================================================================== */
/* -100 ... +100 -> Pulsbreite mit Mittelpunkt bei 1500 us. */
static uint32_t steer_to_pulse(int8_t percent)
{
    if (percent >  100) percent =  100;
    if (percent < -100) percent = -100;

    int32_t pulse = (int32_t)SERVO_MIN_US +
                    ((int32_t)(percent + 100) *
                     ((int32_t)SERVO_MAX_US - (int32_t)SERVO_MIN_US)) / 200;

    if (pulse < (int32_t)SERVO_MIN_US) pulse = (int32_t)SERVO_MIN_US;
    if (pulse > (int32_t)SERVO_MAX_US) pulse = (int32_t)SERVO_MAX_US;
    return (uint32_t)pulse;
}

/* 0 ... +100 -> 1000 ... 2000 us fuer ein unidirektionales ESC.
 * Negative Werte werden als Stop behandelt. */
static uint32_t throttle_to_pulse(int8_t percent)
{
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    return ESC_STOP_US + ((uint32_t)percent *
                          (ESC_MAX_US - ESC_STOP_US)) / 100U;
}

static int8_t clamp_throttle(int8_t percent)
{
    if (percent < 0) return 0;
    if (percent > 100) return 100;
    return percent;
}

/* ========================================================================== */
/*  Oeffentliche API                                                          */
/* ========================================================================== */

/* Elektronisches Differential:
 * steer > 0: Rechtskurve, rechter Motor (CH1) wird reduziert.
 * steer < 0: Linkskurve, linker Motor (CH2) wird reduziert.
 * Bei steer == 0 erhalten beide Motoren den gleichen Wert. */
void RC_PWM_SetDrive(int8_t steer, int8_t throttle)
{
    int16_t inner_throttle;
    int8_t right_throttle = clamp_throttle(throttle);
    int8_t left_throttle = right_throttle;

    if (steer > 100) steer = 100;
    if (steer < -100) steer = -100;

    if (steer > 0) {
        inner_throttle = ((int16_t)right_throttle * (100 - steer)) / 100;
        right_throttle = (int8_t)inner_throttle;
    } else if (steer < 0) {
        inner_throttle = ((int16_t)left_throttle * (100 + steer)) / 100;
        left_throttle = (int8_t)inner_throttle;
    }

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,
                          throttle_to_pulse(right_throttle));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,
                          throttle_to_pulse(left_throttle));
}

void RC_PWM_SetThrottle(int8_t percent)
{
    RC_PWM_SetDrive(0, percent);
}

/* Lenkung, -100 (links) ... +100 (rechts), mit Software-Endanschlaegen */
void RC_PWM_SetSteer(int8_t percent)
{
    uint32_t pulse = steer_to_pulse(percent);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulse);
}

/* Alles auf Neutral (auch vom Failsafe genutzt) */
void RC_PWM_Neutral(void)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ESC_STOP_US);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ESC_STOP_US);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, SERVO_CENTER_US);
}

/* ESC-Arming: Die meisten bidirektionalen Auto-ESCs verlangen beim
 * Einschalten fuer einige Sekunden ein stabiles Neutralsignal, bevor
 * sie den Motor freigeben. Blockierend, daher nur einmal beim Start
 * aufrufen (vor der Hauptschleife).                                          */
void RC_PWM_ArmESCs(void)
{
    RC_PWM_Neutral();
    HAL_Delay(3000);
}
