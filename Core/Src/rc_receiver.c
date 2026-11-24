/**
 ******************************************************************************
 * rc_receiver.c
 *
 * Empfang der RC-Car-Steuerbefehle vom HM-10 (BLE-UART-Bridge)
 * über USART1 auf dem STM32L432KB.
 *
 * Hardware:
 *   PA9  (AF7) = USART1_TX  ->  HM-10 RXD
 *   PA10 (AF7) = USART1_RX  <-  HM-10 TXD
 *   GND gemeinsam, HM-10 mit 3,3 V versorgen
 *   HM-10 Werkseinstellung: 9600 Baud, 8N1
 *
 * Protokoll (vom Web-UI gesendet):
 *   "S:<lenkung>,T:<gas>\n"   S: -100 bis +100, T: 0 bis +100
 *   Beispiel: "S:-25,T:60\n"
 *
 * Failsafe:
 *   Kommt laenger als RC_FAILSAFE_MS kein gueltiges Frame an,
 *   werden Lenkung und Gas auf 0 (neutral) gesetzt.
 *
 * HINWEIS zum UART-Handle:
 *   Das Handle heisst bewusst huart1_rc und nicht huart1, weil die von
 *   CubeMX generierte Datei Core/Src/usart.c bereits ein globales huart1
 *   definiert. Getrennte Namen vermeiden Verwechslungen im Debugger und
 *   Linker-Konflikte, falls das static hier spaeter einmal entfaellt.
 *   Die Konfiguration erfolgt ausschliesslich hier - MX_USART1_UART_Init()
 *   aus usart.c wird nicht aufgerufen.
 *
 * HINWEIS zum Interrupt:
 *   USART1_IRQHandler() ist unten definiert. In CubeMX muss unter
 *   NVIC -> Code generation das Haekchen "Generate IRQ handler" fuer
 *   USART1 abgewaehlt sein, sonst entsteht eine zweite Definition in
 *   Core/Src/stm32l4xx_it.c (Linker-Fehler: multiple definition).
 *
 * Einbindung:
 *   1. RC_Receiver_Init() nach HAL_Init()/SystemClock_Config() aufrufen
 *   2. RC_Receiver_Task() zyklisch in der Hauptschleife aufrufen
 ******************************************************************************
 */

#include "stm32l4xx_hal.h"
#include "rc_receiver.h"
#include "rc_pwm.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------- Konfiguration --------------------------------- */
#define RC_BAUDRATE      9600U     /* HM-10 Standard                          */
#define RC_FAILSAFE_MS   500U      /* Timeout bis Neutralstellung             */
#define RC_FRAME_MAX     32U       /* max. Framelaenge inkl. '\n'             */

/* ------------------------- Modulzustand ----------------------------------- */
static UART_HandleTypeDef huart1_rc;

static volatile uint8_t  rx_byte;                 /* Einzelbyte-Empfang       */
static char              frame_buf[RC_FRAME_MAX]; /* Zeilenpuffer             */
static volatile uint8_t  frame_len   = 0;
static volatile bool     frame_ready = false;

static volatile int8_t   rc_steer    = 0;         /* -100 ... +100            */
static volatile int8_t   rc_throttle = 0;         /* -100 ... +100            */
static volatile uint32_t rc_last_frame_tick = 0;  /* HAL_GetTick() beim
                                                     letzten gueltigen Frame  */
static volatile bool     rc_failsafe_active = true;

//	Einfügung Debugging
static volatile uint32_t rx_byte_count  = 0;
static volatile uint32_t rx_frame_count = 0;
static volatile uint32_t rx_error_count = 0;
/* Diagnosezaehler – nur zum Debuggen */
volatile uint32_t rc_rx_bytes  = 0;
//volatile uint32_t rc_rx_errors = 0;
volatile uint32_t rc_last_err  = 0;
// Ende Einfügung

/* ========================================================================== */
/*  Initialisierung                                                           */
/* ========================================================================== */
void RC_Receiver_Init(void)
{
    /* --- Takt fuer GPIOA und USART1 --- */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    /* --- PA9/PA10 als AF7 (USART1) --- */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;          /* RX-Leitung definiert halten */
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;  /* 9600 Baud braucht nicht mehr */
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* --- USART1: 9600 8N1 --- */
    huart1_rc.Instance          = USART1;
    huart1_rc.Init.BaudRate     = RC_BAUDRATE;
    huart1_rc.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1_rc.Init.StopBits     = UART_STOPBITS_1;
    huart1_rc.Init.Parity       = UART_PARITY_NONE;
    huart1_rc.Init.Mode         = UART_MODE_TX_RX;
    huart1_rc.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1_rc.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1_rc);

    /* --- Interrupt aktivieren und ersten Empfang starten --- */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_UART_Receive_IT(&huart1_rc, (uint8_t *)&rx_byte, 1);

    rc_last_frame_tick = HAL_GetTick();
}

/* ========================================================================== */
/*  Interrupt-Handler (in startup/Vektortabelle referenziert)                 */
/* ========================================================================== */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1_rc);
}

/* Byte empfangen -> Zeilenpuffer fuellen, bei '\n' Frame melden */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Diagnose-Test Debuggen */
//	rc_rx_bytes ++;
	if (huart->Instance != USART1) return;

    uint8_t b = rx_byte;

    if (b == '\n') {
        frame_buf[frame_len] = '\0';
        frame_ready = true;                 /* Auswertung im Task-Kontext */
    } else if (b != '\r') {
        if (frame_len < RC_FRAME_MAX - 1) {
            frame_buf[frame_len++] = (char)b;
        } else {
            frame_len = 0;                  /* Overflow -> Frame verwerfen */
        }
    }

    /* naechstes Byte anfordern */
    HAL_UART_Receive_IT(&huart1_rc, (uint8_t *)&rx_byte, 1);
}

//	Einfügung fürs Debuggen

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    //rc_rx_errors++;
    rc_last_err = huart->ErrorCode;
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    frame_len = 0;
    rx_error_count++;

    HAL_UART_Receive_IT(huart, (uint8_t *)&rx_byte, 1);
}
//	Ende Debugging
/* ========================================================================== */
/*  Frame-Parser: "S:<int>,T:<int>"                                           */
/* ========================================================================== */
static int8_t clamp100(long v)
{
    if (v >  100) return  100;
    if (v < -100) return -100;
    return (int8_t)v;
}

static bool parse_frame(const char *s, int8_t *steer, int8_t *throttle)
{
    if (s[0] != 'S' || s[1] != ':') return false;

    char *end;
    long sv = strtol(&s[2], &end, 10);
    if (end == &s[2] || *end != ',')  return false;

    if (end[1] != 'T' || end[2] != ':') return false;

    const char *tstart = &end[3];
    long tv = strtol(tstart, &end, 10);
    if (end == tstart || *end != '\0') return false;

    *steer    = clamp100(sv);
    *throttle = clamp100(tv);
    return true;
}

/* ========================================================================== */
/*  Ausgabe an das PWM-Modul (rc_pwm.c)                                       */
/* ========================================================================== */
static void RC_ApplyOutputs(int8_t steer, int8_t throttle)
{
    RC_PWM_SetSteer(steer);        /* PA3, TIM2_CH4          */
    RC_PWM_SetDrive(steer, throttle); /* CH1 rechts, CH2 links */
}

/* ========================================================================== */
/*  Zyklischer Task: in der Hauptschleife aufrufen                            */
/* ========================================================================== */
void RC_Receiver_Task(void)
{
    /* 1) Neues Frame auswerten */
    if (frame_ready) {
        int8_t s, t;
        /* Kopie ziehen, damit der IRQ waehrenddessen weiterschreiben darf */
        char local[RC_FRAME_MAX];
        __disable_irq();
        strncpy(local, frame_buf, RC_FRAME_MAX);
        frame_len   = 0;
        frame_ready = false;
        __enable_irq();

        if (parse_frame(local, &s, &t)) {
            rc_steer           = s;
            rc_throttle        = t;
            rc_last_frame_tick = HAL_GetTick();
            rc_failsafe_active = false;
        }
    }

    /* 2) Failsafe pruefen */
    if (!rc_failsafe_active &&
        (HAL_GetTick() - rc_last_frame_tick) > RC_FAILSAFE_MS) {
        rc_steer           = 0;
        rc_throttle        = 0;
        rc_failsafe_active = true;
        /* Optional: LED setzen oder Fehler loggen */
    }

    /* 3) Ausgaenge nachfuehren */
    RC_ApplyOutputs(rc_steer, rc_throttle);
}


//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance != USART1) return;
//
//    rc_rx_errors++;
//    rc_last_err = huart->ErrorCode;
//
//    __HAL_UART_CLEAR_OREFLAG(huart);
//    __HAL_UART_CLEAR_NEFLAG(huart);
//    __HAL_UART_CLEAR_FEFLAG(huart);
//    __HAL_UART_CLEAR_PEFLAG(huart);
//
//    frame_len = 0;
//    HAL_UART_Receive_IT(huart, (uint8_t *)&rx_byte, 1);   /* neu scharfschalten */
//}
/* Ende Diagnosezaheler */

/* ------------------------- Getter fuer andere Module ---------------------- */
int8_t RC_GetSteer(void)      { return rc_steer; }
int8_t RC_GetThrottle(void)   { return rc_throttle; }
bool   RC_IsFailsafe(void)    { return rc_failsafe_active; }
