/**
 ******************************************************************************
 * rc_receiver.h
 *
 * Empfang der Steuerbefehle vom HM-10 (BLE-UART) ueber USART1
 * (PA9 = TX, PA10 = RX) auf dem STM32L432KB.
 ******************************************************************************
 */
#ifndef RC_RECEIVER_H
#define RC_RECEIVER_H

#include <stdint.h>
#include <stdbool.h>

void   RC_Receiver_Init(void);   /* USART1 + Interrupt konfigurieren        */
void   RC_Receiver_Task(void);   /* zyklisch aufrufen: Parser + Failsafe    */

int8_t RC_GetSteer(void);        /* letzter Lenkwert    -100 ... +100       */
int8_t RC_GetThrottle(void);     /* letzter Gaswert     -100 ... +100       */
bool   RC_IsFailsafe(void);      /* true = kein gueltiges Frame (Timeout)   */

#endif /* RC_RECEIVER_H */
