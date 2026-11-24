/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  ******************************************************************************
  ******************************************************************************
  * RC-Car auf STM32L432KC - Hauptprogramm
  *
  * Systemuebersicht:
  *   Web-UI (Browser) --BLE--> HM-10 --UART 9600--> USART1 (PA9/PA10)
  *       -> rc_receiver.c (Parser + Failsafe)
  *       -> rc_pwm.c      (TIM2: PA0/PA1 = ESC 1/2, PA3 = Lenkservo)
  *
  * Takt: MSI 4 MHz -> PLL -> SYSCLK 80 MHz (nur interne Oszillatoren)
  *
  * Status-LED (PB3 = LD3 auf dem Nucleo-L432KC):
  *   dauerhaft an  -> Verbindung aktiv, Frames kommen an
  *   blinkt (2 Hz) -> Failsafe aktiv (keine/abgerissene Verbindung)
  *
  * ---------------------------------------------------------------------------
  * REIHENFOLGE DER INITIALISIERUNG - bitte nicht umstellen:
  *
  *   MX_TIM2_Init()        Timer konfigurieren (CubeMX)
  *   MX_USART1_UART_Init() UART konfigurieren  (CubeMX)
  *   MX_GPIO_Init()        PB3 als Ausgang     (CubeMX)
  *   RC_PWM_Init()         PWM-Kanaele starten -> 1000 us an den ESCs
  *   RC_PWM_ArmESCs()      BLOCKIEREND ~3 s
  *   MX_IWDG_Init()        Watchdog ERST JETZT starten
  *   RC_Receiver_Init()    UART-Empfang starten
  *
  *   Wuerde MX_IWDG_Init() vor dem Arming laufen, loest der Watchdog
  *   waehrend der 3 s HAL_Delay() aus und das Board resettet endlos.
  *   Deshalb ist fuer IWDG in CubeMX "Do Not Generate Function Call"
  *   gesetzt und der Aufruf steht hier von Hand.
  * ---------------------------------------------------------------------------
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

//Includes-RC-Car
#include "tim.h"		//tim.h --> für PWM
#include "usart.h"		//usart.h --> für BLE

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rc_pwm.h"
#include "rc_receiver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define LED_PORT        GPIOB
#define LED_PIN         GPIO_PIN_3
#define LED_BLINK_MS    250U          /* 2 Hz bei Failsafe */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Status_LED_Task(void);	//LED gibt BLE-Status aus (connected/disconnected)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

    RC_PWM_Init();        /* PWM starten: ESCs 1000 us, Servo 1500 us         */
    RC_PWM_ArmESCs();     /* blockierend ~3 s Leerlauf -> ESCs armen          */
    RC_Receiver_Init();   /* Empfang vom HM-10 starten, USART1 + PA9/PA10     */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    RC_Receiver_Task();          /* Frames parsen, Failsafe, PWM nachfuehren */
	    Status_LED_Task();           /* Verbindungsstatus anzeigen               */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Status-LED an PB3 nachfuehren.
  *         Die Pin-Konfiguration erledigt MX_GPIO_Init() in gpio.c -
  *         hier wird nur geschaltet.
  * @retval None
  */
static void Status_LED_Task(void)
{
  static uint32_t last_toggle = 0;

  if (RC_IsFailsafe())
  {
    /* Failsafe: blinken */
    uint32_t now = HAL_GetTick();
    if ((now - last_toggle) >= LED_BLINK_MS)
    {
      last_toggle = now;
      HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    }
  }
  else
  {
    /* Verbindung aktiv: dauerhaft an */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
