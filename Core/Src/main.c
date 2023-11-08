/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* Constants */
#define AT86RF2XX_IRQ_STATUS_MASK__BAT_LOW                      (0x80)
#define AT86RF2XX_IRQ_STATUS_MASK__TRX_UR                       (0x40)
#define AT86RF2XX_IRQ_STATUS_MASK__AMI                          (0x20)
#define AT86RF2XX_IRQ_STATUS_MASK__CCA_ED_DONE                  (0x10)
#define AT86RF2XX_IRQ_STATUS_MASK__TRX_END                      (0x08)
#define AT86RF2XX_IRQ_STATUS_MASK__RX_START                     (0x04)
#define AT86RF2XX_IRQ_STATUS_MASK__PLL_UNLOCK                   (0x02)
#define AT86RF2XX_IRQ_STATUS_MASK__PLL_LOCK                     (0x01)

#define CSRESET HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
#define CSSET HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

#define AT86RF2XX_ACCESS_REG                                    (0x80)
#define AT86RF2XX_ACCESS_FB                                     (0x20)
#define AT86RF2XX_ACCESS_SRAM                                   (0x00)
#define AT86RF2XX_ACCESS_READ                                   (0x00)
#define AT86RF2XX_ACCESS_WRITE                                  (0x40)

#define REG_READ     0B10000000 //Read command register access
#define REG_WRITE    0B11000000 //Write command register access
#define FRAME_WRITE  0b01100000

/* SPI Commands */
#define CMD_FB_READ 0B00100000 //Frame Buffer read
#define CMD_SRAM_READ 0B00000000 //SRAM Read command

/* Interrupt codes */
#define IRQ_BAT_LOW	(1 << 7)
#define IRQ_TRX_UR	(1 << 6)
#define IRQ_AMI		(1 << 5)
#define IRQ_CCA_ED	(1 << 4)
#define IRQ_TRX_END	(1 << 3)
#define IRQ_RX_START	(1 << 2)
#define IRQ_PLL_UNL	(1 << 1)
#define IRQ_PLL_LOCK	(1 << 0)

/* Possible states (register: xxx) */
#define STATE_P_ON		0x00	/* BUSY */
#define STATE_BUSY_RX		0x01
#define STATE_BUSY_TX		0x02
#define STATE_FORCE_TRX_OFF	0x03
#define STATE_FORCE_TX_ON	0x04	/* IDLE */
/* 0x05 */				/* INVALID_PARAMETER */
#define STATE_RX_ON		0x06
/* 0x07 */				/* SUCCESS */
#define STATE_TRX_OFF		0x08
#define STATE_TX_ON		0x09
/* 0x0a - 0x0e */			/* 0x0a - UNSUPPORTED_ATTRIBUTE */
#define STATE_SLEEP		0x0F
#define STATE_PREP_DEEP_SLEEP	0x10
#define STATE_BUSY_RX_AACK	0x11
#define STATE_BUSY_TX_ARET	0x12
#define STATE_RX_AACK_ON	0x16
#define STATE_TX_ARET_ON	0x19
#define STATE_RX_ON_NOCLK	0x1C
#define STATE_RX_AACK_ON_NOCLK	0x1D
#define STATE_BUSY_RX_AACK_NOCLK 0x1E
#define STATE_TRANSITION_IN_PROGRESS 0x1F

int received = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t readRegister(const uint8_t addr)
{
    uint8_t value;
    uint8_t readCommand = addr | AT86RF2XX_ACCESS_REG | AT86RF2XX_ACCESS_READ;
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi3, &readCommand, &value, sizeof(value), HAL_MAX_DELAY);
    //HAL_SPI_Transmit(&hspi3, readCommand, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi3, &value, sizeof(value), HAL_MAX_DELAY);
    //HAL_SPI_TransmitReceive(&hspi3, 0x00, value, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

    return (uint8_t)value;
}

void writeRegister(const uint8_t addr,
        const uint8_t value)
{
	uint8_t writeCommand = addr | AT86RF2XX_ACCESS_REG | AT86RF2XX_ACCESS_WRITE;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &writeCommand, sizeof(writeCommand), HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi3, &value, sizeof(value), HAL_MAX_DELAY);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

}

void at86rf233_init(){


	//hardware_reset();
	HAL_GPIO_WritePin(SLP_GPIO_Port, SLP_Pin, GPIO_PIN_RESET);
	//digitalWrite(SLP_TR, LOW);
	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
	//digitalWrite(RESET, HIGH);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	//digitalWrite(SEL, HIGH);


	//

	//

	writeRegister(0x08, 0x0B); //Channel: 2405 MHz

	  // Enable promiscuous mode:
	writeRegister(0x20, 0x12); //Short address bit[7:0]
	writeRegister(0x21, 0x34); //Short address bit[15:8]
	writeRegister(0x22, 0x10); //PAN ID bit[7:0]
	writeRegister(0x23, 0x20); //PAN ID bit[15:8]
	writeRegister(0x24, 0x30); //IEEE_ADDR_0
	writeRegister(0x25, 0x40); //IEEE_ADDR_1
	writeRegister(0x26, 0x50); //IEEE_ADDR_2
	writeRegister(0x27, 0x60); //IEEE_ADDR_3
	writeRegister(0x28, 0x70); //IEEE_ADDR_4
	writeRegister(0x29, 0x80); //IEEE_ADDR_5
	writeRegister(0x2A, 0x90); //IEEE_ADDR_6
	writeRegister(0x2B, 0x22); //IEEE_ADDR_7
	writeRegister(0x05, 0x0); // tx power

	writeRegister(0x17, 0B00000010); //AACK_PROM_MODE
	writeRegister(0x2E, 0B00010000); //AACK_DIS_ACK

	uint8_t TRX_CTRL_0 = readRegister(0x03);
	TRX_CTRL_0 = TRX_CTRL_0 | 0B10000000; //TOM mode enabled
	writeRegister(0x03, TRX_CTRL_0);

	readRegister(0x1C);


	  //Serial.print("Detected part nr: 0x");
	  //Serial.println(readRegister(0x1C), HEX);
	  //Serial.print("Version: 0x");
	  //Serial.println(readRegister(0x1D), HEX);

	HAL_Delay(1);
}

void hardware_reset(void){

	    /* trigger hardware reset */

		HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);
	    HAL_Delay(HAL_MAX_DELAY);
	    HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
	    HAL_Delay(HAL_MAX_DELAY);
}
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
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  at86rf233_init();
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //writeRegister(0x02);
	  uint8_t CurrentState = readRegister(0x01); //Page 37 of datasheet
	  uint8_t Interrupt = readRegister(0x0F);
	  uint8_t PHY_RSSI = readRegister(0x06); //if bit[7] = 1 (RX_CRC_VALID), FCS is valid

	  //  unsigned long jetzt = millis();

	  // This can be used to write status updates
	  //  if (jetzt - zuletzt > Intervall)
	  //  {
	  //    zuletzt = jetzt;
	  //
	  //    Serial.print("Status report @ ");
	  //    Serial.print(zuletzt);
	  //    Serial.print(": ");
	  //    Serial.print(CurrentState, HEX);
	  //
	  //    Serial.print("; Interrupt ");
	  //    Serial.println(Interrupt, BIN);
	  //
	  //    if (ctrlZustand == LOW)
	  //      ctrlZustand = HIGH;
	  //    else
	  //      ctrlZustand = LOW;
	  //
	  //    digitalWrite(ctrlLED, ctrlZustand);
	  //
	  //  }

	  if (CurrentState == STATE_P_ON) { //P_ON
	    writeRegister(0x0E, IRQ_CCA_ED); // Interrupt AWAKE_END (IRQ_4) enabled
	    writeRegister(0x02, STATE_TRX_OFF); //Go from P_ON to TRX_OFF state
	    HAL_Delay(1);
	  }
	  if (CurrentState == STATE_TRX_OFF) { //TRX_OFF = 0x00
	    writeRegister(0x0E, IRQ_PLL_LOCK); // Interrupt PLL_LOCK (IRQ_0) enabled
	    writeRegister(0x02, STATE_TX_ON); //Go from TRX_OFF to PLL_ON state
	    HAL_Delay(0.016);
	  }

	  if (Interrupt & IRQ_PLL_LOCK) { //if PLL is locked
	    writeRegister(0x0E, IRQ_RX_START & IRQ_TRX_END & IRQ_AMI); // Interrupts RX_START (IRQ_2), TRX_END (IRQ_3) and AMI (IRQ_5) enabled
	    writeRegister(0x02, STATE_RX_AACK_ON); //Go from PLL_ON to RX_AACK_ON state
	    // during RX_ON state, listen for incoming frame, BUSY_RX -> receiving frame, interrupt IRQ_TRX_END -> done
	    HAL_Delay(1);
	  }
	  if (Interrupt & AT86RF2XX_IRQ_STATUS_MASK__RX_START){
		  int abba = 12;

	  }
	  if (Interrupt & IRQ_TRX_END) {
	    // If we receive something, state will be BUSY_RX, interrupt IRQ_TRX_END when receive done
	   // readFrame();
	    received++;
	    //Serial.print("Total frames received: ");
	    //Serial.println(received);
	  }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SLP_Pin|RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_Pin */
  GPIO_InitStruct.Pin = IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SLP_Pin RESET_Pin */
  GPIO_InitStruct.Pin = SLP_Pin|RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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

