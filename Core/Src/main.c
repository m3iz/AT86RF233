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
//tx
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


int received = 0;
int frame_len = 0;
int idle_state = 0;
uint16_t options;
uint8_t state;                      /**< current state of the radio */
uint8_t seq_nr;

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

uint8_t get_status(void){
	return readRegister(0x01);
}

void set_csma_max_retries(int8_t retries)
{
    retries = (retries > 5) ? 5 : retries; /* valid values: 0-5 */
    retries = (retries < 0) ? 7 : retries; /* max < 0 => disable CSMA (set to 7) */
    //DEBUG("[at86rf2xx] opt: Set CSMA retries to %u\n", retries);

    uint8_t tmp = readRegister(AT86RF2XX_REG__XAH_CTRL_0);
    tmp &= ~(AT86RF2XX_XAH_CTRL_0__MAX_CSMA_RETRIES);
    tmp |= (retries << 1);
    writeRegister(AT86RF2XX_REG__XAH_CTRL_0, tmp);
}

void set_csma_backoff_exp(uint8_t min, uint8_t max)
{
    max = (max > 8) ? 8 : max;
    min = (min > max) ? max : min;
    //DEBUG("[at86rf2xx] opt: Set min BE=%u, max BE=%u\n", min, max);

    writeRegister(AT86RF2XX_REG__CSMA_BE, (max << 4) | (min));
}

void set_csma_seed(uint8_t entropy[2])
{
    if(entropy == NULL) {
        //DEBUG("[at86rf2xx] opt: CSMA seed entropy is nullpointer\n");
        return;
    }
    //DEBUG("[at86rf2xx] opt: Set CSMA seed to 0x%x 0x%x\n", entropy[0], entropy[1]);

    writeRegister(AT86RF2XX_REG__CSMA_SEED_0, entropy[0]);

    uint8_t tmp = readRegister(AT86RF2XX_REG__CSMA_SEED_1);
    tmp &= ~(AT86RF2XX_CSMA_SEED_1__CSMA_SEED_1);
    tmp |= entropy[1] & AT86RF2XX_CSMA_SEED_1__CSMA_SEED_1;
    writeRegister(AT86RF2XX_REG__CSMA_SEED_1, tmp);
}

void set_option(uint16_t option, int state)
{
    uint8_t tmp;

    //DEBUG("set option %i to %i\n", option, state);

    /* set option field */
    if (state) {
        options |= option;
        /* trigger option specific actions */
        switch (option) {
            case AT86RF2XX_OPT_CSMA:
                //DEBUG("[at86rf2xx] opt: enabling CSMA mode" \
                      "(4 retries, min BE: 3 max BE: 5)\n");
                /* Initialize CSMA seed with hardware address */
                set_csma_seed(0b101011);
                set_csma_max_retries(4);
                set_csma_backoff_exp(3, 5);
                break;
            case AT86RF2XX_OPT_PROMISCUOUS:
                //DEBUG("[at86rf2xx] opt: enabling PROMISCUOUS mode\n");
                /* disable auto ACKs in promiscuous mode */
                tmp = readRegister(AT86RF2XX_REG__CSMA_SEED_1);
                tmp |= AT86RF2XX_CSMA_SEED_1__AACK_DIS_ACK;
                writeRegister(AT86RF2XX_REG__CSMA_SEED_1, tmp);
                /* enable promiscuous mode */
                tmp = readRegister(AT86RF2XX_REG__XAH_CTRL_1);
                tmp |= AT86RF2XX_XAH_CTRL_1__AACK_PROM_MODE;
                writeRegister(AT86RF2XX_REG__XAH_CTRL_1, tmp);
                break;
            case AT86RF2XX_OPT_AUTOACK:
                //DEBUG("[at86rf2xx] opt: enabling auto ACKs\n");
                tmp = readRegister(AT86RF2XX_REG__CSMA_SEED_1);
                tmp &= ~(AT86RF2XX_CSMA_SEED_1__AACK_DIS_ACK);
                writeRegister(AT86RF2XX_REG__CSMA_SEED_1, tmp);
                break;
            case AT86RF2XX_OPT_TELL_RX_START:
                //DEBUG("[at86rf2xx] opt: enabling SFD IRQ\n");
                tmp = readRegister(AT86RF2XX_REG__IRQ_MASK);
                tmp |= AT86RF2XX_IRQ_STATUS_MASK__RX_START;
                writeRegister(AT86RF2XX_REG__IRQ_MASK, tmp);
                break;
            default:
                /* do nothing */
                break;
        }
    }
    else {
        options &= ~(option);
        /* trigger option specific actions */
        switch (option) {
            case AT86RF2XX_OPT_CSMA:
                //DEBUG("[at86rf2xx] opt: disabling CSMA mode\n");
                /* setting retries to -1 means CSMA disabled */
                set_csma_max_retries(-1);
                break;
            case AT86RF2XX_OPT_PROMISCUOUS:
                //DEBUG("[at86rf2xx] opt: disabling PROMISCUOUS mode\n");
                /* disable promiscuous mode */
                tmp = readRegister(AT86RF2XX_REG__XAH_CTRL_1);
                tmp &= ~(AT86RF2XX_XAH_CTRL_1__AACK_PROM_MODE);
                writeRegister(AT86RF2XX_REG__XAH_CTRL_1, tmp);
                /* re-enable AUTOACK only if the option is set */
                if (options & AT86RF2XX_OPT_AUTOACK) {
                    tmp = readRegister(AT86RF2XX_REG__CSMA_SEED_1);
                    tmp &= ~(AT86RF2XX_CSMA_SEED_1__AACK_DIS_ACK);
                    writeRegister(AT86RF2XX_REG__CSMA_SEED_1,
                                        tmp);
                }
                break;
            case AT86RF2XX_OPT_AUTOACK:
                //DEBUG("[at86rf2xx] opt: disabling auto ACKs\n");
                tmp = readRegister(AT86RF2XX_REG__CSMA_SEED_1);
                tmp |= AT86RF2XX_CSMA_SEED_1__AACK_DIS_ACK;
                writeRegister(AT86RF2XX_REG__CSMA_SEED_1, tmp);
                break;
            case AT86RF2XX_OPT_TELL_RX_START:
                //DEBUG("[at86rf2xx] opt: disabling SFD IRQ\n");
                tmp = readRegister(AT86RF2XX_REG__IRQ_MASK);
                tmp &= ~AT86RF2XX_IRQ_STATUS_MASK__RX_START;
                writeRegister(AT86RF2XX_REG__IRQ_MASK, tmp);
                break;
            default:
                /* do nothing */
                break;
        }
    }
}

void _set_state(uint8_t state_)
{
	writeRegister(AT86RF2XX_REG__TRX_STATE, state_);
    while (get_status() != state_);
}

void set_state(uint8_t state_)
{
	_set_state(state_);
    uint8_t old_state = get_status();

    if (state_ == old_state) {
        return;
    }
    /* make sure there is no ongoing transmission, or state transition already
     * in progress */
    while (old_state == AT86RF2XX_STATE_BUSY_RX_AACK ||
           old_state == AT86RF2XX_STATE_BUSY_TX_ARET ||
           old_state == AT86RF2XX_STATE_IN_PROGRESS) {
        old_state = get_status();
    }

    /* we need to go via PLL_ON if we are moving between RX_AACK_ON <-> TX_ARET_ON */
    if ((old_state == AT86RF2XX_STATE_RX_AACK_ON &&
             state_ == AT86RF2XX_STATE_TX_ARET_ON) ||
        (old_state == AT86RF2XX_STATE_TX_ARET_ON &&
             state_ == AT86RF2XX_STATE_RX_AACK_ON)) {
        _set_state(AT86RF2XX_STATE_PLL_ON);
    }
    /* check if we need to wake up from sleep mode */
    else if (old_state == AT86RF2XX_STATE_SLEEP) {
        //DEBUG("at86rf2xx: waking up from sleep mode\n");
        //assert_awake();
    }
}

void set_addr_short(uint16_t addr)
{
	uint8_t addr_short[2];
    addr_short[0] = addr >> 8;
    addr_short[1] = addr & 0xff;
    writeRegister(AT86RF2XX_REG__SHORT_ADDR_0,
                        addr_short[0]);
    writeRegister(AT86RF2XX_REG__SHORT_ADDR_1,
                        addr_short[1]);
}

void set_addr_long(uint64_t addr)
{
	uint8_t addr_long[8];
    for (int i = 0; i < 8; i++) {
        addr_long[i] = (addr >> ((7 - i) * 8));
        writeRegister((AT86RF2XX_REG__IEEE_ADDR_0 + i), addr_long[i]);
    }
}

void set_pan(uint16_t pan)
{
    //pan = pan_;
    //DEBUG("pan0: %u, pan1: %u\n", (uint8_t)pan, pan >> 8);
    writeRegister(AT86RF2XX_REG__PAN_ID_0, (uint8_t)pan);
    writeRegister(AT86RF2XX_REG__PAN_ID_1, (pan >> 8));
}

void set_chan(uint8_t channel)
{
    uint8_t tmp;

    if (channel < AT86RF2XX_MIN_CHANNEL
        || channel > AT86RF2XX_MAX_CHANNEL) {
        return;
    }
    //chan = channel;
    tmp = readRegister(AT86RF2XX_REG__PHY_CC_CCA);
    tmp &= ~(AT86RF2XX_PHY_CC_CCA_MASK__CHANNEL);
    tmp |= (channel & AT86RF2XX_PHY_CC_CCA_MASK__CHANNEL);
    writeRegister(AT86RF2XX_REG__PHY_CC_CCA, tmp);
}

void at86rf233_init(){


	HAL_GPIO_WritePin(SLP_GPIO_Port, SLP_Pin, GPIO_PIN_RESET);
	//digitalWrite(SLP_TR, LOW);
	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
	//digitalWrite(RESET, HIGH);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	//digitalWrite(SEL, HIGH);

	uint8_t part_num = readRegister(AT86RF2XX_REG__PART_NUM);
	    if (part_num != 0xb) {

	        return -1;
	    }
	//
	hardware_reset();
	reset_state_machine();
	//

	 seq_nr = 0;
	 options = 0;


	  // Enable promiscuous mode:
	set_addr_short(0x431); //в передатчике не важно
	set_pan(0x0023); //не важно
	set_addr_long(0x1222334445666768); //не важно


	set_chan(AT86RF2XX_DEFAULT_CHANNEL); //важно


	writeRegister(0x05, 0x0); // tx power

	/* set default options*/
	 set_option(AT86RF2XX_OPT_PROMISCUOUS, 1);
	 set_option(AT86RF2XX_OPT_AUTOACK, 1);
	 set_option(AT86RF2XX_OPT_CSMA, 1);
	 set_option(AT86RF2XX_OPT_TELL_RX_START, 1);
	 set_option(AT86RF2XX_OPT_TELL_RX_END, 1);

	writeRegister(AT86RF2XX_REG__TRX_CTRL_2, AT86RF2XX_TRX_CTRL_2_MASK__RX_SAFE_MODE); // не важно



	/* disable clock output to save power */
    uint8_t tmp = readRegister(AT86RF2XX_REG__TRX_CTRL_0);
    tmp &= ~(AT86RF2XX_TRX_CTRL_0_MASK__CLKM_CTRL);
    tmp &= ~(AT86RF2XX_TRX_CTRL_0_MASK__CLKM_SHA_SEL);
    tmp |= (AT86RF2XX_TRX_CTRL_0_CLKM_CTRL__OFF);
    writeRegister(AT86RF2XX_REG__TRX_CTRL_0, tmp);

	    /* enable interrupts */
	writeRegister(AT86RF2XX_REG__IRQ_MASK, AT86RF2XX_IRQ_STATUS_MASK__TRX_END);

	    /* clear interrupt flags */
	readRegister(AT86RF2XX_REG__IRQ_STATUS);

	set_state(25); //16 - RX_ACACK 6 - rx 25 - tx очень важно
	////////////////////////////////////////////////////////////


	/*uint8_t val = readRegister(0x04); //TX_AUTO_CRC_ON_1
	val = val | 0b10000;
	writeRegister(0x04, val);

	val = readRegister(0x2C); //MAX_FRAME_RETRIES, MAX_CSMA_RETRIES
	val = val | 0b110110;
	writeRegister(0x2C, val);


	writeRegister(0x2C, 0xEA);//CSMA_SEED
	val = readRegister(0x2E);
	val = val | 0b110;
	writeRegister(0x2E, val);

		//max min be
	val = readRegister(0x2F);
		val = val | 0b110001;
		writeRegister(0x2F, val);


		readRegister(0x1C);

*/
	  //Serial.print("Detected part nr: 0x");
	  //Serial.println(readRegister(0x1C), HEX);
	  //Serial.print("Version: 0x");
	  //Serial.println(readRegister(0x1D), HEX);

	HAL_Delay(1);
}
void force_trx_off()
{
    writeRegister(AT86RF2XX_REG__TRX_STATE, AT86RF2XX_TRX_STATE__FORCE_TRX_OFF);
    while (get_status() != AT86RF2XX_STATE_TRX_OFF);
}
void reset_state_machine()
{
    uint8_t old_state;

    //assert_awake();

    /* Wait for any state transitions to complete before forcing TRX_OFF */
    do {
        old_state = get_status();
    } while (old_state == AT86RF2XX_STATE_IN_PROGRESS);

    force_trx_off();
}

void hardware_reset(void){

	    /* trigger hardware reset */

		HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);
	    HAL_Delay(AT86RF2XX_RESET_PULSE_WIDTH);
	    HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
	    HAL_Delay(AT86RF2XX_RESET_PULSE_WIDTH);
}

void sram_write(const uint8_t offset,
                            const uint8_t *data,
                            const size_t len)
  {
      uint8_t writeCommand = AT86RF2XX_ACCESS_SRAM | AT86RF2XX_ACCESS_WRITE;
      CSRESET;
      HAL_SPI_Transmit(&hspi3, &writeCommand, sizeof(writeCommand), HAL_MAX_DELAY);
     // SPI.transfer(writeCommand);
      //HAL_SPI_Transmit(&hspi3, &offset, sizeof(offset), HAL_MAX_DELAY);
     // HAL_SPI_Transmit(&hspi3, data, sizeof(data), HAL_MAX_DELAY);
     // SPI.transfer((char)offset);
      for (int b=0; b<len; b++) {
       // SPI.transfer(data[b]);
    	  HAL_SPI_Transmit(&hspi3, data[b], sizeof(data[b]), HAL_MAX_DELAY);
      }
      CSSET;
  }



 void tx_prepare(void){
	 uint8_t state;
	     /* make sure ongoing transmissions are finished */
	 do {
	     state = get_status();
	 }
	 while (state == AT86RF2XX_STATE_BUSY_TX_ARET);

	     /* if receiving cancel */
	 if(state == AT86RF2XX_STATE_BUSY_RX_AACK) {
		 //force_trx_off();
	 	 idle_state = AT86RF2XX_STATE_RX_AACK_ON;
	 } else if (state != AT86RF2XX_STATE_TX_ARET_ON) {
	 	 idle_state = state;
	 }
	 	 set_state(AT86RF2XX_STATE_TX_ARET_ON);
	     frame_len = IEEE802154_FCS_LEN;
 }

size_t tx_load(uint8_t *data,
         size_t len, size_t offset)
{
	frame_len += (uint8_t)len + 50;
	sram_write(offset + 1, data, len);
	return offset + len;
}

void tx_exec()
{
    /* write frame length field in FIFO */
    sram_write(0, &(frame_len), 1);
    /* trigger sending of pre-loaded frame */
    writeRegister(AT86RF2XX_REG__TRX_STATE, AT86RF2XX_TRX_STATE__TX_START);
    /*if (at86rf2xx.event_cb && (at86rf2xx.options & AT86RF2XX_OPT_TELL_TX_START)) {
        at86rf2xx.event_cb(NETDEV_EVENT_TX_STARTED, NULL);
    }*/
}
 int send(uint8_t *data, size_t len)
  {
	tx_prepare();
	tx_load(data, len, 0);
	tx_exec();
    return len;
  }

 void fb_read(uint8_t *data,
                        const size_t len)
 {
     uint8_t readCommand = AT86RF2XX_ACCESS_FB | AT86RF2XX_ACCESS_READ;
     //digitalWrite(cs_pin, LOW);
     CSRESET;
     //SPI.transfer(readCommand);
     HAL_SPI_Transmit(&hspi3, &readCommand, sizeof(readCommand), HAL_MAX_DELAY);
     HAL_SPI_Receive(&hspi3, data, len, HAL_MAX_DELAY);
     //for (int b=0; b<len; b++) {
       //data[b] = SPI.transfer(0x00);
     //}
     //digitalWrite(cs_pin, HIGH);
     CSSET;
 }

 size_t rx_len(void)
 {
     uint8_t phr;
     fb_read(&phr, 1);

     /* ignore MSB (refer p.80) and substract length of FCS field */
     return (size_t)((phr & 0x7f) - 2);
 }

 void sram_read(const uint8_t offset,
                          uint8_t *data,
                          const size_t len)
 {
     uint8_t readCommand = AT86RF2XX_ACCESS_SRAM | AT86RF2XX_ACCESS_READ;
     //digitalWrite(cs_pin, LOW);
     CSRESET;
     //SPI.transfer(readCommand);
     HAL_SPI_Transmit(&hspi3, &readCommand, sizeof(readCommand), HAL_MAX_DELAY);
     //SPI.transfer((char)offset);
     HAL_SPI_Transmit(&hspi3, &offset, sizeof(offset), HAL_MAX_DELAY);
     HAL_SPI_Receive(&hspi3, data, len, HAL_MAX_DELAY);
     if(data[0]==0x0)HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
     //for (int b=0; b<len; b++) {
    	// HAL_SPI_Receive(&hspi3, data[b], sizeof(data[b]), HAL_MAX_DELAY);
     //}
     CSSET;
     //digitalWrite(cs_pin, HIGH);
 }

 void rx_read(uint8_t *data, size_t len, size_t offset)
 {
     /* when reading from SRAM, the different chips from the AT86RF2xx family
      * behave differently: the AT86F233, the AT86RF232 and the ATRF86212B return
      * frame length field (PHR) at position 0 and the first data byte at
      * position 1.
      * The AT86RF231 does not return the PHR field and return
      * the first data byte at position 0.
      */
 #ifndef MODULE_AT86RF231
     sram_read(offset+1, data, len);
 #else
     sram_read(offset, data, len);
 #endif
 }

 void at86rf2xx_receive_data() {
   /*  print the length of the frame
    *  (including the header)
    */
   size_t pkt_len = rx_len();

   /*  Print the frame, byte for byte  */
   uint8_t data[pkt_len];
   rx_read(data, pkt_len, 0);


   /* How many frames is this so far?  */

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
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
  //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //writeRegister(0x02,0x02);
	  //send("bla", sizeof("bla"));
	  uint8_t CurrentState = get_status(); //Page 37 of datasheet
	  uint8_t Interrupt = readRegister(0x0F);
	  uint8_t PHY_RSSI = readRegister(0x06); //if bit[7] = 1 (RX_CRC_VALID), FCS is valid

	  uint8_t irq_mask = readRegister(AT86RF2XX_REG__IRQ_STATUS);

	  if (irq_mask & AT86RF2XX_IRQ_STATUS_MASK__RX_START){
		  	//uint8_t test = 0;
		  	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
	  }
	  uint8_t data[] = {0x0, 0x2,0x2,0x8,0x1,0x4,0x8,0x8};

	  send(data, sizeof(data));
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

