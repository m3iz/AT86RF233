/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

#define AT86RF2XX_MIN_CHANNEL           (11U)
#define AT86RF2XX_MAX_CHANNEL           (26U)
#define AT86RF2XX_DEFAULT_CHANNEL       (11U)
#define LED_Pin GPIO_PIN_7
#define LED_GPIO_Port GPIOA
#define IRQ_Pin GPIO_PIN_10
#define IRQ_GPIO_Port GPIOC
#define CS_Pin GPIO_PIN_11
#define CS_GPIO_Port GPIOC
#define SLP_Pin GPIO_PIN_6
#define SLP_GPIO_Port GPIOB
#define RESET_Pin GPIO_PIN_8
#define RESET_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define AT86RF2XX_XAH_CTRL_1__AACK_FLTR_RES_FT                  (0x20)
#define AT86RF2XX_XAH_CTRL_1__AACK_UPLD_RES_FT                  (0x10)
#define AT86RF2XX_XAH_CTRL_1__AACK_ACK_TIME                     (0x04)
#define AT86RF2XX_XAH_CTRL_1__AACK_PROM_MODE                    (0x02)
#define AT86RF2XX_CSMA_SEED_1__AACK_SET_PD                      (0x20)
#define AT86RF2XX_CSMA_SEED_1__AACK_DIS_ACK                     (0x10)
#define AT86RF2XX_CSMA_SEED_1__AACK_I_AM_COORD                  (0x08)
#define AT86RF2XX_CSMA_SEED_1__CSMA_SEED_1                      (0x07)
#define AT86RF2XX_XAH_CTRL_0__MAX_FRAME_RETRIES                 (0xF0)
#define AT86RF2XX_XAH_CTRL_0__MAX_CSMA_RETRIES                  (0x0E)
#define AT86RF2XX_XAH_CTRL_0__SLOTTED_OPERATION                 (0x01)
#define AT86RF2XX_REG__TRX_STATUS                               (0x01)
#define AT86RF2XX_REG__TRX_STATE                                (0x02)
#define AT86RF2XX_REG__TRX_CTRL_0                               (0x03)
#define AT86RF2XX_REG__TRX_CTRL_1                               (0x04)
#define AT86RF2XX_REG__PHY_TX_PWR                               (0x05)
#define AT86RF2XX_REG__PHY_RSSI                                 (0x06)
#define AT86RF2XX_REG__PHY_ED_LEVEL                             (0x07)
#define AT86RF2XX_REG__PHY_CC_CCA                               (0x08)
#define AT86RF2XX_REG__CCA_THRES                                (0x09)
#define AT86RF2XX_REG__RX_CTRL                                  (0x0A)
#define AT86RF2XX_REG__SFD_VALUE                                (0x0B)
#define AT86RF2XX_REG__TRX_CTRL_2                               (0x0C)
#define AT86RF2XX_REG__ANT_DIV                                  (0x0D)
#define AT86RF2XX_REG__IRQ_MASK                                 (0x0E)
#define AT86RF2XX_REG__IRQ_STATUS                               (0x0F)
#define AT86RF2XX_REG__VREG_CTRL                                (0x10)
#define AT86RF2XX_REG__BATMON                                   (0x11)
#define AT86RF2XX_REG__XOSC_CTRL                                (0x12)
#define AT86RF2XX_REG__CC_CTRL_1                                (0x14)
#define AT86RF2XX_REG__RX_SYN                                   (0x15)
#ifdef MODULE_AT86RF212B
#define AT86RF2XX_REG__RF_CTRL_0                                (0x16)
#endif
#define AT86RF2XX_REG__XAH_CTRL_1                               (0x17)
#define AT86RF2XX_REG__FTN_CTRL                                 (0x18)
#define AT86RF2XX_REG__PLL_CF                                   (0x1A)
#define AT86RF2XX_REG__PLL_DCU                                  (0x1B)
#define AT86RF2XX_REG__PART_NUM                                 (0x1C)
#define AT86RF2XX_REG__VERSION_NUM                              (0x1D)
#define AT86RF2XX_REG__MAN_ID_0                                 (0x1E)
#define AT86RF2XX_REG__MAN_ID_1                                 (0x1F)
#define AT86RF2XX_REG__SHORT_ADDR_0                             (0x20)
#define AT86RF2XX_REG__SHORT_ADDR_1                             (0x21)
#define AT86RF2XX_REG__PAN_ID_0                                 (0x22)
#define AT86RF2XX_REG__PAN_ID_1                                 (0x23)
#define AT86RF2XX_REG__IEEE_ADDR_0                              (0x24)
#define AT86RF2XX_REG__IEEE_ADDR_1                              (0x25)
#define AT86RF2XX_REG__IEEE_ADDR_2                              (0x26)
#define AT86RF2XX_REG__IEEE_ADDR_3                              (0x27)
#define AT86RF2XX_REG__IEEE_ADDR_4                              (0x28)
#define AT86RF2XX_REG__IEEE_ADDR_5                              (0x29)
#define AT86RF2XX_REG__IEEE_ADDR_6                              (0x2A)
#define AT86RF2XX_REG__IEEE_ADDR_7                              (0x2B)
#define AT86RF2XX_REG__XAH_CTRL_0                               (0x2C)
#define AT86RF2XX_REG__CSMA_SEED_0                              (0x2D)
#define AT86RF2XX_REG__CSMA_SEED_1                              (0x2E)
#define AT86RF2XX_REG__CSMA_BE                                  (0x2F)
#define AT86RF2XX_REG__TST_CTRL_DIGI                            (0x36)
/** @} */

/**
 * @brief   Bitfield definitions for the TRX_CTRL_0 register
 * @{
 */
#define AT86RF2XX_TRX_CTRL_0_MASK__PAD_IO                       (0xC0)
#define AT86RF2XX_TRX_CTRL_0_MASK__PAD_IO_CLKM                  (0x30)
#define AT86RF2XX_TRX_CTRL_0_MASK__CLKM_SHA_SEL                 (0x08)
#define AT86RF2XX_TRX_CTRL_0_MASK__CLKM_CTRL                    (0x07)

#define AT86RF2XX_TRX_CTRL_0_DEFAULT__PAD_IO                    (0x00)
#define AT86RF2XX_TRX_CTRL_0_DEFAULT__PAD_IO_CLKM               (0x10)
#define AT86RF2XX_TRX_CTRL_0_DEFAULT__CLKM_SHA_SEL              (0x08)
#define AT86RF2XX_TRX_CTRL_0_DEFAULT__CLKM_CTRL                 (0x01)

#define AT86RF2XX_TRX_CTRL_0_CLKM_CTRL__OFF                     (0x00)
#define AT86RF2XX_TRX_CTRL_0_CLKM_CTRL__1MHz                    (0x01)
#define AT86RF2XX_TRX_CTRL_0_CLKM_CTRL__2MHz                    (0x02)
#define AT86RF2XX_TRX_CTRL_0_CLKM_CTRL__4MHz                    (0x03)
#define AT86RF2XX_TRX_CTRL_0_CLKM_CTRL__8MHz                    (0x04)
#define AT86RF2XX_TRX_CTRL_0_CLKM_CTRL__16MHz                   (0x05)
#define AT86RF2XX_TRX_CTRL_0_CLKM_CTRL__250kHz                  (0x06)
#define AT86RF2XX_TRX_CTRL_0_CLKM_CTRL__62_5kHz                 (0x07)

#define AT86RF2XX_IRQ_STATUS_MASK__BAT_LOW                      (0x80)
#define AT86RF2XX_IRQ_STATUS_MASK__TRX_UR                       (0x40)
#define AT86RF2XX_IRQ_STATUS_MASK__AMI                          (0x20)
#define AT86RF2XX_IRQ_STATUS_MASK__CCA_ED_DONE                  (0x10)
#define AT86RF2XX_IRQ_STATUS_MASK__TRX_END                      (0x08)
#define AT86RF2XX_IRQ_STATUS_MASK__RX_START                     (0x04)
#define AT86RF2XX_IRQ_STATUS_MASK__PLL_UNLOCK                   (0x02)
#define AT86RF2XX_IRQ_STATUS_MASK__PLL_LOCK                     (0x01)
/** @} */

/**
 * @brief   Bitfield definitions for the TRX_CTRL_1 register
 * @{
 */
#define AT86RF2XX_TRX_STATE_MASK__TRAC                          (0xe0)

#define AT86RF2XX_TRX_STATE__NOP                                (0x00)
#define AT86RF2XX_TRX_STATE__TX_START                           (0x02)
#define AT86RF2XX_TRX_STATE__FORCE_TRX_OFF                      (0x03)
#define AT86RF2XX_TRX_STATE__FORCE_PLL_ON                       (0x04)
#define AT86RF2XX_TRX_STATE__RX_ON                              (0x06)
#define AT86RF2XX_TRX_STATE__TRX_OFF                            (0x08)
#define AT86RF2XX_TRX_STATE__PLL_ON                             (0x09)
#define AT86RF2XX_TRX_STATE__RX_AACK_ON                         (0x16)
#define AT86RF2XX_TRX_STATE__TX_ARET_ON                         (0x19)
#define AT86RF2XX_TRX_STATE__TRAC_SUCCESS                       (0x00)
#define AT86RF2XX_TRX_STATE__TRAC_SUCCESS_DATA_PENDING          (0x20)
#define AT86RF2XX_TRX_STATE__TRAC_SUCCESS_WAIT_FOR_ACK          (0x40)
#define AT86RF2XX_TRX_STATE__TRAC_CHANNEL_ACCESS_FAILURE        (0x60)
#define AT86RF2XX_TRX_STATE__TRAC_NO_ACK                        (0xa0)
#define AT86RF2XX_TRX_STATE__TRAC_INVALID                       (0xe0)

#define AT86RF2XX_TRX_CTRL_1_MASK__PA_EXT_EN                    (0x80)
#define AT86RF2XX_TRX_CTRL_1_MASK__IRQ_2_EXT_EN                 (0x40)
#define AT86RF2XX_TRX_CTRL_1_MASK__TX_AUTO_CRC_ON               (0x20)
#define AT86RF2XX_TRX_CTRL_1_MASK__RX_BL_CTRL                   (0x10)
#define AT86RF2XX_TRX_CTRL_1_MASK__SPI_CMD_MODE                 (0x0C)
#define AT86RF2XX_TRX_CTRL_1_MASK__IRQ_MASK_MODE                (0x02)
#define AT86RF2XX_TRX_CTRL_1_MASK__IRQ_POLARITY                 (0x01)

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

#define AT86RF2XX_STATE_TRX_OFF      (0x08)     /**< idle */
#define AT86RF2XX_STATE_PLL_ON       (0x09)     /**< ready to transmit */
#define AT86RF2XX_STATE_SLEEP        (0x0f)     /**< sleep mode */
#define AT86RF2XX_STATE_BUSY_RX_AACK (0x11)     /**< busy receiving data */
#define AT86RF2XX_STATE_BUSY_TX_ARET (0x12)     /**< busy transmitting data */
#define AT86RF2XX_STATE_RX_AACK_ON   (0x16)     /**< wait for incoming data */
#define AT86RF2XX_STATE_TX_ARET_ON   (0x19)     /**< ready for sending data */
#define AT86RF2XX_STATE_IN_PROGRESS  (0x1f)     /**< ongoing state conversion */

#define AT86RF2XX_STATE_TRX_OFF      (0x08)     /**< idle */
#define AT86RF2XX_STATE_PLL_ON       (0x09)     /**< ready to transmit */
#define AT86RF2XX_STATE_SLEEP        (0x0f)     /**< sleep mode */
#define AT86RF2XX_STATE_BUSY_RX_AACK (0x11)     /**< busy receiving data */
#define AT86RF2XX_STATE_BUSY_TX_ARET (0x12)     /**< busy transmitting data */
#define AT86RF2XX_STATE_RX_AACK_ON   (0x16)     /**< wait for incoming data */
#define AT86RF2XX_STATE_TX_ARET_ON   (0x19)     /**< ready for sending data */
#define AT86RF2XX_STATE_IN_PROGRESS  (0x1f)     /**< ongoing state conversion */
/** @} */

/**
 * @brief   Internal device option flags
 * @{
 */
#define AT86RF2XX_OPT_AUTOACK        (0x0001)       /**< auto ACKs active */
#define AT86RF2XX_OPT_CSMA           (0x0002)       /**< CSMA active */
#define AT86RF2XX_OPT_PROMISCUOUS    (0x0004)       /**< promiscuous mode
                                                     *   active */
#define AT86RF2XX_OPT_PRELOADING     (0x0008)       /**< preloading enabled */
#define AT86RF2XX_OPT_TELL_TX_START  (0x0010)       /**< notify MAC layer on TX
                                                     *   start */
#define AT86RF2XX_OPT_TELL_TX_END    (0x0020)       /**< notify MAC layer on TX
                                                     *   finished */
#define AT86RF2XX_OPT_TELL_RX_START  (0x0040)       /**< notify MAC layer on RX
                                                     *   start */
#define AT86RF2XX_OPT_TELL_RX_END    (0x0080)       /**< notify MAC layer on RX
                                                     *   finished */
#define AT86RF2XX_OPT_RAWDUMP        (0x0100)       /**< pass RAW frame data to
                                                     *   upper layer */
#define AT86RF2XX_OPT_SRC_ADDR_LONG  (0x0200)       /**< send data using long
                                                     *   source address */
#define AT86RF2XX_OPT_USE_SRC_PAN    (0x0400)       /**< do not compress source


                                                     *   PAN ID */
#define AT86RF2XX_REG__TRX_STATUS                               (0x01)
#define AT86RF2XX_REG__TRX_STATE                                (0x02)
#define AT86RF2XX_REG__TRX_CTRL_0                               (0x03)
#define AT86RF2XX_REG__TRX_CTRL_1                               (0x04)
#define AT86RF2XX_REG__PHY_TX_PWR                               (0x05)
#define AT86RF2XX_REG__PHY_RSSI                                 (0x06)
#define AT86RF2XX_REG__PHY_ED_LEVEL                             (0x07)
#define AT86RF2XX_REG__PHY_CC_CCA                               (0x08)
#define AT86RF2XX_REG__CCA_THRES                                (0x09)
#define AT86RF2XX_REG__RX_CTRL                                  (0x0A)
#define AT86RF2XX_REG__SFD_VALUE                                (0x0B)
#define AT86RF2XX_REG__TRX_CTRL_2                               (0x0C)
#define AT86RF2XX_REG__ANT_DIV                                  (0x0D)
#define AT86RF2XX_REG__IRQ_MASK                                 (0x0E)
#define AT86RF2XX_REG__IRQ_STATUS                               (0x0F)
#define AT86RF2XX_REG__VREG_CTRL                                (0x10)
#define AT86RF2XX_REG__BATMON                                   (0x11)
#define AT86RF2XX_REG__XOSC_CTRL                                (0x12)
#define AT86RF2XX_REG__CC_CTRL_1                                (0x14)
#define AT86RF2XX_REG__RX_SYN                                   (0x15)
#ifdef MODULE_AT86RF212B
#define AT86RF2XX_REG__RF_CTRL_0                                (0x16)
#endif
#define AT86RF2XX_REG__XAH_CTRL_1                               (0x17)
#define AT86RF2XX_REG__FTN_CTRL                                 (0x18)
#define AT86RF2XX_REG__PLL_CF                                   (0x1A)
#define AT86RF2XX_REG__PLL_DCU                                  (0x1B)
#define AT86RF2XX_REG__PART_NUM                                 (0x1C)
#define AT86RF2XX_REG__VERSION_NUM                              (0x1D)
#define AT86RF2XX_REG__MAN_ID_0                                 (0x1E)
#define AT86RF2XX_REG__MAN_ID_1                                 (0x1F)
#define AT86RF2XX_REG__SHORT_ADDR_0                             (0x20)
#define AT86RF2XX_REG__SHORT_ADDR_1                             (0x21)
#define AT86RF2XX_REG__PAN_ID_0                                 (0x22)
#define AT86RF2XX_REG__PAN_ID_1                                 (0x23)
#define AT86RF2XX_REG__IEEE_ADDR_0                              (0x24)
#define AT86RF2XX_REG__IEEE_ADDR_1                              (0x25)
#define AT86RF2XX_REG__IEEE_ADDR_2                              (0x26)
#define AT86RF2XX_REG__IEEE_ADDR_3                              (0x27)
#define AT86RF2XX_REG__IEEE_ADDR_4                              (0x28)
#define AT86RF2XX_REG__IEEE_ADDR_5                              (0x29)
#define AT86RF2XX_REG__IEEE_ADDR_6                              (0x2A)
#define AT86RF2XX_REG__IEEE_ADDR_7                              (0x2B)
#define AT86RF2XX_REG__XAH_CTRL_0                               (0x2C)
#define AT86RF2XX_REG__CSMA_SEED_0                              (0x2D)
#define AT86RF2XX_REG__CSMA_SEED_1                              (0x2E)
#define AT86RF2XX_REG__CSMA_BE                                  (0x2F)
#define AT86RF2XX_REG__TST_CTRL_DIGI                            (0x36)
#define IEEE802154_MAX_HDR_LEN              (23U)
#define IEEE802154_FCF_LEN                  (2U)
#define IEEE802154_FCS_LEN                  (2U)

#define IEEE802154_FCF_TYPE_MASK            (0x07)
#define IEEE802154_FCF_TYPE_BEACON          (0x00)
#define IEEE802154_FCF_TYPE_DATA            (0x01)
#define IEEE802154_FCF_TYPE_ACK             (0x02)
#define IEEE802154_FCF_TYPE_MACCMD          (0x03)
#define AT86RF2XX_RESET_PULSE_WIDTH     (1U)

#define AT86RF2XX_TRX_CTRL_2_MASK__RX_SAFE_MODE                 (0x80)
#define IEEE802154_FCF_SECURITY_EN          (0x08)
#define IEEE802154_FCF_FRAME_PEND           (0x10)
#define IEEE802154_FCF_ACK_REQ              (0x20)
#define IEEE802154_FCF_PAN_COMP             (0x40)

#define IEEE802154_FCF_DST_ADDR_MASK        (0x0c)
#define IEEE802154_FCF_DST_ADDR_VOID        (0x00)
#define IEEE802154_FCF_DST_ADDR_SHORT       (0x08)
#define IEEE802154_FCF_DST_ADDR_LONG        (0x0c)

#define IEEE802154_FCF_VERS_V0              (0x00)
#define IEEE802154_FCF_VERS_V1              (0x10)

#define IEEE802154_FCF_SRC_ADDR_MASK        (0xc0)
#define IEEE802154_FCF_SRC_ADDR_VOID        (0x00)
#define IEEE802154_FCF_SRC_ADDR_SHORT       (0x80)
#define IEEE802154_FCF_SRC_ADDR_LONG        (0xc0)


#define AT86RF2XX_PHY_CC_CCA_MASK__CCA_REQUEST                  (0x80)
#define AT86RF2XX_PHY_CC_CCA_MASK__CCA_MODE                     (0x60)
#define AT86RF2XX_PHY_CC_CCA_MASK__CHANNEL                      (0x1F)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
