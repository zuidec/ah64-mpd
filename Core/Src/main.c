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
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "logger.h"
#include "uart.h"
#include "bus.h"
#include "spi_bus.h"
#include "dev_id_strings.h"
#include "dfu_load.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define LOG_PREFIX "[ah64-mpd]"
#define REPORT_SIZE 8U
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;

volatile bool USB_Interval_Elapsed = false;
static uint16_t prev_brt = 0;
static uint16_t prev_vid = 0;
static int16_t vid_enc = 0;
static int16_t brt_enc = 0;
static char stdout_buf[128] = {0};
volatile uint8_t report_buf[REPORT_SIZE] = {0};
uart_handle_t* uart_log = (uart_handle_t*)NULL;
static uart_handle_t uart_3;

bus_t spibus;
spi_bus_ctx_t spi_ctx = {&hspi3,FLASH_CS_GPIO_Port,FLASH_CS_Pin,100};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void print_boot_msg(void);
static void check_dfu_input(void);
void update_input(void);
void send_usb_report(void);
uint8_t USBD_CUSTOM_HID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len);
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
    //dfu_check_and_enter();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI3_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */

    setvbuf(stdout, stdout_buf, _IOLBF, sizeof(stdout_buf));
    print_boot_msg();
    uart_log = &uart_3;
    if(UART_OK != uart_init(uart_log, &huart3)){
      if(UART_OK != uart_init(uart_log, &huart3))    {
        //Fall back to plain HAL_UART function
        uart_log = NULL;
        log_error("%sFailed to initialize log UART",LOG_PREFIX);
      }
    }

    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    log_info("%sWaiting for USB to finish configuring...",LOG_PREFIX);
    uint32_t ms_elapsed = HAL_GetTick();
    while(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)  {
        ;;
    }
    log_info("%sConfiguration took %lums!",LOG_PREFIX, HAL_GetTick() - ms_elapsed);
    HAL_TIM_Base_Start_IT(&htim11);
    log_info("%sAll initialization complete, jumping to main program loop",LOG_PREFIX);
  while (1)
  {
    if(USB_Interval_Elapsed)    {
        check_dfu_input();
        update_input();
        send_usb_report();
        USB_Interval_Elapsed = false;
        HAL_TIM_Base_Start_IT(&htim11);

    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim == &htim11) {
            USB_Interval_Elapsed= true;
    }
}

static void check_dfu_input(void)   {

    static uint16_t ticks_5ms = 0;
    if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(VID_BTN_GPIO_Port, VID_BTN_Pin) && GPIO_PIN_RESET == HAL_GPIO_ReadPin(BRT_BTN_GPIO_Port, BRT_BTN_Pin))    {
       ticks_5ms++;
       if(ticks_5ms >= 1000)    {
           log_warning("%sBootloader request received, resetting into DFU mode!", LOG_PREFIX);
           //dfu_request_and_reset();
           reboot_dfu();
       }
    }
    else    {
        ticks_5ms = 0;
    }
    
}

static inline int16_t enc_delta_16(TIM_HandleTypeDef *htim, uint16_t *prev)
{
    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(htim);
    int16_t d = (int16_t)(now - *prev);   // signed handles wrap naturally in 2's complement
    *prev = now;
    return d;
}

static inline uint32_t compress_bits_u32(uint32_t v, uint32_t mask)
{
    uint32_t out = 0;
    uint32_t bit = 0;

    while (mask) {
        uint32_t lsb = mask & (0u - mask);
        out |= ((v & lsb) ? 0u : 1u) << bit;
        mask &= (mask - 1u);
        bit++;
    }
    return out;
}

static inline uint32_t popcount_u32(uint32_t x)
{
    uint32_t n = 0;
    while (x) { x &= (x - 1u); n++; }
    return n;
}

static inline void pack_buttons(uint8_t* out5)
{
    uint32_t idrA = GPIOA->IDR;
    uint32_t idrB = GPIOB->IDR;
    uint32_t idrC = GPIOC->IDR;
    uint32_t idrD = GPIOD->IDR;
    uint32_t idrE = GPIOE->IDR;

    const struct { uint32_t idr; uint32_t mask; } groups[] = {

        // GPIOE buttons
        { idrE, T1_Pin | T2_Pin | T3_Pin | T4_Pin | T5_Pin | B2_Pin | B3_Pin | B4_Pin | 
                B5_Pin | B6_Pin | B7_Pin | B8_Pin | R1_Pin | R2_Pin },

        // GPIOC buttons
        { idrC, T6_Pin | L1_Pin | L2_Pin | L8_Pin | VID_BTN_Pin },

        // GPIOA buttons
        { idrA, L3_Pin | L4_Pin | L5_Pin | L6_Pin | L7_Pin },

        // GPIOB buttons
        { idrB, BRT_BTN_Pin | B1_Pin | R3_Pin | R4_Pin | R5_Pin | R6_Pin | R7_Pin },

        // GPIOD buttons 
        { idrD, R8_Pin | R9_Pin | SW_DAY_Pin | SW_MONO_Pin | SW_NT_Pin},
    };

    uint64_t bits = 0;
    uint32_t bitpos = 0;

    for (uint32_t i = 0; i < 5; i++) {
        uint32_t packed = compress_bits_u32(groups[i].idr, groups[i].mask);
        uint32_t nbits  = popcount_u32(groups[i].mask);
        bits |= ((uint64_t)packed) << bitpos;
        bitpos += nbits;
    }

    // Write 36 bits → 5 bytes
    out5[0] = (uint8_t)(bits >> 0);
    out5[1] = (uint8_t)(bits >> 8);
    out5[2] = (uint8_t)(bits >> 16);
    out5[3] = (uint8_t)(bits >> 24);
    out5[4] = (uint8_t)(bits >> 32);
}

static inline int8_t clamp_i8(int16_t v)
{
    if (v > 127) return 127;
    if (v < -127) return -127;
    return (int8_t)v;
}

static inline uint8_t clamp_u8(int16_t v)
{
    if(v>255)   {
        return (uint8_t) 255;
    }
    else if (v<0)   {
        return (uint8_t) 0;
    }
    else    {
        return (uint8_t)v;
    }
}

static inline int8_t pop_detents(int16_t *acc)
{
    int8_t det = (int8_t)(*acc / 4);
    *acc -= (int16_t)det * 4;
    return det;
}

static inline int8_t encoder_to_hid(int16_t delta_counts)
{
    // If your encoder produces 4 counts per detent, use >>2
    // If 2 counts per detent, use >>1
    // Start with /4 and adjust after testing.
    int16_t detents = delta_counts / 4;
    return clamp_i8(detents);
}

void update_input(void) {
    static uint8_t brt_axis = 128;
    static uint8_t vid_axis = 128;
    static int8_t brt_detents = 0;
    static int8_t vid_detents = 0;

    report_buf[0] = 0x01; // report ID
    pack_buttons(&report_buf[1]);

    vid_enc += enc_delta_16(&htim3,&prev_vid);
    brt_enc += enc_delta_16(&htim4,&prev_brt);
    vid_detents = pop_detents(&vid_enc);
    brt_detents = pop_detents(&brt_enc);

    if(vid_detents){
        vid_axis = clamp_u8((int16_t)vid_axis + (int16_t)vid_detents * 4);
    }
    if(brt_detents){
        brt_axis = clamp_u8((int16_t)brt_axis + (int16_t)brt_detents * 4);
    }

    // Encoders (example placeholders)
    report_buf[6] = vid_axis; 
    report_buf[7] = brt_axis;
}

void send_usb_report(void)  {
    volatile static uint8_t send_buf[REPORT_SIZE] = {0};
    memcpy(send_buf,report_buf,sizeof(send_buf));
    uint8_t status = USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, send_buf, sizeof(send_buf));
    if(status != USBD_OK) {
        if(status==USBD_BUSY)   {
        	return;
        }
        else if(status != USBD_OK)   {
            log_warning("%sUSB SendReport failed with error: %d", LOG_PREFIX, status);
        }
    }
    if(status==USBD_OK) {
        //vid_enc -= (int16_t)((int8_t)report_buf[6]*4);
        //brt_enc -= (int16_t)((int8_t)report_buf[7]*4);
    }

}

static void print_boot_msg(void) {
    log_printf("\n");
	log_info("\033[0;37mBooting from internal flash...");
    uint32_t uuid[3] =  {HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2()}; 
    uint32_t devid = HAL_GetDEVID();
    log_info("\033[0;37mDevice family: %s, Device revision: 0x%04x", get_stm32_family_str((uint16_t)(devid & 0x0FFF)),(uint16_t)((devid >> 16) & 0xFFFF));
    log_info("\033[0;37mDevice UID: %lu%lu%lu", uuid[0], uuid[1], uuid[2]);
	RCC_ClkInitTypeDef clk;
	uint32_t flashLatency;
	HAL_RCC_GetClockConfig(&clk, &flashLatency);
	uint32_t sysclk = HAL_RCC_GetSysClockFreq();
	uint32_t hclk = HAL_RCC_GetHCLKFreq();
	uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
	uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
    log_info("\033[0;37msysclk: %lu, hclk: %lu, pclk1: %lu, pclk2: %lu, flash_latency: %lu",
                sysclk, hclk, pclk1, pclk2, flashLatency);
}

PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, 0xFFFF);
	//CDC_Transmit_FS((uint8_t*)&ch, 1);

	return ch;
}

int _write(int fd, char *ptr, int len) {

    if(uart_log!=NULL)  {
        uart_write(uart_log, (uint8_t*)ptr, len);
    }
    else    {
        HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    }
	return len;
}
#ifdef halerror
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
#endif
  /* USER CODE END Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
