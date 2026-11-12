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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void uart_print(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s, strlen(s), 100);
}

#define EE_ADDR   (0x50 << 1)
#define EE_PAGE   64

static HAL_StatusTypeDef ee_write(uint16_t addr, const uint8_t *data, uint16_t len)
{
    while (len > 0)
    {
        // bytes left in the current 64-byte page
        uint16_t chunk = EE_PAGE - (addr % EE_PAGE);
        if (chunk > len) chunk = len;

        HAL_StatusTypeDef s = HAL_I2C_Mem_Write(&hi2c1, EE_ADDR, addr,
                                I2C_MEMADD_SIZE_16BIT, (uint8_t *)data, chunk, 100);
        if (s != HAL_OK) return s;

        // wait until the chip finishes its internal write cycle (about 5 ms)
        uint32_t t = HAL_GetTick();
        while (HAL_I2C_IsDeviceReady(&hi2c1, EE_ADDR, 1, 10) != HAL_OK)
            if (HAL_GetTick() - t > 20) return HAL_TIMEOUT;

        addr += chunk;
        data += chunk;
        len  -= chunk;
    }
    return HAL_OK;
}

static HAL_StatusTypeDef ee_read(uint16_t addr, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, EE_ADDR, addr,
                            I2C_MEMADD_SIZE_16BIT, data, len, 100);
}

/* ---------- UART receive (interrupt, ring buffer) ---------- */
#define RX_BUF_SIZE 128
#define LINE_MAX    100

static volatile uint8_t  rx_ring[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0, rx_tail = 0;
static uint8_t rx_byte;

/* ---------- EEPROM commands ---------- */
static void cmd_save(char *args)
{
    uint8_t len = strlen(args);
    if (len == 0) { uart_print("usage: save <text>\r\n"); return; }

    uint8_t buf[LINE_MAX + 1];
    buf[0] = len;                       /* length byte at address 0 */
    memcpy(&buf[1], args, len);
    uart_print(ee_write(0x0000, buf, len + 1) == HAL_OK ? "saved\r\n"
                                                        : "write error\r\n");
}

static void cmd_load(char *args)
{
    (void)args;
    uint8_t len = 0;
    if (ee_read(0x0000, &len, 1) != HAL_OK) { uart_print("read error\r\n"); return; }
    if (len == 0 || len > LINE_MAX)         { uart_print("nothing stored\r\n"); return; }

    char buf[LINE_MAX + 1];
    ee_read(0x0001, (uint8_t *)buf, len);
    buf[len] = '\0';
    uart_print(buf);
    uart_print("\r\n");
}

static void cmd_clear(char *args)
{
    (void)args;
    uint8_t zero = 0;
    uart_print(ee_write(0x0000, &zero, 1) == HAL_OK ? "cleared\r\n"
                                                    : "write error\r\n");
}

static void cmd_dump(char *a)           /* dump <addr> [len] */
{
    char *end;
    uint32_t addr = strtoul(a, &end, 0);
    if (end == a) { uart_print("usage: dump <addr> [len]\r\n"); return; }
    a = end;
    uint32_t len = strtoul(a, &end, 0);
    if (end == a || len == 0) len = 32;
    if (len > 256) len = 256;
    if (addr + len > 0x8000) { uart_print("out of range\r\n"); return; }

    uint8_t buf[16];
    char m[96];
    for (uint32_t off = 0; off < len; off += 16)
    {
        uint16_t n = (len - off > 16) ? 16 : (len - off);
        if (ee_read(addr + off, buf, n) != HAL_OK) { uart_print("read error\r\n"); return; }
        int p = snprintf(m, sizeof(m), "%04lX: ", addr + off);
        for (uint16_t i = 0; i < n; i++)
            p += snprintf(m + p, sizeof(m) - p, "%02X ", buf[i]);
        snprintf(m + p, sizeof(m) - p, "\r\n");
        uart_print(m);
    }
}

static void cmd_wr(char *a)             /* wr <addr> <value> */
{
    char *end;
    uint32_t addr = strtoul(a, &end, 0);
    if (end == a) { uart_print("usage: wr <addr> <value>\r\n"); return; }
    a = end;
    uint32_t val = strtoul(a, &end, 0);
    if (end == a || addr > 0x7FFF || val > 255) { uart_print("usage: wr <addr> <value>\r\n"); return; }

    uint8_t b = (uint8_t)val;
    uart_print(ee_write((uint16_t)addr, &b, 1) == HAL_OK ? "ok\r\n" : "write error\r\n");
}

static void cmd_rd(char *a)             /* rd <addr> */
{
    char *end;
    uint32_t addr = strtoul(a, &end, 0);
    if (end == a || addr > 0x7FFF) { uart_print("usage: rd <addr>\r\n"); return; }

    uint8_t b;
    if (ee_read((uint16_t)addr, &b, 1) != HAL_OK) { uart_print("read error\r\n"); return; }
    char m[32];
    snprintf(m, sizeof(m), "0x%02X (%u)\r\n", b, b);
    uart_print(m);
}

static void cmd_help(char *args);

/* ---------- command table ---------- */
typedef struct {
    const char *name;
    void (*fn)(char *args);
    const char *help;
} cmd_t;

static const cmd_t cmds[] = {
    { "help",  cmd_help,  "list commands" },
    { "save",  cmd_save,  "save <text>      store text in EEPROM" },
    { "load",  cmd_load,  "load             print stored text" },
    { "clear", cmd_clear, "clear            forget stored text" },
    { "dump",  cmd_dump,  "dump <addr> [len] hex dump" },
    { "wr",    cmd_wr,    "wr <addr> <val>  write one byte" },
    { "rd",    cmd_rd,    "rd <addr>        read one byte" },
};
#define NUM_CMDS (sizeof(cmds) / sizeof(cmds[0]))

static void cmd_help(char *args)
{
    (void)args;
    for (size_t i = 0; i < NUM_CMDS; i++)
    {
        uart_print(cmds[i].help);
        uart_print("\r\n");
    }
}

/* ---------- parser ---------- */
static void shell_execute(char *l)
{
    while (*l == ' ') l++;
    char *args = l;
    while (*args && *args != ' ') args++;
    if (*args) { *args++ = '\0'; while (*args == ' ') args++; }

    for (size_t i = 0; i < NUM_CMDS; i++)
    {
        if (strcmp(l, cmds[i].name) == 0) { cmds[i].fn(args); return; }
    }
    uart_print("unknown command, type help\r\n");
}

/* ---------- line editor, called from the main loop ---------- */
static char     line[LINE_MAX];
static uint16_t line_len = 0;

static void shell_poll(void)
{
    while (rx_tail != rx_head)
    {
        uint8_t c = rx_ring[rx_tail];
        rx_tail = (rx_tail + 1) % RX_BUF_SIZE;

        if (c == '\r' || c == '\n')
        {
            if (line_len > 0)               /* ignores the second char of CR+LF */
            {
                line[line_len] = '\0';
                uart_print("\r\n");
                shell_execute(line);
                line_len = 0;
                uart_print("> ");
            }
        }
        else if (c == 0x08 || c == 0x7F)    /* backspace */
        {
            if (line_len > 0) { line_len--; uart_print("\b \b"); }
        }
        else if (c >= 32 && c < 127 && line_len < LINE_MAX - 1)
        {
            line[line_len++] = (char)c;
            HAL_UART_Transmit(&huart1, &c, 1, 10);   /* echo */
        }
    }
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);

  // Release the pins from I2C and test them as plain inputs
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);

  {
      uint8_t wr = 0x43;
      uint8_t rd = 0;
      char msg[64];

      HAL_StatusTypeDef w = HAL_I2C_Mem_Write(&hi2c1, 0x50 << 1, 0x0000,
                                              I2C_MEMADD_SIZE_16BIT, &wr, 1, 100);
      HAL_Delay(5);   // wait for the write cycle

      HAL_StatusTypeDef r = HAL_I2C_Mem_Read(&hi2c1, 0x50 << 1, 0x0000,
                                             I2C_MEMADD_SIZE_16BIT, &rd, 1, 100);

      snprintf(msg, sizeof(msg), "write=%d read=%d value=0x%02X\r\n", w, r, rd);
      uart_print(msg);
  }
  HAL_Delay(100);
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  uart_print("\r\nEEPROM shell ready, type help\r\n> ");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    shell_poll();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint16_t next = (rx_head + 1) % RX_BUF_SIZE;
        if (next != rx_tail) { rx_ring[rx_head] = rx_byte; rx_head = next; }
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);   /* re-arm for the next byte */
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);   /* recover from overrun/noise */
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
