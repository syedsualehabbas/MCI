/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   Lab 10 — Dynamic Memory (malloc, calloc, free)
  *           UART output via USART2 @115200
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define SIZE 10
/* USER CODE END PD */

I2C_HandleTypeDef  hi2c1;
SPI_HandleTypeDef  hspi1;
UART_HandleTypeDef huart2;
PCD_HandleTypeDef  hpcd_USB_FS;

/* USER CODE BEGIN PV */
static char buffer[128];
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_PCD_Init(void);

/* USER CODE BEGIN PFP */
static void uart_send(const char *text)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)text, strlen(text), HAL_MAX_DELAY);
}
/* USER CODE END PFP */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_USART2_UART_Init();
    MX_USB_PCD_Init();

    HAL_Delay(100);

    uart_send("=== Dynamic Allocation Demo ===\r\n\r\n");

    /* ---------- malloc section ---------- */
    uart_send(">> malloc()\r\n");

    int *m_ptr = malloc(SIZE * sizeof(int));

    if (m_ptr == NULL)
    {
        uart_send("malloc failed!\r\n");
    }
    else
    {
        uart_send("malloc ok\r\n");

        for (int i = 0; i < SIZE; i++)
            m_ptr[i] = i * 2;

        uart_send("Data: ");
        for (int i = 0; i < SIZE; i++)
        {
            int len = snprintf(buffer, sizeof(buffer),
                               "%d%s", m_ptr[i], (i < SIZE - 1) ? ", " : "\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);
        }
    }

    /* ---------- calloc section ---------- */
    uart_send("\r\n>> calloc()\r\n");

    int *c_ptr = calloc(SIZE, sizeof(int));

    if (c_ptr == NULL)
    {
        uart_send("calloc failed!\r\n");
    }
    else
    {
        uart_send("calloc ok\r\n");

        uart_send("Initial (zero): ");
        for (int i = 0; i < SIZE; i++)
        {
            int len = snprintf(buffer, sizeof(buffer),
                               "%d%s", c_ptr[i], (i < SIZE - 1) ? ", " : "\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);
        }

        for (int i = 0; i < SIZE; i++)
            c_ptr[i] = i + 1;

        uart_send("After assign: ");
        for (int i = 0; i < SIZE; i++)
        {
            int len = snprintf(buffer, sizeof(buffer),
                               "%d%s", c_ptr[i], (i < SIZE - 1) ? ", " : "\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);
        }
    }

    /* ---------- free section ---------- */
    uart_send("\r\n>> free()\r\n");

    if (m_ptr)
    {
        free(m_ptr);
        m_ptr = NULL;
        uart_send("m_ptr cleared\r\n");
    }

    if (c_ptr)
    {
        free(c_ptr);
        c_ptr = NULL;
        uart_send("c_ptr cleared\r\n");
    }

    uart_send("\r\nAll done.\r\n");

    while (1)
    {
        /* nothing to do */
    }
}

/* ================= CLOCK CONFIG ================= */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    RCC_PeriphCLKInitTypeDef periph = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_BYPASS;
    osc.HSIState = RCC_HSI_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL = RCC_PLL_MUL6;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) != HAL_OK) Error_Handler();

    periph.PeriphClockSelection = RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USB | RCC_PERIPHCLK_I2C1;
    periph.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    periph.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    periph.USBClockSelection = RCC_USBCLKSOURCE_PLL;

    if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK) Error_Handler();
}

/* ================= INIT FUNCTIONS ================= */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;

    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.DataSize = SPI_DATASIZE_4BIT;

    if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

static void MX_USB_PCD_Init(void)
{
    hpcd_USB_FS.Instance = USB;

    if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

/* ================= ERROR ================= */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}