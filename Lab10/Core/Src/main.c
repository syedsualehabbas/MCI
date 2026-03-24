/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Lab 10 — Task 1 (commented) + Task 2 (active)
  *                   Task 1: malloc/calloc/free demonstration
  *                   Task 2: Custom Heap Driver using Direct SRAM Addressing
  *                   UART2 used for output at 115200 baud.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include "Lab10_heap_driver.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>   /* needed for Task 1: malloc, calloc, free */
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define N 10          /* Task 1: number of elements to allocate */
/* USER CODE END PD */

I2C_HandleTypeDef  hi2c1;
SPI_HandleTypeDef  hspi1;
UART_HandleTypeDef huart2;
PCD_HandleTypeDef  hpcd_USB_FS;

/* USER CODE BEGIN PV */
static char tx_buf[128];
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_PCD_Init(void);

/* USER CODE BEGIN PFP */
static void uart_send(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}

static void uart_send_int(const char *name, int idx, int val)
{
    int len = snprintf(tx_buf, sizeof(tx_buf), "  %s[%d] = %d\r\n", name, idx, val);
    HAL_UART_Transmit(&huart2, (uint8_t *)tx_buf, (uint16_t)len, HAL_MAX_DELAY);
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_USART2_UART_Init();
    MX_USB_PCD_Init();

    /* USER CODE BEGIN 2 */

    HAL_Delay(100);

    /* ====================================================================
     * TASK 1 — Standard Dynamic Memory Allocation (malloc/calloc/free)
     * --------------------------------------------------------------------
     * Demonstrates heap allocation using standard C library functions.
     * Uncomment the block below to run Task 1 instead of Task 2.
     * ==================================================================== */

    /*
    uart_send("========================================\r\n");
    uart_send("  Lab 10 Task 1: Dynamic Memory Alloc  \r\n");
    uart_send("========================================\r\n\r\n");

    // --------------------------------------------------------
    // STEP 1 — malloc()
    // Allocates N integers (uninitialized).
    // Each element is assigned the value i * 2.
    // --------------------------------------------------------
    uart_send("[STEP 1] malloc() — allocate and initialize\r\n");

    int *arr_malloc = (int *)malloc(N * sizeof(int));

    if (arr_malloc == NULL)
    {
        uart_send("  ERROR: malloc() returned NULL. Heap exhausted!\r\n");
    }
    else
    {
        uart_send("  malloc() successful.\r\n");

        for (int i = 0; i < N; i++)
            arr_malloc[i] = i * 2;

        uart_send("  Contents (val = i*2):\r\n");
        for (int i = 0; i < N; i++)
            uart_send_int("arr_malloc", i, arr_malloc[i]);
    }

    // --------------------------------------------------------
    // STEP 2 — calloc()
    // Allocates N integers, all zero-initialized by calloc().
    // Verifies zeros first, then assigns i + 1 to each element.
    // --------------------------------------------------------
    uart_send("\r\n[STEP 2] calloc() — zero-init, then assign\r\n");

    int *arr_calloc = (int *)calloc(N, sizeof(int));

    if (arr_calloc == NULL)
    {
        uart_send("  ERROR: calloc() returned NULL. Heap exhausted!\r\n");
    }
    else
    {
        uart_send("  calloc() successful.\r\n");

        uart_send("  Before assignment (expect all 0):\r\n");
        for (int i = 0; i < N; i++)
            uart_send_int("arr_calloc", i, arr_calloc[i]);

        for (int i = 0; i < N; i++)
            arr_calloc[i] = i + 1;

        uart_send("  After assignment (val = i+1):\r\n");
        for (int i = 0; i < N; i++)
            uart_send_int("arr_calloc", i, arr_calloc[i]);
    }

    // --------------------------------------------------------
    // STEP 3 — free()
    // Releases both arrays back to the heap.
    // Pointers set to NULL to prevent dangling references.
    // --------------------------------------------------------
    uart_send("\r\n[STEP 3] free() — release heap memory\r\n");

    free(arr_malloc);
    arr_malloc = NULL;
    uart_send("  arr_malloc freed. Pointer set to NULL.\r\n");

    free(arr_calloc);
    arr_calloc = NULL;
    uart_send("  arr_calloc freed. Pointer set to NULL.\r\n");

    uart_send("\r\n========================================\r\n");
    uart_send("  Task 1 Complete. All memory released.  \r\n");
    uart_send("========================================\r\n");
    */

    /* ====================================================================
     * TASK 2 — Custom Heap Driver (Direct SRAM Addressing)
     * --------------------------------------------------------------------
     * Uses a custom allocator (Lab10_heap_driver.c) that manages a fixed
     * 4 KB region of SRAM starting at 0x20001000, divided into 256 blocks
     * of 16 bytes each, tracked via a bitmap (block_map[]).
     * ==================================================================== */

    uart_send("========================================\r\n");
    uart_send("  Lab 10 Task 2: Custom Heap Driver     \r\n");
    uart_send("========================================\r\n\r\n");

    // Step 1: Initialize the custom heap — clears all 256 block_map entries to 0
    heap_init();
    uart_send("[INFO] Heap initialized.\r\n");

    // Step 2: Allocate two memory blocks from custom heap
    // block1: 32 bytes → occupies 2 blocks (0x20001000 to 0x2000101F)
    // block2: 48 bytes → occupies 3 blocks (0x20001020 to 0x2000104F)
    char *block1 = (char *)heap_alloc(32);
    char *block2 = (char *)heap_alloc(48);

    // Step 3: Verify allocations and write data
    if (block1 != NULL && block2 != NULL)
    {
        uart_send("[INFO] Both blocks allocated successfully.\r\n\r\n");

        // Print addresses to confirm correct SRAM placement
        int len = snprintf(tx_buf, sizeof(tx_buf),
                           "[ADDR] block1 = 0x%08lX\r\n", (uint32_t)block1);
        HAL_UART_Transmit(&huart2, (uint8_t *)tx_buf, len, HAL_MAX_DELAY);

        len = snprintf(tx_buf, sizeof(tx_buf),
                       "[ADDR] block2 = 0x%08lX\r\n\r\n", (uint32_t)block2);
        HAL_UART_Transmit(&huart2, (uint8_t *)tx_buf, len, HAL_MAX_DELAY);

        // Write strings into raw SRAM
        strcpy(block1, "Data in Block 1");
        strcpy(block2, "Text from Block 2");

        // Read back and print contents
        uart_send("[DATA] block1 contains: ");
        uart_send(block1);
        uart_send("\r\n");

        uart_send("[DATA] block2 contains: ");
        uart_send(block2);
        uart_send("\r\n");
    }
    else
    {
        if (block1 == NULL) uart_send("[ERROR] block1 allocation failed!\r\n");
        if (block2 == NULL) uart_send("[ERROR] block2 allocation failed!\r\n");
    }

    // Step 4: Free both blocks — resets block_map entries back to 0
    uart_send("\r\n[INFO] Freeing block1...\r\n");
    heap_free(block1);
    block1 = NULL;    /* prevent dangling pointer */

    uart_send("[INFO] Freeing block2...\r\n");
    heap_free(block2);
    block2 = NULL;    /* prevent dangling pointer */

    uart_send("Blocks freed.\r\n");

    uart_send("\r\n========================================\r\n");
    uart_send("  Task 2 Complete.                       \r\n");
    uart_send("========================================\r\n");

    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef       RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef       RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit     = {0};

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState            = RCC_HSE_BYPASS;
    RCC_OscInitStruct.HSEPredivValue      = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL6;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) Error_Handler();

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_USART2
                                       | RCC_PERIPHCLK_I2C1;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_HSI;
    PeriphClkInit.USBClockSelection    = RCC_USBCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance              = I2C1;
    hi2c1.Init.Timing           = 0x2000090E;
    hi2c1.Init.OwnAddress1      = 0;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2      = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) Error_Handler();
}

static void MX_SPI1_Init(void)
{
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_4BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 7;
    hspi1.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode          = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance            = USART2;
    huart2.Init.BaudRate       = 115200;
    huart2.Init.WordLength     = UART_WORDLENGTH_8B;
    huart2.Init.StopBits       = UART_STOPBITS_1;
    huart2.Init.Parity         = UART_PARITY_NONE;
    huart2.Init.Mode           = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling   = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_USB_PCD_Init(void)
{
    hpcd_USB_FS.Instance                     = USB;
    hpcd_USB_FS.Init.dev_endpoints           = 8;
    hpcd_USB_FS.Init.speed                   = PCD_SPEED_FULL;
    hpcd_USB_FS.Init.phy_itface              = PCD_PHY_EMBEDDED;
    hpcd_USB_FS.Init.low_power_enable        = DISABLE;
    hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
    if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin | LD4_Pin | LD3_Pin | LD5_Pin
                           | LD7_Pin | LD9_Pin | LD10_Pin | LD8_Pin
                           | LD6_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin  = DRDY_Pin | MEMS_INT3_Pin | MEMS_INT4_Pin
                         | MEMS_INT1_Pin | MEMS_INT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = CS_I2C_SPI_Pin | LD4_Pin | LD3_Pin | LD5_Pin
                          | LD7_Pin | LD9_Pin | LD10_Pin | LD8_Pin | LD6_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif