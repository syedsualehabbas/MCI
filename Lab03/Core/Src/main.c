
//*********************************************************************** */
//*********************************************************************** */
//*********************************************************************** */
// TASK 4
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Task 4 - Random Digit Display (1-6) on Button Press
  ******************************************************************************
  * @attention
  * 7-Segment Display: COMMON ANODE
  * USER Button (PA0): Display random number 1-6 (like a dice)
  * Uses rand() function with seed for randomness
  * Segment ON  = GPIO LOW  (0)
  * Segment OFF = GPIO HIGH (1)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <stdlib.h>  // For rand() and srand()
#include <time.h>    // For time() - though not available on MCU, we'll use HAL_GetTick()

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN 0 */

/* ===== 7-Segment COMMON ANODE Pin Mapping ===== */
#define SEG_A_PORT GPIOC
#define SEG_A_PIN  GPIO_PIN_11

#define SEG_B_PORT GPIOD
#define SEG_B_PIN  GPIO_PIN_5

#define SEG_C_PORT GPIOA
#define SEG_C_PIN  GPIO_PIN_9

#define SEG_D_PORT GPIOD
#define SEG_D_PIN  GPIO_PIN_4

#define SEG_E_PORT GPIOA
#define SEG_E_PIN  GPIO_PIN_13

#define SEG_F_PORT GPIOD
#define SEG_F_PIN  GPIO_PIN_0

#define SEG_G_PORT GPIOD
#define SEG_G_PIN  GPIO_PIN_2

#define SEG_DP_PORT GPIOC
#define SEG_DP_PIN  GPIO_PIN_9

/* ===== USER Button Configuration ===== */
#define USER_BUTTON_PORT GPIOA
#define USER_BUTTON_PIN  GPIO_PIN_0

/* Global variables for button handling */
uint32_t lastButtonPressTime = 0;
#define DEBOUNCE_DELAY 200  // 200ms debounce delay

/* Function to turn all segments OFF (HIGH for common anode) */
void Segments_AllOff(void)
{
  HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SEG_DP_PORT, SEG_DP_PIN, GPIO_PIN_SET);
}

/* Function to display a single digit (1-6) on 7-segment display */
void Display_Digit(uint8_t digit)
{
  // First turn all segments off
  Segments_AllOff();
  
  // For common anode: 0 = ON, 1 = OFF
  switch(digit)
  {
    case 1: // Display 1
      HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
      break;
      
    case 2: // Display 2
      HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
      break;
      
    case 3: // Display 3
      HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
      break;
      
    case 4: // Display 4
      HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
      break;
      
    case 5: // Display 5
      HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
      break;
      
    case 6: // Display 6
      HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
      break;
      
    default:
      Segments_AllOff();
      break;
  }
}

/* Function to generate random number between 1 and 6 */
uint8_t Generate_Random_1_to_6(void)
{
  // Generate random number between 1 and 6 (like a dice)
  uint8_t randomNum = (rand() % 6) + 1;
  return randomNum;
}

/* Function to check button press with debouncing - Active LOW */
uint8_t Button_IsPressed(void)
{
  static uint8_t lastButtonState = 1;  // Initialize to HIGH (unpressed state for active LOW)
  uint8_t currentButtonState = HAL_GPIO_ReadPin(USER_BUTTON_PORT, USER_BUTTON_PIN);
  uint32_t currentTime = HAL_GetTick();
  
  // Check if button is pressed (falling edge: HIGH to LOW) and debounce time has passed
  if (currentButtonState == GPIO_PIN_RESET && lastButtonState == GPIO_PIN_SET)
  {
    if ((currentTime - lastButtonPressTime) > DEBOUNCE_DELAY)
    {
      lastButtonPressTime = currentTime;
      lastButtonState = currentButtonState;
      return 1; // Button pressed
    }
  }
  
  lastButtonState = currentButtonState;
  return 0; // Button not pressed or debouncing
}

/* USER CODE END 0 */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */
  
  // Seed the random number generator with current tick count
  // This provides pseudo-randomness based on when the device was powered on
  srand(HAL_GetTick());
  
  // Display initial random number
  uint8_t currentNumber = Generate_Random_1_to_6();
  Display_Digit(currentNumber);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Update seed continuously for better randomness
    // This makes the random number depend on WHEN the button is pressed
    srand(HAL_GetTick());
    
    // Check if button is pressed
    if (Button_IsPressed())
    {
      // Generate a new random number between 1 and 6
      currentNumber = Generate_Random_1_to_6();
      
      // Display the random number
      Display_Digit(currentNumber);
    }
    
    /* Small delay to reduce CPU usage */
    HAL_Delay(10);
    
    /* USER CODE END WHILE */
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Configure GPIO pin Output Level - Set all HIGH initially (OFF for common anode) */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_13, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9|GPIO_PIN_11, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);

  /* Configure GPIO pins : GPIOA - Segments C, E (PA9, PA13) */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure GPIO pins : GPIOC - Segments A, DP (PC11, PC9) */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Configure GPIO pins : GPIOD - Segments B, D, F, G (PD5, PD4, PD0, PD2) */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* Configure GPIO pin : USER Button (PA0) as INPUT */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;  // Pull-up resistor for active LOW button (connected to GND)
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Ensure all segments are OFF initially */
  Segments_AllOff();
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
//*********************************************************************** */
//*********************************************************************** */
//*********************************************************************** */
//TASK 3
// /* USER CODE BEGIN Header */
// /**
//   ******************************************************************************
//   * @file           : main.c
//   * @brief          : Task 3 - Dual Button Counter (+1/-1)
//   ******************************************************************************
//   * @attention
//   * 7-Segment Display: COMMON ANODE
//   * USER Button (PA0): Increment counter (+1) - Active LOW (connected to GND)
//   * External Button (PB5): Decrement counter (-1) - Active HIGH (connected to 3.3V with pull-down)
//   * Counter Range: 0-9 (wraps around)
//   * Segment ON  = GPIO LOW  (0)
//   * Segment OFF = GPIO HIGH (1)
//   ******************************************************************************
//   */
// /* USER CODE END Header */

// #include "main.h"

// /* Private variables ---------------------------------------------------------*/
// I2C_HandleTypeDef hi2c1;
// SPI_HandleTypeDef hspi1;
// UART_HandleTypeDef huart2;

// /* USER CODE BEGIN 0 */

// /* ===== 7-Segment COMMON ANODE Pin Mapping ===== */
// #define SEG_A_PORT GPIOC
// #define SEG_A_PIN  GPIO_PIN_11

// #define SEG_B_PORT GPIOD
// #define SEG_B_PIN  GPIO_PIN_5

// #define SEG_C_PORT GPIOA
// #define SEG_C_PIN  GPIO_PIN_9

// #define SEG_D_PORT GPIOD
// #define SEG_D_PIN  GPIO_PIN_4

// #define SEG_E_PORT GPIOA
// #define SEG_E_PIN  GPIO_PIN_13

// #define SEG_F_PORT GPIOD
// #define SEG_F_PIN  GPIO_PIN_0

// #define SEG_G_PORT GPIOD
// #define SEG_G_PIN  GPIO_PIN_2

// #define SEG_DP_PORT GPIOC
// #define SEG_DP_PIN  GPIO_PIN_9

// /* ===== Button Configuration ===== */
// #define USER_BUTTON_PORT GPIOA        // Increment button
// #define USER_BUTTON_PIN  GPIO_PIN_0

// #define EXTERNAL_BUTTON_PORT GPIOB    // Decrement button
// #define EXTERNAL_BUTTON_PIN  GPIO_PIN_5

// /* Global variables for counter and button handling */
// int8_t counter = 0;                   // Counter value (0-9)
// uint32_t lastUserButtonPressTime = 0;
// uint32_t lastExternalButtonPressTime = 0;
// #define DEBOUNCE_DELAY 200            // 200ms debounce delay

// /* Function to turn all segments OFF (HIGH for common anode) */
// void Segments_AllOff(void)
// {
//   HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(SEG_DP_PORT, SEG_DP_PIN, GPIO_PIN_SET);
// }

// /* Function to display a single digit (0-9) on 7-segment display */
// void Display_Digit(uint8_t digit)
// {
//   // First turn all segments off
//   Segments_AllOff();
  
//   // For common anode: 0 = ON, 1 = OFF
//   switch(digit)
//   {
//     case 0: // Display 0
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_RESET);
//       // G is OFF
//       break;
      
//     case 1: // Display 1
//       HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 2: // Display 2
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 3: // Display 3
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 4: // Display 4
//       HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 5: // Display 5
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 6: // Display 6
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 7: // Display 7
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 8: // Display 8
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 9: // Display 9
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
//       break;
      
//     default:
//       Segments_AllOff();
//       break;
//   }
// }

// /* Function to check USER button press (PA0 - Active LOW) with debouncing */
// uint8_t UserButton_IsPressed(void)
// {
//   static uint8_t lastButtonState = 1;  // Initialize to HIGH (unpressed state for active LOW)
//   uint8_t currentButtonState = HAL_GPIO_ReadPin(USER_BUTTON_PORT, USER_BUTTON_PIN);
//   uint32_t currentTime = HAL_GetTick();
  
//   // Check if button is pressed (falling edge: HIGH to LOW) and debounce time has passed
//   if (currentButtonState == GPIO_PIN_RESET && lastButtonState == GPIO_PIN_SET)
//   {
//     if ((currentTime - lastUserButtonPressTime) > DEBOUNCE_DELAY)
//     {
//       lastUserButtonPressTime = currentTime;
//       lastButtonState = currentButtonState;
//       return 1; // Button pressed
//     }
//   }
  
//   lastButtonState = currentButtonState;
//   return 0; // Button not pressed or debouncing
// }

// /* Function to check External button press (PB5 - Active HIGH) with debouncing */
// uint8_t ExternalButton_IsPressed(void)
// {
//   static uint8_t lastButtonState = 0;  // Initialize to LOW (unpressed state for active HIGH)
//   uint8_t currentButtonState = HAL_GPIO_ReadPin(EXTERNAL_BUTTON_PORT, EXTERNAL_BUTTON_PIN);
//   uint32_t currentTime = HAL_GetTick();
  
//   // Check if button is pressed (rising edge: LOW to HIGH) and debounce time has passed
//   if (currentButtonState == GPIO_PIN_SET && lastButtonState == GPIO_PIN_RESET)
//   {
//     if ((currentTime - lastExternalButtonPressTime) > DEBOUNCE_DELAY)
//     {
//       lastExternalButtonPressTime = currentTime;
//       lastButtonState = currentButtonState;
//       return 1; // Button pressed
//     }
//   }
  
//   lastButtonState = currentButtonState;
//   return 0; // Button not pressed or debouncing
// }

// /* USER CODE END 0 */

// /* Private function prototypes -----------------------------------------------*/
// void SystemClock_Config(void);
// static void MX_GPIO_Init(void);

// /**
//   * @brief  The application entry point.
//   * @retval int
//   */
// int main(void)
// {
//   /* MCU Configuration--------------------------------------------------------*/
//   HAL_Init();

//   /* Configure the system clock */
//   SystemClock_Config();

//   /* Initialize all configured peripherals */
//   MX_GPIO_Init();

//   /* USER CODE BEGIN 2 */
  
//   // Display initial counter value (0)
//   Display_Digit(counter);
  
//   /* USER CODE END 2 */

//   /* Infinite loop */
//   /* USER CODE BEGIN WHILE */
//   while (1)
//   {
//     // Check if USER button (PA0) is pressed - INCREMENT
//     if (UserButton_IsPressed())
//     {
//       counter++;
      
//       // Wrap around: if counter exceeds 9, reset to 0
//       if (counter > 9)
//       {
//         counter = 0;
//       }
      
//       // Display the updated counter
//       Display_Digit(counter);
//     }
    
//     // Check if External button (PB5) is pressed - DECREMENT
//     if (ExternalButton_IsPressed())
//     {
//       counter--;
      
//       // Wrap around: if counter goes below 0, set to 9
//       if (counter < 0)
//       {
//         counter = 9;
//       }
      
//       // Display the updated counter
//       Display_Digit(counter);
//     }
    
//     /* Small delay to reduce CPU usage */
//     HAL_Delay(10);
    
//     /* USER CODE END WHILE */
//   }
// }

// /**
//   * @brief System Clock Configuration
//   * @retval None
//   */
// void SystemClock_Config(void)
// {
//   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//   /** Initializes the RCC Oscillators according to the specified parameters
//   * in the RCC_OscInitTypeDef structure.
//   */
//   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//   RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//   RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//   {
//     Error_Handler();
//   }

//   /** Initializes the CPU, AHB and APB buses clocks
//   */
//   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                               |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
//   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//   {
//     Error_Handler();
//   }
// }

// /**
//   * @brief GPIO Initialization Function
//   * @param None
//   * @retval None
//   */
// static void MX_GPIO_Init(void)
// {
//   GPIO_InitTypeDef GPIO_InitStruct = {0};

//   /* GPIO Ports Clock Enable */
//   __HAL_RCC_GPIOA_CLK_ENABLE();
//   __HAL_RCC_GPIOB_CLK_ENABLE();
//   __HAL_RCC_GPIOC_CLK_ENABLE();
//   __HAL_RCC_GPIOD_CLK_ENABLE();

//   /* Configure GPIO pin Output Level - Set all HIGH initially (OFF for common anode) */
//   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_13, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9|GPIO_PIN_11, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);

//   /* Configure GPIO pins : GPIOA - Segments C, E (PA9, PA13) */
//   GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_13;
//   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//   /* Configure GPIO pins : GPIOC - Segments A, DP (PC11, PC9) */
//   GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11;
//   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

//   /* Configure GPIO pins : GPIOD - Segments B, D, F, G (PD5, PD4, PD0, PD2) */
//   GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5;
//   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//   HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

//   /* Configure GPIO pin : USER Button (PA0) as INPUT - Active LOW */
//   GPIO_InitStruct.Pin = GPIO_PIN_0;
//   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//   GPIO_InitStruct.Pull = GPIO_PULLUP;  // Pull-up resistor for active LOW button (connected to GND)
//   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//   /* Configure GPIO pin : External Button (PB5) as INPUT - Active HIGH */
//   GPIO_InitStruct.Pin = GPIO_PIN_5;
//   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//   GPIO_InitStruct.Pull = GPIO_PULLDOWN;  // Pull-down resistor for active HIGH button (connected to 3.3V)
//   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//   /* Ensure all segments are OFF initially */
//   Segments_AllOff();
// }

// /* USER CODE BEGIN 4 */

// /* USER CODE END 4 */

// /**
//   * @brief  This function is executed in case of error occurrence.
//   * @retval None
//   */
// void Error_Handler(void)
// {
//   /* USER CODE BEGIN Error_Handler_Debug */
//   /* User can add his own implementation to report the HAL error return state */
//   __disable_irq();
//   while (1)
//   {
//   }
//   /* USER CODE END Error_Handler_Debug */
// }

// #ifdef  USE_FULL_ASSERT
// /**
//   * @brief  Reports the name of the source file and the source line number
//   *         where the assert_param error has occurred.
//   * @param  file: pointer to the source file name
//   * @param  line: assert_param error line source number
//   * @retval None
//   */
// void assert_failed(uint8_t *file, uint32_t line)
// {
//   /* USER CODE BEGIN 6 */
//   /* User can add his own implementation to report the file name and line number,
//      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//   /* USER CODE END 6 */
// }
// #endif /* USE_FULL_ASSERT */


//*********************************************************************** */
//*********************************************************************** */
//*********************************************************************** */
// TASK 2

// /* USER CODE BEGIN Header */
// /**
//   ******************************************************************************
//   * @file           : main.c
//   * @brief          : Task 2 - Display Student ID Digits Using USER Button
//   ******************************************************************************
//   * @attention
//   * Student ID: 10303
//   * 7-Segment Display: COMMON ANODE
//   * USER Button: PA0 (Active LOW - connected between PA0 and GND)
//   * Segment ON  = GPIO LOW  (0)
//   * Segment OFF = GPIO HIGH (1)
//   ******************************************************************************
//   */
// /* USER CODE END Header */

// #include "main.h"

// /* Private variables ---------------------------------------------------------*/
// I2C_HandleTypeDef hi2c1;
// SPI_HandleTypeDef hspi1;
// UART_HandleTypeDef huart2;

// /* USER CODE BEGIN 0 */

// /* ===== 7-Segment COMMON ANODE Pin Mapping ===== */
// #define SEG_A_PORT GPIOC
// #define SEG_A_PIN  GPIO_PIN_11

// #define SEG_B_PORT GPIOD
// #define SEG_B_PIN  GPIO_PIN_5

// #define SEG_C_PORT GPIOA
// #define SEG_C_PIN  GPIO_PIN_9

// #define SEG_D_PORT GPIOD
// #define SEG_D_PIN  GPIO_PIN_4

// #define SEG_E_PORT GPIOA
// #define SEG_E_PIN  GPIO_PIN_13

// #define SEG_F_PORT GPIOD
// #define SEG_F_PIN  GPIO_PIN_0

// #define SEG_G_PORT GPIOD
// #define SEG_G_PIN  GPIO_PIN_2

// #define SEG_DP_PORT GPIOC
// #define SEG_DP_PIN  GPIO_PIN_9

// /* ===== USER Button Configuration ===== */
// #define USER_BUTTON_PORT GPIOA
// #define USER_BUTTON_PIN  GPIO_PIN_0

// /* ===== Student ID Configuration ===== */
// #define STUDENT_ID_LENGTH 5
// uint8_t studentID[STUDENT_ID_LENGTH] = {1,0,3,0,3}; // Student ID: 10303

// /* Global variables for button handling */
// uint8_t currentDigitIndex = 0;
// uint8_t buttonPressed = 0;
// uint32_t lastButtonPressTime = 0;
// #define DEBOUNCE_DELAY 200  

// /* Function to turn all segments OFF (HIGH for common anode) */
// void Segments_AllOff(void)
// {
//   HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(SEG_DP_PORT, SEG_DP_PIN, GPIO_PIN_SET);
// }

// void Display_Digit(uint8_t digit)
// {
//   Segments_AllOff();
  
//   switch(digit)
//   {
//     case 0: 
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 1: // Display 1
//       HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 2: // Display 2
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 3: // Display 3
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 4: // Display 4
//       HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 5: // Display 5
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 6: // Display 6
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 7: // Display 7
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 8: // Display 8
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 9: // Display 9
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
//       break;
      
//     default:
//       Segments_AllOff();
//       break;
//   }
// }

// /* Function to check button press with debouncing */
// uint8_t Button_IsPressed(void)
// {
//   static uint8_t lastButtonState = 1;  // Initialize to HIGH (unpressed state for active LOW)
//   uint8_t currentButtonState = HAL_GPIO_ReadPin(USER_BUTTON_PORT, USER_BUTTON_PIN);
//   uint32_t currentTime = HAL_GetTick();
  
//   // Check if button is pressed (falling edge: HIGH to LOW) and debounce time has passed
//   if (currentButtonState == GPIO_PIN_RESET && lastButtonState == GPIO_PIN_SET)
//   {
//     if ((currentTime - lastButtonPressTime) > DEBOUNCE_DELAY)
//     {
//       lastButtonPressTime = currentTime;
//       lastButtonState = currentButtonState;
//       return 1; // Button pressed
//     }
//   }
  
//   lastButtonState = currentButtonState;
//   return 0; // Button not pressed or debouncing
// }

// /* USER CODE END 0 */

// /* Private function prototypes -----------------------------------------------*/
// void SystemClock_Config(void);
// static void MX_GPIO_Init(void);

// /**
//   * @brief  The application entry point.
//   * @retval int
//   */
// int main(void)
// {
//   /* MCU Configuration--------------------------------------------------------*/
//   HAL_Init();

//   /* Configure the system clock */
//   SystemClock_Config();

//   /* Initialize all configured peripherals */
//   MX_GPIO_Init();

//   /* USER CODE BEGIN 2 */
  
//   // Display the first digit of student ID initially
//   Display_Digit(studentID[currentDigitIndex]);
  
//   /* USER CODE END 2 */

//   /* Infinite loop */
//   /* USER CODE BEGIN WHILE */
//   while (1)
//   {
//     // Check if button is pressed
//     if (Button_IsPressed())
//     {
//       // Move to next digit
//       currentDigitIndex++;
      
//       // Reset to first digit after last digit
//       if (currentDigitIndex >= STUDENT_ID_LENGTH)
//       {
//         currentDigitIndex = 0;
//       }
      
//       // Display the current digit
//       Display_Digit(studentID[currentDigitIndex]);
//     }
    
//     /* Small delay to reduce CPU usage */
//     HAL_Delay(10);
    
//     /* USER CODE END WHILE */
//   }
// }

// /**
//   * @brief System Clock Configuration
//   * @retval None
//   */
// void SystemClock_Config(void)
// {
//   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//   /** Initializes the RCC Oscillators according to the specified parameters
//   * in the RCC_OscInitTypeDef structure.
//   */
//   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//   RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//   RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//   {
//     Error_Handler();
//   }

//   /** Initializes the CPU, AHB and APB buses clocks
//   */
//   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                               |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
//   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//   {
//     Error_Handler();
//   }
// }

// /**
//   * @brief GPIO Initialization Function
//   * @param None
//   * @retval None
//   */
// static void MX_GPIO_Init(void)
// {
//   GPIO_InitTypeDef GPIO_InitStruct = {0};

//   /* GPIO Ports Clock Enable */
//   __HAL_RCC_GPIOA_CLK_ENABLE();
//   __HAL_RCC_GPIOC_CLK_ENABLE();
//   __HAL_RCC_GPIOD_CLK_ENABLE();

//   /* Configure GPIO pin Output Level - Set all HIGH initially (OFF for common anode) */
//   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_13, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9|GPIO_PIN_11, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);

//   /* Configure GPIO pins : GPIOA - Segments C, E (PA9, PA13) */
//   GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_13;
//   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//   /* Configure GPIO pins : GPIOC - Segments A, DP (PC11, PC9) */
//   GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11;
//   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

//   /* Configure GPIO pins : GPIOD - Segments B, D, F, G (PD5, PD4, PD0, PD2) */
//   GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5;
//   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//   HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

//   /* Configure GPIO pin : USER Button (PA0) as INPUT */
//   GPIO_InitStruct.Pin = GPIO_PIN_0;
//   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//   GPIO_InitStruct.Pull = GPIO_PULLUP;  // Pull-up resistor for active LOW button (connected to GND)
//   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//   /* Ensure all segments are OFF initially */
//   Segments_AllOff();
// }

// /* USER CODE BEGIN 4 */

// /* USER CODE END 4 */

// /**
//   * @brief  This function is executed in case of error occurrence.
//   * @retval None
//   */
// void Error_Handler(void)
// {
//   /* USER CODE BEGIN Error_Handler_Debug */
//   /* User can add his own implementation to report the HAL error return state */
//   __disable_irq();
//   while (1)
//   {
//   }
//   /* USER CODE END Error_Handler_Debug */
// }

// #ifdef  USE_FULL_ASSERT
// /**
//   * @brief  Reports the name of the source file and the source line number
//   *         where the assert_param error has occurred.
//   * @param  file: pointer to the source file name
//   * @param  line: assert_param error line source number
//   * @retval None
//   */
// void assert_failed(uint8_t *file, uint32_t line)
// {
//   /* USER CODE BEGIN 6 */
//   /* User can add his own implementation to report the file name and line number,
//      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//   /* USER CODE END 6 */
// }
// #endif /* USE_FULL_ASSERT */



//************************************************************************************ */
//************************************************************************************ */
//************************************************************************************ */
// TASK 1:
/* USER CODE BEGIN Header */
// /**
//   ******************************************************************************
//   * @file           : main.c
//   * @brief          : Main program body - Hexadecimal Counter (0-F) on 7-Segment
//   ******************************************************************************
//   * @attention
//   * 7-Segment Display: COMMON ANODE
//   * Segment ON  = GPIO LOW  (0)
//   * Segment OFF = GPIO HIGH (1)
//   ******************************************************************************
//   */
// /* USER CODE END Header */

// #include "main.h"

// /* Private variables ---------------------------------------------------------*/
// I2C_HandleTypeDef hi2c1;
// SPI_HandleTypeDef hspi1;
// UART_HandleTypeDef huart2;

// /* USER CODE BEGIN 0 */

// /* ===== 7-Segment COMMON ANODE Pin Mapping ===== */
// #define SEG_A_PORT GPIOC
// #define SEG_A_PIN  GPIO_PIN_11

// #define SEG_B_PORT GPIOD
// #define SEG_B_PIN  GPIO_PIN_5

// #define SEG_C_PORT GPIOA
// #define SEG_C_PIN  GPIO_PIN_9

// #define SEG_D_PORT GPIOD
// #define SEG_D_PIN  GPIO_PIN_4

// #define SEG_E_PORT GPIOA
// #define SEG_E_PIN  GPIO_PIN_13

// #define SEG_F_PORT GPIOD
// #define SEG_F_PIN  GPIO_PIN_0

// #define SEG_G_PORT GPIOD
// #define SEG_G_PIN  GPIO_PIN_2

// #define SEG_DP_PORT GPIOC
// #define SEG_DP_PIN  GPIO_PIN_9

// /* Function to turn all segments OFF (HIGH for common anode) */
// void Segments_AllOff(void)
// {
//   HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(SEG_DP_PORT, SEG_DP_PIN, GPIO_PIN_SET);
// }

// /* Function to display a hexadecimal digit (0-F) on 7-segment display */
// void Display_Hex(uint8_t digit)
// {
//   // First turn all segments off
//   Segments_AllOff();
  
//   // For common anode: 0 = ON, 1 = OFF
//   switch(digit)
//   {
//     case 0x0: // Display 0
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_RESET);
//       // G is OFF
//       break;
      
//     case 0x1: // Display 1
//       HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 0x2: // Display 2
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 0x3: // Display 3
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 0x4: // Display 4
//       HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 0x5: // Display 5
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 0x6: // Display 6
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 0x7: // Display 7
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 0x8: // Display 8
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 0x9: // Display 9
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 0xA: // Display A
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 0xB: // Display b
//       HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 0xC: // Display C
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 0xD: // Display d
//       HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 0xE: // Display E
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_RESET);
//       break;
      
//     case 0xF: // Display F
//       HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_RESET);
//       HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_RESET);
//       break;
      
//     default:
//       Segments_AllOff();
//       break;
//   }
// }

// /* USER CODE END 0 */

// /* Private function prototypes -----------------------------------------------*/
// void SystemClock_Config(void);
// static void MX_GPIO_Init(void);

// /**
//   * @brief  The application entry point.
//   * @retval int
//   */
// int main(void)
// {
//   /* MCU Configuration--------------------------------------------------------*/
//   HAL_Init();

//   /* Configure the system clock */
//   SystemClock_Config();

//   /* Initialize all configured peripherals */
//   MX_GPIO_Init();

//   /* USER CODE BEGIN 2 */
//   uint8_t counter = 0;
  
//   /* USER CODE END 2 */

//   /* Infinite loop */
//   /* USER CODE BEGIN WHILE */
//   while (1)
//   {
//     /* Display current hexadecimal digit (0-F) */
//     Display_Hex(counter);
    
//     /* Wait for 2 seconds */
//     HAL_Delay(2000);
    
//     /* Increment counter */
//     counter++;
    
//     /* Reset counter after F (15) */
//     if(counter > 0xF)
//     {
//       counter = 0;
//     }
    
//     /* USER CODE END WHILE */
//   }
// }

// /**
//   * @brief System Clock Configuration
//   * @retval None
//   */
// void SystemClock_Config(void)
// {
//   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//   /** Initializes the RCC Oscillators according to the specified parameters
//   * in the RCC_OscInitTypeDef structure.
//   */
//   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//   RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//   RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//   {
//     Error_Handler();
//   }

//   /** Initializes the CPU, AHB and APB buses clocks
//   */
//   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                               |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
//   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//   {
//     Error_Handler();
//   }
// }

// /**
//   * @brief GPIO Initialization Function
//   * @param None
//   * @retval None
//   */
// static void MX_GPIO_Init(void)
// {
//   GPIO_InitTypeDef GPIO_InitStruct = {0};

//   /* GPIO Ports Clock Enable */
//   __HAL_RCC_GPIOA_CLK_ENABLE();
//   __HAL_RCC_GPIOC_CLK_ENABLE();
//   __HAL_RCC_GPIOD_CLK_ENABLE();

//   /* Configure GPIO pin Output Level - Set all HIGH initially (OFF for common anode) */
//   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_13, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9|GPIO_PIN_11, GPIO_PIN_SET);
//   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);

//   /* Configure GPIO pins : GPIOA - Segments C, E */
//   GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_13;
//   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//   /* Configure GPIO pins : GPIOC - Segments A, DP */
//   GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11;
//   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

//   /* Configure GPIO pins : GPIOD - Segments B, D, F, G */
//   GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5;
//   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//   HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

//   /* Ensure all segments are OFF initially */
//   Segments_AllOff();
// }

// /* USER CODE BEGIN 4 */

// /* USER CODE END 4 */

// /**
//   * @brief  This function is executed in case of error occurrence.
//   * @retval None
//   */
// void Error_Handler(void)
// {
//   /* USER CODE BEGIN Error_Handler_Debug */
//   /* User can add his own implementation to report the HAL error return state */
//   __disable_irq();
//   while (1)
//   {
//   }
//   /* USER CODE END Error_Handler_Debug */
// }

// #ifdef  USE_FULL_ASSERT
// /**
//   * @brief  Reports the name of the source file and the source line number
//   *         where the assert_param error has occurred.
//   * @param  file: pointer to the source file name
//   * @param  line: assert_param error line source number
//   * @retval None
//   */
// void assert_failed(uint8_t *file, uint32_t line)
// {
//   /* USER CODE BEGIN 6 */
//   /* User can add his own implementation to report the file name and line number,
//      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//   /* USER CODE END 6 */
// }
// #endif /* USE_FULL_ASSERT */