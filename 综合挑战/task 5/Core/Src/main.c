/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_OFF 0
#define LED_LOW 25
#define LED_MEDIUM 75
#define LED_HIGH 100

#define LED_Breath 4
#define LED_Strink 5

#define BUTTON_IDLE 0
#define BUTTON_SINGLE_CLICK 1
#define BUTTON_DOUBLE_CLICK 2
#define BUTTON_LONG_PRESS 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t led_brightness = LED_OFF;
uint16_t button_state = BUTTON_IDLE;

uint16_t g_Time_Count;		/*TIM溢出次数 -- 10ms  	Tout(溢出时间) = (ARR+1)(PSC+1)/Tclk(时钟分割)*/
uint8_t g_Key_Number=0;
uint32_t g_Time_Temp;
uint32_t g_Time_Last;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t get_buttonstate()
{
	return  button_state;
}
uint16_t get_led_brightness()
{
	return  led_brightness;
}
void start_breathing_mode(void);

void start_blinking_mode(uint32_t period);

void LED_SetBrightness(uint16_t brightness) 
{
    /* 设置PWM占空比来控制LED亮度*/
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, brightness);
}

void LED_Brightness_init()
{
	__HAL_TIM_SET_AUTORELOAD(&htim2,720-1);
	__HAL_TIM_SET_PRESCALER(&htim2,100-1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,0);	
}

void start_breathing_mode(void)
{

	for(uint16_t i=0; i<100;i++)
	{
		LED_SetBrightness(i);
		HAL_Delay(25);
	}	
	for(uint16_t i=0; i<100;i++)
	{
		LED_SetBrightness(100-i);
		HAL_Delay(25);
	}	
}
void start_blinking_mode(uint32_t period)
{
	uint16_t arr=10000*period;
	uint16_t psc=7200;   
	__HAL_TIM_SET_AUTORELOAD(&htim2,arr-1);
	__HAL_TIM_SET_PRESCALER(&htim2,psc-1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,arr/2);
}

void scan()
{
	//static uint8_t long_click_cnt = 0;
	static uint8_t short_click_cnt = 0;
	switch(button_state)
	{
		case BUTTON_SINGLE_CLICK:	 
		{
			if(led_brightness==LED_Breath)
				
			{
				led_brightness=LED_OFF;
				LED_SetBrightness(0);
				button_state=0;
			}
			else if(led_brightness==LED_Strink)
			{
				led_brightness=LED_OFF;
				LED_SetBrightness(0);
				button_state=0;
				LED_Brightness_init();
			}				
			else
			{
					short_click_cnt++;
					short_click_cnt = short_click_cnt%4;
					/* 根据按键的次数依次执行不同的动作 */
					switch(short_click_cnt)
					{
						case 0:		   LED_SetBrightness(0); led_brightness=0; //button_state=0;
										button_state=0;
								
						break;
						
						case 1: 	   LED_SetBrightness(25); led_brightness=25; button_state=0;
							
						break;
						
						case 2:		   LED_SetBrightness(75); led_brightness=75; button_state=0;
								
						break;
						
						case 3:		   LED_SetBrightness(100);led_brightness=100; button_state=0;
								
						break;
						
						default:break;
					}		
				}
			}
			break;
	
		case BUTTON_LONG_PRESS:			 /*长按按键*/
		{
		
		  short_click_cnt=0;
		  uint32_t time=g_Time_Last;
		  start_blinking_mode(time);
		  led_brightness=LED_Strink;
			
		}
		break;
		
		case BUTTON_DOUBLE_CLICK:			/*双击按键*/
		{
		  short_click_cnt=0;
		  led_brightness=LED_Breath;
		  start_breathing_mode();
			
		}
		break;
		
		
		default:	
		break;
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);/*放在外面不会反复开启*/
	LED_SetBrightness(0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	   button_state =get_buttonstate();
	   led_brightness=get_led_brightness();
	   scan();
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
