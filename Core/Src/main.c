/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "obraz.c"
#include "panel.h"
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

/* USER CODE BEGIN PV */

uint8_t flaga=0;

uint8_t bufor[6][4][768];
uint8_t bufferSPI[4][48];

uint8_t przejsciowy[6][6144];

uint8_t pole=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void PANEL (uint8_t surface, uint8_t scena);

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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI1_Init();
  MX_DMA_Init();
  MX_TIM1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);



  //    PINOUT
  //    SIN R1 -- G1
  //  	  B1 -- GND
  //  	  R2 -- G2
  //  	  B2 -- GND
  //  	   A -- B
  //  	   C -- D
  //  	 CLK -- LAT
  //  	  OE -- GND






  //przejsciowy[0][6144] = sonic[18432];

  for (int g=0,d=48; g<18432; g+=576,d+=192)
  {
	  for (int f=0,k=0; f<6; f++,k+=96)
	  	  {
	  		  for (int z=0; z<96; z++)
	  		  {
	  			  przejsciowy[f][z+d] = sonic[z+k+g];
	  		  }
	  	  }
  }







for (int scena=0; scena<6; scena++)
{
  for (int k=0; k<4; k++) 				//4 przejścia dla 4 najbardziej znaczących bitów
  {
  	for (int m=0; m<32; m++) 			//mam 32 linie do przepisania
  	{
  		for (int p=0; p<3; p++)			//tu robimy to samo dla trzech kolorów osobno R G B
  	  	{
  			for (int n=0; n<8; n++) 	//tym zapisuję do bufora dane do pierwszej linii dla pajważniejszych pixeli bo (1<<7)
  	  		{
  	  			bufor[scena][k][p*8 + n + m*24] =  	(( przejsciowy[scena] [p + 3*8*n + 192*m + 3*0]  &  (1<<(7-k)) )<< k) >>(0) |
  									 	 			(( przejsciowy[scena] [p + 3*8*n + 192*m + 3*1]  &  (1<<(7-k)) )<< k) >>(1) |
													(( przejsciowy[scena] [p + 3*8*n + 192*m + 3*2]  &  (1<<(7-k)) )<< k) >>(2) |
													(( przejsciowy[scena] [p + 3*8*n + 192*m + 3*3]  &  (1<<(7-k)) )<< k) >>(3) |
													(( przejsciowy[scena] [p + 3*8*n + 192*m + 3*4]  &  (1<<(7-k)) )<< k) >>(4) |
													(( przejsciowy[scena] [p + 3*8*n + 192*m + 3*5]  &  (1<<(7-k)) )<< k) >>(5) |
													(( przejsciowy[scena] [p + 3*8*n + 192*m + 3*6]  &  (1<<(7-k)) )<< k) >>(6) |
													(( przejsciowy[scena] [p + 3*8*n + 192*m + 3*7]  &  (1<<(7-k)) )<< k) >>(7) ;
  	  		}
  	  	}
  	}
  }
 }













  void PANEL (uint8_t surface, uint8_t scena)
  	{

  		  for (int32_t i=0; i<16; i++)		// 'i' mean ROW to send for all picture
  			  {

  			  for (int32_t j=0; j<8; j++)	//'j' mean BYTES to sent in each ROW - lower and upper in one time
  			  {
  				  //for upper part picture
  				bufferSPI[surface][ 0+j] = bufor[scena][surface][(i+ 0)*24 + 16 + (7-j)];	//line R
  				bufferSPI[surface][16+j] = bufor[scena][surface][(i+ 0)*24 +  8 + (7-j)];	//line G
  				bufferSPI[surface][32+j] = bufor[scena][surface][(i+ 0)*24 +  0 + (7-j)];	//line B

  				  //for lower part picture
  				bufferSPI[surface][ 8+j] = bufor[scena][surface][(i+16)*24 + 16 + (7-j)];	//line R
  				bufferSPI[surface][24+j] = bufor[scena][surface][(i+16)*24 +  8 + (7-j)];	//line G
  				bufferSPI[surface][40+j] = bufor[scena][surface][(i+16)*24 +  0 + (7-j)];	//line B
  			  }


  			  HAL_SPI_Transmit(&hspi1, bufferSPI[surface], 48,2); //dla funkcji blokującej potrzeba minimum 2 ms aby wysłać ramkę
  			   //HAL_SPI_Transmit_DMA(&hspi1, Bufor[surface], 48); //z dma trzeba popracować nad wywołaniem funkcji od przerwania dma po zakończeniu transmisji

  			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12,1); // OE  - 0 włączanie zasilania ledów - np do sterowania poziomem jasności
  			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0); // LAT - 1 PRZERUTNIK  dane z rejestru na ledy //sprawdzic czy oby na pewno 1
  			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1); // LAT - 1 PRZERUTNIK  dane z rejestru na ledy //sprawdzic czy oby na pewno 1
  			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0); // LAT - 1 PRZERUTNIK  dane z rejestru na ledy //sprawdzic czy oby na pewno 1

  			  GPIOD->ODR = (15-i); //wykorzystanie bitów 0,1,2,3 portu GPIOD aby zmniejeszyć czasy wykonania przełączania linii

  			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12,0); // OE  - 0 włączanie zasilania ledów - np do sterowania poziomem jasności
  			  }
  }




  uint32_t czas = HAL_GetTick();
  uint32_t a=0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	 PANEL (0,a);
	 PANEL (0,a);
	 PANEL (1,a);
	 PANEL (0,a);

	 PANEL (2,a);

	 PANEL (0,a);
	 PANEL (1,a);
	 PANEL (0,a);

	 PANEL (3,a);

	 PANEL (0,a);
	 PANEL (1,a);
	 PANEL (0,a);
	 PANEL (0,a);

	 PANEL (2,a);

	 PANEL (1,a);
	 PANEL (0,a);
	 PANEL (0,a);
	 PANEL (1,a);


	   if ((HAL_GetTick() - czas) > 70)
	   {
		   czas = HAL_GetTick();
		   if(a>=5){a=0;}

		   a++;
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* SPI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(SPI1_IRQn);
  /* TIM1_UP_TIM10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

/* USER CODE BEGIN 4 */


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	if (flaga > 15) {flaga=1;}
	else flaga++;
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
