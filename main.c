/**
  ******************************************************************************
  * @file    Templates/Src/main.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    29-April-2016
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lcd.h"
#include <time.h>
#include <stdio.h>

/** @addtogroup STM32F1xx_HAL_Examples
  * @{
  */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/** @addtogroup Templates
  * @{
  */


ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart1;
static GPIO_InitTypeDef  GPIO_InitStruct;

UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim1;

void SystemClock_Config(void);
void draw_base(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
void Error_Handler(void);
void drawi();
int eliminate(int);
void qingqiang(int, int, int);  //clean
int rozhao(int); //rotate

void cum();

static void MX_TIM1_Init(void);
//static void MX_FSMC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

unsigned char String1[]={22};
unsigned char String2[]={23};
unsigned char String3[]={24};
unsigned char String4[]={25};
unsigned char String5[]={26};
unsigned char String6[]={27};
unsigned char String7[]={28};
unsigned char String8[]={29};
unsigned char String9[]={30};
unsigned char String10[]={31};
unsigned char String11[]={32};
unsigned char String12[]={33};


void draw(int xinit, int yinit, int type);
int collanpa(int, int, int);
void printall();
int testicle(int);
unsigned int a;
unsigned int x, y;

unsigned int speed;
unsigned int deltx, delty;


unsigned int px, py; 

int surprise = 0;
int number[] = {0, 1, 3, 7, 11, 15, 17};
int size = 6;
int base[12][12] = {0};									
//11&8
int tetr[19][4][4]={{{0,0,1,1},{0,0,1,1},{0,0,0,0},{0,0,0,0}},//O       0
									  {{1,1,1,1},{0,0,0,0},{0,0,0,0},{0,0,0,0}},//I1      1      
									  {{0,0,0,1},{0,0,0,1},{0,0,0,1},{0,0,0,1}},//I2      2
									  {{0,0,0,1},{0,0,1,1},{0,0,0,1},{0,0,0,0}},//T1      3
									  {{0,1,1,1},{0,0,1,0},{0,0,0,0},{0,0,0,0}},//T2      4
									  {{0,0,1,0},{0,0,1,1},{0,0,1,0},{0,0,0,0}},//T3      5
									  {{0,0,1,0},{0,1,1,1},{0,0,0,0},{0,0,0,0}},//T4      6
									  {{0,0,1,1},{0,0,0,1},{0,0,0,1},{0,0,0,0}},//L1      7
									  {{0,1,1,1},{0,1,0,0},{0,0,0,0},{0,0,0,0}},//L2      8
									  {{0,0,1,0},{0,0,1,0},{0,0,1,1},{0,0,0,0}},//L3      9
									  {{0,0,0,1},{0,1,1,1},{0,0,0,0},{0,0,0,0}},//L4      10
									  {{0,0,0,1},{0,0,0,1},{0,0,1,1},{0,0,0,0}},//J1      11
									  {{0,1,1,1},{0,0,0,1},{0,0,0,0},{0,0,0,0}},//J2      12
									  {{0,0,1,1},{0,0,1,0},{0,0,1,0},{0,0,0,0}},//J3      13
									  {{0,1,0,0},{0,1,1,1},{0,0,0,0},{0,0,0,0}},//J4      14      
									  {{0,0,0,1},{0,0,1,1},{0,0,1,0},{0,0,0,0}},//S1      15     
									  {{0,1,1,0},{0,0,1,1},{0,0,0,0},{0,0,0,0}},//S2      16
									  {{0,0,1,0},{0,0,1,1},{0,0,0,1},{0,0,0,0}},//Z1      17
									  {{0,0,1,1},{0,1,1,0},{0,0,0,0},{0,0,0,0}}};//Z2     18
									
			

int main(void)
{
	
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
	LCD_Init();
	LCD_Clear();
	
  
  /* -1- Enable each GPIO Clock (to be able to program the configuration registers) */
	////////////////////////////////////////////////////////////////////////////////////
  LED1_GPIO_CLK_ENABLE();
  LED2_GPIO_CLK_ENABLE();
  LED3_GPIO_CLK_ENABLE();
  LED4_GPIO_CLK_ENABLE();

  /* -2- Configure IOs in output push-pull mode to drive external LEDs */
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStruct.Pin = LED1_PIN;
  HAL_GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED2_PIN;
  HAL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED3_PIN;
  HAL_GPIO_Init(LED3_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED4_PIN;
  HAL_GPIO_Init(LED4_GPIO_PORT, &GPIO_InitStruct);
	
	////////////////////////////////////////////////////////////////////////////////////
  
	WAKEUP_BUTTON_GPIO_CLK_ENABLE();
	////////////////////////////////////////////////////////////////////////////////////
	GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	
	GPIO_InitStruct.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);//wake turn left
	////////////////////////////////////////////////////////////////////////////////////
	TAMPER_BUTTON_GPIO_CLK_ENABLE();
	GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	
	GPIO_InitStruct.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);//temper right
	////////////////////////////////////////////////////////////////////////////////////
  __HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	
	GPIO_InitStruct.Pin = GPIO_PIN_10;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);//key   rotate
	
	HAL_TIM_Base_Start_IT(&htim1);
	
	int type=0;
	px=4;
	py=-1;
	int i = 0;
	
	////////////////////////////////////////////////////////////////////////////////////
  while (1)
  {
		if(a > 0 && a<1365){
		speed=1500;
			
		}else if(a>= 1366 && a <= 2730){
		speed = 1000;
		}else{
		speed = 500;
		}					
		
		if(collanpa(px, py, type)){
				drawi(px, py, type);
				cum();
				cum();
				cum();
			  cum();
			if(surprise >= 1){
				LCD_Clear();
					LCD_DrawString(18, 22+(8*0) , String6,sizeof(String6));
				LCD_DrawString(18, 22+(8*1) , String7,sizeof(String7));
				LCD_DrawString(18, 22+(8*2) , String8,sizeof(String8));
				LCD_DrawString(18, 22+(8*3) , String9,sizeof(String9));
				LCD_DrawString(18, 22+(8*4) , String10,sizeof(String10));
				LCD_DrawString(18, 22+(8*5) , String11,sizeof(String11));
				LCD_DrawString(18, 22+(8*6) , String12,sizeof(String12));
				return 0 ;
			}
			
				printall();
			  HAL_Delay(100);
				px = 4;
				py = -1;
				if(i == size){
						i = 0;
			 }
			 type = number[i++]; 

			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_6);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_7);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_8);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_9);
			HAL_Delay(30);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_6);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_7);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_8);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_9);
			HAL_Delay(30);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_6);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_7);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_8);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_9);
			HAL_Delay(30);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_6);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_7);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_8);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_9);
			  continue;
		}
		if(py==4){
			drawi(px, py, type);
			cum();
			cum();
			cum();
			printall();
			
			if(surprise >= 4){
				LCD_Clear();
					LCD_DrawString(18, 22+(8*0) , String6,sizeof(String6));
				LCD_DrawString(18, 22+(8*1) , String7,sizeof(String7));
				LCD_DrawString(18, 22+(8*2) , String8,sizeof(String8));
				LCD_DrawString(18, 22+(8*3) , String9,sizeof(String9));
				LCD_DrawString(18, 22+(8*4) , String10,sizeof(String10));
				LCD_DrawString(18, 22+(8*5) , String11,sizeof(String11));
				LCD_DrawString(18, 22+(8*6) , String12,sizeof(String12));
				return 0;
			}
		 	HAL_Delay(800);
			px=4;
			py=-1;
			//i =0;
			if(i == size){
					i = 0;
			}
			type = number[i++]; 
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_6);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_7);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_8);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_9);
			HAL_Delay(30);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_6);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_7);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_8);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_9);
			HAL_Delay(30);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_6);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_7);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_8);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_9);
			HAL_Delay(30);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_6);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_7);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_8);
			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_9);
			continue;
		} 
		HAL_ADC_Start(&hadc1);
		a = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

		drawi(px, py, type);
		
		printall();
		HAL_Delay(speed);

		qingqiang(px, py, type);
		

		testicle(type);
		if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)){
			type = rozhao(type);
			HAL_Delay(30);
		}	
		LCD_Clear();		
	} 
}

void cum(){    //clear cum
	
	int test[12][12];
	int count = 0;	
	int full[12] = {0};
	
	for(int j = 0; j < 12; j++ ){ //put base in the array //put in main? //copy
		for(int i = 0; i < 12; i++){
			test[i][j] =0;	
		} 
	}
	for(int j = 0; j < 12; j++ ){ //put base in the array //put in main? //copy
		for(int i = 0; i < 12; i++){
			test[i][j] = base[i][j];
		}  
	}
	for(int j = 0; j < 12; j++ ){ //count the full row //////scan the full 1 row
		for(int i = 0; i < 12; i++){
			if(base[i][j]==1){
				count = count + 1;
			}
		}
		if(count >= 11){ //the full row is 1
			full[j]=1;
			surprise++;
		}
		count = 0;
	}
	for(int j = 11; j >= 0; j-- ){ //buttom to top the whole row, up row replace the buttom	
		if(full[j]==1){
			for(int q = j; q >= 0 ; q--){ //let the upper row get down
				for(int k = 0; k < 12; k++){
					
					base[k][q] = 0;	
					base[k][q] = test[k][q-1];
					base[k][q-1] = 0;
					
				}
			}
		}		
	}	
}



int testicle(int type){
	
	  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)==0){
			 px++;
			 HAL_Delay(30);
		}
		
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){
			 px--;
			 //HAL_Delay(30);
		}
		
		if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)){
			type = rozhao(type);
			HAL_Delay(30);
		}else{
			py++;
		}
		
	return 0;
}	


int rozhao(int type){
	if(type == 0){		
	}else if(type == 1){
		type = 2;
	}else if(type == 2){
		type = 1;
	}else if(type == 3){
		type = 4;
	}else if(type == 4){
		type = 5;
	}else if(type == 5){
		type = 6;
	}else if(type == 6){
		type = 3;
	}else if(type == 7){
		type = 8;
	}else if(type == 8){
		type = 9;
	}else if(type == 9){
		type = 10;
	}else if(type == 10){
		type = 7;
	}else if(type == 11){
		type = 12;
	}else if(type == 12){
		type = 13;
	}else if(type == 13){
		type = 14;
	}else if(type == 14){
		type = 11;
	}else if(type == 15){
		type = 16;
	}else if(type == 16){
		type = 15;
	}else if(type == 17){
		type = 18;
	}else if(type == 18){
		type = 17;
	}
	return type;
}

int collanpa(int init_x, int init_y, int type){
		for(int i = 0; i <4  ; i++){
			for(int j = 0; j < 4; j++){    //need to change
					 if(base[i+init_x][j+init_y+1] ==1 && tetr[type][i][j] ==1 ){
							return 1;
				 }
			}
	 }
	 return 0;
}


void qingqiang(int init_x, int init_y,int type){  //clean

	 for(int i = init_x; i < init_x+4; i++){
			for(int j = init_y; j < init_y+4; j++){
					  if(base[i][j] ==1 && tetr[type][i-init_x][j-init_y] ==1 ){
							base[i][j]=0;
				 }
			}
	 }
}


void drawi(int init_x, int init_y,int type){
	for(int i = init_x; i < init_x+4; i++){
			for(int j = init_y; j < init_y+4; j++){
				 if(base[i][j] ==0 && tetr[type][i-init_x][j-init_y] ==1 ){
					 base[i][j]=1;
				 }
			}
	}
		
}
/** System Clock Configuration
*/
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
	
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);

  return ch;
}

void printall()//print base
	{
		LCD_Clear();
		int x=16;
		int y=22;
		//LCD_Clear();
		for(int i = 0; i < 11 ; i++){
			for(int j = 0; j < 10; j++ ){
				if(base[i][j] == 1){
					LCD_DrawString(x+j, y+(8*i) , String1,sizeof(String1));
					
				}
			}
			
		}
		
		LCD_DrawString(x, y-8 , String5,sizeof(String5));
		LCD_DrawString(x, y+(8*11) , String4,sizeof(String4));
		LCD_DrawString(x+2, y-8 , String5,sizeof(String5));
		LCD_DrawString(x+2, y+(8*11) , String4,sizeof(String4));
		LCD_DrawString(x+4, y-8 , String5,sizeof(String5));
		LCD_DrawString(x+4, y+(8*11) , String4,sizeof(String4));
		LCD_DrawString(x+6, y-8 , String5,sizeof(String5));
		LCD_DrawString(x+6, y+(8*11) , String4,sizeof(String4));
		LCD_DrawString(x+8, y-8 , String5,sizeof(String5));
		LCD_DrawString(x+8, y+(8*11) , String4,sizeof(String4));
		LCD_DrawString(x+10, y-8 , String5,sizeof(String5));
		LCD_DrawString(x+10, y+(8*11) , String4,sizeof(String4));
		
		
	
	return;
}


void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}


void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7200;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1500;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim1);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  HAL_TIM_SlaveConfigSynchronization(&htim1, &sSlaveConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

}


/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
	__GPIOA_CLK_ENABLE();
  __GPIOE_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PF6 PF7 PF8 PF9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */


void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}


int eliminate(int y){							//determine eliminate
	for(int x = 0 ; x < 11; x++){
				if(base[x][y] != 1){
						return 0;
				}				
		}
	return 1;
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
