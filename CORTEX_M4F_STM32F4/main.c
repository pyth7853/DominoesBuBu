/**
  ******************************************************************************
  * @file    Template/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    20-September-2013
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "draw_graph.h"
#include "move_car.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_tim.h"

//Library config for this project!!!!!!!!!!!
#include "stm32f4xx_conf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
xQueueHandle t_queue; /* Traffic light queue. */
xQueueHandle t_mutex; /* Traffic light mutex. */

const int p_scale = 1;
const int period = 1680 ;
const int prescalar = 1000 ;
const int T4full = 1680 * 2 ;
const int T3full = 1680 ;
const int T2full = 1680 ;
const int T5full = 1680 ;
const float s = 0.6 ;

static char cmd='#';

void bubuGo(void){
    //TIM4->CCR1=((period/2.5)*0.825)/p_scale; 
    //TIM3->CCR1=((period/3)*0.825)/p_scale; 
    TIM4->CCR1=T4full;
    TIM3->CCR1=T3full;
//    TIM2->CCR2=T2full;
//    TIM5->CCR2=T5full;
}

void bubuStop(void){
    TIM4->CCR1=0;
    TIM3->CCR1=0; 
}

void bubuLeftfront(void){
    TIM4->CCR1=T4full;
    TIM3->CCR1=T3full*s; 
}

void bubuLeft(void){
    TIM4->CCR1=T4full;
    TIM3->CCR1=T3full*s*s; 
}

void bubuRightfront(void){
    TIM4->CCR1=T4full*s*s;
    TIM3->CCR1=T3full; 
}
void bubuRight(void){
    TIM4->CCR1=T4full*s*s*s;
    TIM3->CCR1=T3full; 
}
void USART1_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;

    /* USARTx configuration ------------------------------------------------------*/
    /* USARTx configured as follow:
     *  - BaudRate = 9600 baud
     *  - Word Length = 8 Bits
     *  - One Stop Bit
     *  - No parity
     *  - Hardware flow control disabled (RTS and CTS signals)
     *  - Receive and transmit enabled
     */
    USART_InitStructure.USART_BaudRate = 57600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}

void RCC_Configuration(void)
{
      /* --------------------------- System Clocks Configuration -----------------*/
      /* USART1 clock enable */
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
      /* GPIOA clock enable */
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
}

void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /*-------------------------- GPIO Configuration ----------------------------*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Connect USART pins to AF */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);   // USART1_TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);  // USART1_RX
}

void RCC_Configuration4(void)
{
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB , ENABLE );//Enalbe AHB for GPIOB
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4, ENABLE );//Enable APB for TIM4
}

void RCC_Configuration3(void)
{
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB , ENABLE );//Enalbe AHB for GPIOB
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, ENABLE );//Enable APB for TIM3
}

void RCC_Configuration2(void)
{
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB , ENABLE );//Enalbe AHB for GPIOB
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );//Enable APB for TIM2
}

/**
  * @brief  configure the PD12~15 to Timers
  * @param  None
  * @retval None
  */
  
void GPIO_Configuration4(void){
	GPIO_InitTypeDef GPIO_InitStructure;//Create GPIO_InitStructure 
	GPIO_StructInit(&GPIO_InitStructure); // Reset GPIO_structure
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4); 
	// set GPIOD_Pin12 to AF_TIM4
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            
	// Alt Function - Push Pull
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOB, &GPIO_InitStructure );  
}
    
void GPIO_Configuration3(void){
	GPIO_InitTypeDef GPIO_InitStructure;//Create GPIO_InitStructure 
	GPIO_StructInit(&GPIO_InitStructure); // Reset GPIO_structure

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3); 
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3); 
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOB, &GPIO_InitStructure );  
   
}
void GPIO_Configuration2(void){
	GPIO_InitTypeDef GPIO_InitStructure;//Create GPIO_InitStructure 
	GPIO_StructInit(&GPIO_InitStructure); // Reset GPIO_structure
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2); 
	// set GPIOD_Pin2 to AF_TIM2
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           
	 // Alt Function - Push Pull
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOB, &GPIO_InitStructure );  
   
}

/**
  * @brief  configure the TIM4 for PWM mode
  * @param  None
  * @retval None
  */
void TIM_Configuration4(void)
{

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	// Let PWM frequency equal 100Hz.
	// Let period equal 1000. Therefore, 
	//timer runs from zero to 1000. Gives 0.1Hz resolution.
	
	// Solving for prescaler gives 240.
	TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV4;
	TIM_TimeBaseInitStruct.TIM_Period = (period - 1)/p_scale;   
	//84000000/1680*1000=50hz  20ms for cycle
	TIM_TimeBaseInitStruct.TIM_Prescaler = prescalar - 1; 
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;    
	TIM_TimeBaseInit( TIM4, &TIM_TimeBaseInitStruct );
	TIM_OCStructInit( &TIM_OCInitStruct );
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	// Initial duty cycle equals 0%. Value can range from zero to 65535.
	//TIM_Pulse = TIM4_CCR1 register (16 bits)
	TIM_OCInitStruct.TIM_Pulse = 0; //(0=Always Off, 65535=Always On)
	TIM_OC1Init( TIM4, &TIM_OCInitStruct ); // Channel 1  LED
	TIM_Cmd( TIM4, ENABLE );
}

void TIM_Configuration3(void)
{

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	// Let PWM frequency equal 100Hz.
	// Let period equal 1000. Therefore, 
	//timer runs from zero to 1000. Gives 0.1Hz resolution.
	
	// Solving for prescaler gives 240.
	TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV4;
	TIM_TimeBaseInitStruct.TIM_Period = (period - 1)/p_scale;   
	//84000000/1680*1000=50hz  20ms for cycle
	TIM_TimeBaseInitStruct.TIM_Prescaler = prescalar  - 1; 
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;    
	TIM_TimeBaseInit( TIM3, &TIM_TimeBaseInitStruct );
	TIM_OCStructInit( &TIM_OCInitStruct );
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	// Initial duty cycle equals 0%. Value can range from zero to 65535.
	//TIM_Pulse = TIM4_CCR1 register (16 bits)
	TIM_OCInitStruct.TIM_Pulse = 0; //(0=Always Off, 65535=Always On)
	TIM_OC1Init( TIM3, &TIM_OCInitStruct ); // Channel 1  LED
	TIM_OC2Init( TIM3, &TIM_OCInitStruct ); // Channel 1  LED
//	TIM_OC3Init( TIM3, &TIM_OCInitStruct ); // Channel 1  LED
//	TIM_OC4Init( TIM3, &TIM_OCInitStruct ); // Channel 1  LED
	TIM_Cmd( TIM3, ENABLE );
}

void TIM_Configuration2(void)
{

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	// Let PWM frequency equal 100Hz.
	// Let period equal 1000. Therefore, 
	//timer runs from zero to 1000. Gives 0.1Hz resolution.
	
	// Solving for prescaler gives 240.
	TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV4;
	TIM_TimeBaseInitStruct.TIM_Period = (period - 1)/p_scale;   
	//84000000/1680*1000=50hz  20ms for cycle
	TIM_TimeBaseInitStruct.TIM_Prescaler = prescalar  - 1; 
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;    
	TIM_TimeBaseInit( TIM2, &TIM_TimeBaseInitStruct );
	TIM_OCStructInit( &TIM_OCInitStruct );
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	// Initial duty cycle equals 0%. Value can range from zero to 65535.
	//TIM_Pulse = TIM4_CCR1 register (16 bits)
	TIM_OCInitStruct.TIM_Pulse = 0; //(0=Always Off, 65535=Always On)
//	TIM_OC1Init( TIM2, &TIM_OCInitStruct ); // Channel 1  LED
	TIM_OC2Init( TIM2, &TIM_OCInitStruct ); // Channel 1  LED
//	TIM_OC3Init( TIM2, &TIM_OCInitStruct ); // Channel 1  LED
//	TIM_OC4Init( TIM2, &TIM_OCInitStruct ); // Channel 1  LED
	TIM_Cmd( TIM2, ENABLE );
}

void USART1_puts(char* s)
{
    while(*s) {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, *s);
        s++;
    }
}
static void BuBuTask(void *pvParameters)
{
  volatile int i;
  int n = 1;
  int x = 1;
  int c = 1;
  int pulse=42;
  int Button=0;

  //Timer4
  RCC_Configuration4();
  TIM_Configuration4();
  GPIO_Configuration4();
  
  //Timer3
  RCC_Configuration3();
  TIM_Configuration3();
  GPIO_Configuration3();
  
  
  char localCmd='#';
  while(1){  // Do not exit
      xSemaphoreTake(t_mutex, portMAX_DELAY);
	  localCmd = cmd;
	  xSemaphoreGive(t_mutex);
 
      if( localCmd == 'B' ){
          bubuStop();
      }

      if( localCmd == 'A' ){

          bubuGo();
          while (1 ){
                  
		  
		  	xSemaphoreTake(t_mutex, portMAX_DELAY);			
		  	localCmd = cmd;
		  	xSemaphoreGive(t_mutex);
                  
          	if( localCmd == '3'){
		      bubuGo();
          	}
	      	if( localCmd == '2' ){
              bubuLeftfront();
	      	}
		  	if( localCmd == '4' ){
		      bubuRightfront();
		  	}
		  	if( localCmd == '1' ){
		      bubuLeft();
		  	}
  		  	if( localCmd == '5' ){
		      bubuRight(); 
		 	}
	 	  	if( localCmd == 'B'){
			  bubuStop();
			  break;
		 	}
		  
	     }
	  }

   }//while
}
static void BuBuBeatTask(void *pvParameters){
    //Timer2
    RCC_Configuration2();
    TIM_Configuration2();
    GPIO_Configuration2();
    char localCmd='%';
    while(1){
	    xSemaphoreTake(t_mutex, portMAX_DELAY);
		localCmd = cmd;
		xSemaphoreGive(t_mutex);
			
	    if(localCmd=='C'){
	        TIM2->CCR2=42;
 		    vTaskDelay(1000);
		    TIM2->CCR2=210;
            vTaskDelay(1000);
	  	}
	}

}

static void BuBuSplasherTask(void *pvParameters){
	RCC_Configuration3();
	TIM_Configuration3();
    GPIO_Configuration3();
    char localCmd='%';
    while(1){
	    xSemaphoreTake(t_mutex, portMAX_DELAY);
		localCmd = cmd;
		xSemaphoreGive(t_mutex);
			

		if(localCmd=='C'){
	    	TIM3->CCR2=42;
 			vTaskDelay(1000);
			TIM3->CCR2=210;
        	vTaskDelay(1000);
		}
	}

}
static void uartTask(void *pvParameters){

    //USART
    RCC_Configuration();
    GPIO_Configuration();
    USART1_Configuration();
    while(1){
			while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
			xSemaphoreTake(t_mutex, portMAX_DELAY);
			cmd =  USART_ReceiveData(USART1);
			xSemaphoreGive(t_mutex);
	}

}

//Main Function
int main(void)
{

	t_queue = xQueueCreate(1, sizeof(int));
	if (!t_queue) {
		ReportError("Failed to create t_queue");
		while(1);
	}

	t_mutex = xSemaphoreCreateMutex();
	if (!t_mutex) {
		ReportError("Failed to create t_mutex");
		while(1);
	}

	xTaskCreate(BuBuTask, (char *) "USART", 256,
		   	NULL, tskIDLE_PRIORITY + 2, NULL);

	xTaskCreate(BuBuBeatTask, (char *) "USART", 256,
		   	NULL, tskIDLE_PRIORITY + 2, NULL);

	xTaskCreate(BuBuSplasherTask, (char *) "USART", 256,
		   	NULL, tskIDLE_PRIORITY + 2, NULL);

	xTaskCreate(uartTask, (char *) "USART", 256,
		   	NULL, tskIDLE_PRIORITY + 2, NULL);

	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
        RNG_Cmd(ENABLE);

	//Call Scheduler
	vTaskStartScheduler();
}

