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

static int traffic_index = 0; 
static int button_change_traffic = 0;
static int states[] = {TRAFFIC_RED, TRAFFIC_YELLOW, TRAFFIC_GREEN, 
							TRAFFIC_YELLOW};
static float axes[3] = {0};
int count = 0;

const int p_scale = 1;
const int period = 1680 ;
const int prescalar = 1000 * 2 ;
const int T4full = 1680 * 2 ;
const int T3full = 1680 ;
const float s = 0.6 ;


void bubuGo(void){
    //TIM4->CCR1=((period/2.5)*0.825)/p_scale; 
    //TIM3->CCR1=((period/3)*0.825)/p_scale; 
    TIM4->CCR1=T4full;
    TIM3->CCR1=T3full;
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
/*
static char* itoa(int value, char* result, int base)
{
	if (base < 2 || base > 36) {
		*result = '\0';
		return result;
	}
	char *ptr = result, *ptr1 = result, tmp_char;
	int tmp_value;RCC_Configuration

	do {
		tmp_value = value;
		value /= base;
		*ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
	} while (value);

	if (tmp_value < 0) *ptr++ = '-';
	*ptr-- = '\0';
	while (ptr1 < ptr) {
		tmp_char = *ptr;
		*ptr-- = *ptr1;
		*ptr1++ = tmp_char;
	}
	return result;
}*/
void
prvInit()
{
	//LCD init
	LCD_Init();
	IOE_Config();
	LTDC_Cmd( ENABLE );

	LCD_LayerInit();
	LCD_SetLayer( LCD_FOREGROUND_LAYER );
	LCD_Clear( LCD_COLOR_BLACK );
	LCD_SetTextColor( LCD_COLOR_WHITE );

	//Button
	STM_EVAL_PBInit( BUTTON_USER, BUTTON_MODE_GPIO );
}

static void GetTrafficState(int change_state, int *v_state, int *h_state)
{

	switch (change_state) {
	case TRAFFIC_RED:
		*v_state = TRAFFIC_RED;
		*h_state = TRAFFIC_GREEN;
		break;
	case TRAFFIC_YELLOW:
		if (*v_state == TRAFFIC_GREEN)
			*v_state = TRAFFIC_YELLOW;
		else
			*h_state = TRAFFIC_YELLOW;
		break;
	case TRAFFIC_GREEN:
		*v_state = TRAFFIC_GREEN;
		*h_state = TRAFFIC_RED;
		break;
	default:
		ReportError("out of range");
		break;
	}
}

static void DrawGraphTask( void *pvParameters)
{
	const portTickType ticks = 100 / portTICK_RATE_MS;
	int value;
	int traffic_v_state = TRAFFIC_GREEN;
	int traffic_h_state = TRAFFIC_RED;

	portBASE_TYPE status;

	DrawBackground();

	while ( 1 ) {
		/*
		 * Check if the traffic changed event is sent to
		 * the queue. If so, we need to change the traffic
		 * light.
		 */
		status = xQueueReceive(t_queue, &value, ticks);

		if (status == pdPASS) {
			GetTrafficState(value, &traffic_v_state, 
						&traffic_h_state);
		}

		MoveCar(traffic_v_state, traffic_h_state);
	}
}

static void ChgTrafficLightTask(void *pvParameters)
{
	int num_ticks;
	int states_num = sizeof(states) / sizeof(states[0]);

	portBASE_TYPE status;
	portTickType ticks = TRAFFIC_GREEN_TICK;

	while ( 1 ) {
		ticks = (states[traffic_index] == TRAFFIC_YELLOW ? 
			TRAFFIC_YELLOW_TICK : TRAFFIC_GREEN_TICK);

		num_ticks = ticks / TRAFFIC_TICK_SLICE;

		status = xQueueSendToBack(t_queue, &states[traffic_index++], 0);
	
		if (status != pdPASS)
			ReportError("Cannot send to the queue!");

		if (traffic_index >= states_num)
			traffic_index = 0;

		while (num_ticks--) { 
			xSemaphoreTake(t_mutex, portMAX_DELAY);
			
			if (button_change_traffic) {
				button_change_traffic = 0;
				xSemaphoreGive(t_mutex);
				break;
			}

			xSemaphoreGive(t_mutex);

			vTaskDelay(TRAFFIC_TICK_SLICE);
		}
	}
}

static void ButtonEventTask(void *pvParameters)
{
	while (1) {
		if( STM_EVAL_PBGetState( BUTTON_USER ) ){

			while( STM_EVAL_PBGetState( BUTTON_USER ) );

			xSemaphoreTake(t_mutex, portMAX_DELAY);
			button_change_traffic = 1;
			xSemaphoreGive(t_mutex);
		}
	}
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

void RCC_Configuration1(void)
{
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB , ENABLE );//Enalbe AHB for GPIOB
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4, ENABLE );//Enable APB for TIM4
}

void RCC_Configuration2(void)
{
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB , ENABLE );//Enalbe AHB for GPIOB
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, ENABLE );//Enable APB for TIM4
}

void RCC_Configuration3(void)
{
      /* USART1 clock enable */
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
      /* GPIOA clock enable */
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
}

/**
  * @brief  configure the PD12~15 to Timers
  * @param  None
  * @retval None
  */
  
void GPIO_Configuration1(void){
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
    
void GPIO_Configuration2(void){
	GPIO_InitTypeDef GPIO_InitStructure;//Create GPIO_InitStructure 
	GPIO_StructInit(&GPIO_InitStructure); // Reset GPIO_structure
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3); 
	// set GPIOD_Pin12 to AF_TIM4
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
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
void TIM_Configuration1(void)
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
	TIM_TimeBaseInit( TIM3, &TIM_TimeBaseInitStruct );
	TIM_OCStructInit( &TIM_OCInitStruct );
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	// Initial duty cycle equals 0%. Value can range from zero to 65535.
	//TIM_Pulse = TIM4_CCR1 register (16 bits)
	TIM_OCInitStruct.TIM_Pulse = 0; //(0=Always Off, 65535=Always On)
	TIM_OC1Init( TIM3, &TIM_OCInitStruct ); // Channel 1  LED
	TIM_OC2Init( TIM3, &TIM_OCInitStruct ); // Channel 1  LED
	TIM_OC3Init( TIM3, &TIM_OCInitStruct ); // Channel 1  LED
	TIM_OC4Init( TIM3, &TIM_OCInitStruct ); // Channel 1  LED
	TIM_Cmd( TIM3, ENABLE );
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
/*
    RCC_Configuration();
    GPIO_Configuration();
    USART1_Configuration();

    USART1_puts("Hello World!\r\n");
    USART1_puts("Just for STM32F429I Discovery verify USART1 with USB TTL Cable\r\n");

	char strLcd[20]="";e4
	int i=0;
    while(1)
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
        char t = USART_ReceiveData(USART1);
    	*(strLcd+i) = t;
		DrawUsartReceive(strLcd);
		i += 1;
        if ((t == '\r')) {	        
            while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		    USART_SendData(USART1, t);	
            t = '\n';
		
		}
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, t);		
    }

*/

  volatile int i;
  int n = 1;
  int x = 1;
  int c = 1;
  int pulse=42;
  int Button=0;

  //Timer4
  RCC_Configuration1();
  TIM_Configuration1();
  GPIO_Configuration1();
  
  //Timer3
  RCC_Configuration2();
  TIM_Configuration2();
  GPIO_Configuration2();


  
  //USART
  RCC_Configuration();
  GPIO_Configuration();
  USART1_Configuration();

  //TIM4 for left  DC motor
  //TIM3 for right DC motor
  
  TIM4->CCR1=0; //right
  TIM3->CCR1=0; //left

  while(1){  // Do not exit

      //uint8_t button1;
      //button1 = GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_13);

      while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
      char t = USART_ReceiveData(USART1);
      while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
      USART_SendData(USART1,t);

      if( t == 'B' ){
          bubuStop();
      }

      if( t == 'A' ){

       	  bubuGo();
          while (1 ){
                  
		  
	          while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
              char t = USART_ReceiveData(USART1);
                  
              if( t == '3'){
		          bubuGo();
              }

		      if( t == '2' ){
		          bubuLeftfront();
			  }
		      if( t == '4' ){
		          bubuRightfront();
		      }
		      if( t == '1' ){
		          bubuLeft();
		      }
  			  if( t == '5' ){
				  bubuRight(); 
		      }
			  if (t == 'B'){
				  bubuStop();
				  break;
		  	  }
		  
	     }
	  }

   }//while
}
static void Gyroscope_Init(void)
{
	L3GD20_InitTypeDef L3GD20_InitStructure;
	L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
	L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;
	L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
	L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;
	L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
	L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
	L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_250;
	L3GD20_Init(&L3GD20_InitStructure);

	L3GD20_FilterConfigTypeDef L3GD20_FilterStructure;
	L3GD20_FilterStructure.HighPassFilter_Mode_Selection = L3GD20_HPM_NORMAL_MODE_RES;
	L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_0;
	L3GD20_FilterConfig(&L3GD20_FilterStructure);
	L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_ENABLE);
}
static void Gyroscope_Update(void)
{
	uint8_t tmp[6] = {0};
	int16_t a[3] = {0};
	uint8_t tmpreg = 0;

	L3GD20_Read(&tmpreg, L3GD20_CTRL_REG4_ADDR, 1);
	L3GD20_Read(tmp, L3GD20_OUT_X_L_ADDR, 6);

	/* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
	if (!(tmpreg & 0x40)) {
		for (int i = 0; i < 3; i++)
			a[i] = (int16_t)(((uint16_t)tmp[2 * i + 1] << 8) | (uint16_t)tmp[2 * i]);
	} else {
		for (int i = 0; i < 3; i++)
			a[i] = (int16_t)(((uint16_t)tmp[2 * i] << 8) | (uint16_t)tmp[2 * i + 1]);
	}

	for (int i = 0; i < 3; i++){
		axes[i] = a[i] / 114.285f;
//		axes[i] += a[i]*delta / 114.285f;
	}

}
static void Gyroscope_Render(void)
{
	LCD_ClearLine(LCD_LINE_5);
	LCD_ClearLine(LCD_LINE_6);
	LCD_ClearLine(LCD_LINE_7);

	LCD_SetTextColor( LCD_COLOR_RED );

	char str[16] = "X: ";
	itoa(axes[0], str + 3, 10);
	LCD_DisplayStringLine(LCD_LINE_5, str);
	str[0] = 'Y';
	itoa(axes[1], str + 3, 10);
	LCD_DisplayStringLine(LCD_LINE_6, str);
	str[0] = 'Z';
	itoa(axes[2], str + 3, 10);
	LCD_DisplayStringLine(LCD_LINE_7, str);

	

	if(axes[2]>100){
		LCD_SetTextColor( 0x227700);
		LCD_DrawFullRect( 50+count, 50, 20, 10);
		count++;
	}

	
	if(axes[2]< -100){
		LCD_SetTextColor( 0xFFFFFF);
		LCD_DrawFullRect( 50+count, 10, 20, 10);
		count++;
	}
	if(axes[0] > 250 || 	axes[0] < -250){

		LCD_Clear( 0xEEEE00 );
	 	count = 0;
	}
	if(axes[1] > 250 || 	axes[1] < -250){

		LCD_Clear( LCD_COLOR_RED );
	 	count = 0;
	}
}

static void GyroscopeTask(void *pvParameters)
{
	Gyroscope_Init();
	while(1){
		Gyroscope_Update();
		Gyroscope_Render();   
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

	prvInit();
/*
	xTaskCreate(ChgTrafficLightTask, "Traffic Light Task", 256, 
			( void * ) NULL, tskIDLE_PRIORITY + 1, NULL);

	xTaskCreate(ButtonEventTask, (char *) "Button Event Task", 256,
		   	NULL, tskIDLE_PRIORITY + 1, NULL);

	xTaskCreate(DrawGraphTask, (char *) "Draw Graph Task", 256,
		   	NULL, tskIDLE_PRIORITY + 2, NULL);
*/
	xTaskCreate(BuBuTask, (char *) "USART", 256,
		   	NULL, tskIDLE_PRIORITY + 2, NULL);
/*
	xTaskCreate(GyroscopeTask, (char *) "Gyroscope", 256,
		   	NULL, tskIDLE_PRIORITY + 2, NULL);
*/
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
        RNG_Cmd(ENABLE);

	//Call Scheduler
	vTaskStartScheduler();
}

