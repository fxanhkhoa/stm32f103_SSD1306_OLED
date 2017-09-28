#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_gpio.h"
#include "usart_print.h"	
#include "MPU9250.h"
#include "stm32f10x_tim.h"
#include "Main.h"
//#include "MPU6050.h"
#include <math.h>
#include "Kalman.c"
#include "SSD1306.h"
#include <string.h>
#include <stdlib.h>

#define Rad2Dree       57.295779513082320876798154814105
#define PI	3.1415926535897932384626433832795
#define declinationAngle  0.007563

void sleep(long i);
void led_toggle(void);
float Distance(float x, float y);
int Get_Central(kalman p, kalman r, kalman y);

/****************** Get ******************/
void getLatLon(char *s);

uint32_t timer, time_pre =0, time_now = 0;

char s[100] = "";
char pos = 0;
char mode = NONE;

float lat,lon;


	
int main()
{
	//prvSetupHardware();
	/* Enable I2C and GPIO clocks */
    RCC_APB1PeriphClockCmd(MPU9250_I2C_RCC_Periph, ENABLE);
    RCC_APB2PeriphClockCmd(MPU9250_I2C_RCC_Port, ENABLE);
/***********************************************************************************************
*											GPIO_Init						
***********************************************************************************************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	/* Set Led */
		GPIO_InitTypeDef GPIO_InitStruct,GPIO_InitStructure,GPIO;
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
	  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	  GPIO_Init(GPIOC, &GPIO_InitStruct);
		
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
		//GPIO_ResetBits(GPIOC, GPIO_Pin_13);

/*************************************************************************************
*																	USART_Init
**************************************************************************************/
			
	USART_InitTypeDef USART;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	USART.USART_BaudRate = 115200;
	USART.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART.USART_StopBits = USART_StopBits_1;
	USART.USART_WordLength = USART_WordLength_8b;
	USART.USART_Parity = USART_Parity_No;
	USART.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	/*---- Configure USART1 ----*/
			USART.USART_BaudRate = 9600;
			USART_Init(USART1, &USART);
			USART.USART_BaudRate = 115200;
			USART_Init(USART2, &USART);
	/*---- Enable RXNE interrupt ----*/
			USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
			USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	/*---- USART ENABLE ----*/
			USART_Cmd(USART1, ENABLE);
			USART_Cmd(USART2, ENABLE);
	/* Enable USART1 global interrupt */
			NVIC_EnableIRQ(USART1_IRQn);
			NVIC_EnableIRQ(USART2_IRQn);
		/*------ TX-Pin PA9 & RX-Pin PA10 -----*/
			
			GPIO.GPIO_Pin = GPIO_Pin_9;
			GPIO.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, & GPIO);
			
			GPIO.GPIO_Pin = GPIO_Pin_10;
			GPIO.GPIO_Mode = GPIO_Mode_IN_FLOATING;
			GPIO.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO);
			
		/*---- TX-Pin PA2 & RX-Pin PA3 ----*/
			GPIO.GPIO_Pin = GPIO_Pin_2;
			GPIO.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, & GPIO);
			
			GPIO.GPIO_Pin = GPIO_Pin_3;
			GPIO.GPIO_Mode = GPIO_Mode_IN_FLOATING;
			GPIO.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO);
			
/************************************************************************************************
*															I2C_Init
************************************************************************************************/
		I2C_InitTypeDef I2C_InitStructure;
    //GPIO_InitTypeDef GPIO_InitStructure;

    /* Configure I2C pins: SCL and SDA */
    GPIO_InitStructure.GPIO_Pin = MPU9250_I2C_SCL_Pin | MPU9250_I2C_SDA_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(MPU9250_I2C_Port, &GPIO_InitStructure);

    /* I2C configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;// MPU6050 7-bit adress = 0x68, 8-bit adress = 0xD0;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = MPU9250_I2C_Speed;

    /* Apply I2C configuration after enabling it */
    I2C_Init(MPU9250_I2C, &I2C_InitStructure);
    /* I2C Peripheral Enable */
    I2C_Cmd(MPU9250_I2C, ENABLE);
/**************************************************************************************************
*											Timer
**************************************************************************************************/
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

		TIM_TimeBaseInitTypeDef timerInitStructure; 
		timerInitStructure.TIM_Prescaler = 36000;
		timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
		timerInitStructure.TIM_Period = 2-1;
		timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		timerInitStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM4, &timerInitStructure);
		TIM_Cmd(TIM4, ENABLE);
		TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
		
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
		/*----- NVIC Timer interrupt -----*/
			NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
	
	init(0x3C);
	display();
	
	while (1)
	{
		if (mode == NONE)
		{
		}
	}
}


extern "C" void USART1_IRQHandler(void)
{
		/* RXNE handler */
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
			s[pos] = (char)USART_ReceiveData(USART1);
			pos++;
			if (s[pos - 1] == '\n')
			{
				//led_toggle();
				if (strstr(s, "$GNGLL") != NULL)
				{
					//led_toggle();
					//GPIO_ResetBits(GPIOC, GPIO_Pin_13);
					getLatLon(s);
				}
				pos = 0;
				strcpy(s, "");
			}
		}
    /* ------------------------------------------------------------ */
    /* Other USART1 interrupts handler can go here ...             */
		//U_Print_Char(USART2, "End");
}  

extern "C" void USART2_IRQHandler(void)
{
		/* RXNE handler */
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
			//GPIO_ResetBits(GPIOC, GPIO_Pin_13);
			//s[pos] = (char)USART_ReceiveData(USART1);
		}
}

void sleep(long i)
{
	for (long k = 0 ; k < i; k++);
}

extern "C" void TIM4_IRQHandler()
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
				timer ++;
				time_now++;
    }
}

void led_toggle(void)
		{
				/* Read LED output (GPIOA PIN8) status */
				uint8_t led_bit = GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13);
			 
				/* If LED output set, clear it */
				if(led_bit == (uint8_t)Bit_SET)
				{
						GPIO_ResetBits(GPIOC, GPIO_Pin_13);
				}
				/* If LED output clear, set it */
				else
				{
						GPIO_SetBits(GPIOC, GPIO_Pin_13);
				}
		}

void getLatLon(char *s)
{
	
}
		