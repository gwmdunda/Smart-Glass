#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_exti.h"
#include <stm32f10x_pwr.h>
#include <stm32f10x_bkp.h>
#include <stm32f10x_rtc.h>
#include "stm32f10x_it.h"
#include "ssd1306.h"
#include "mpu9250.h"
#include "dht11.h"
#include <math.h>
#include "ov7725\bsp_ov7725.h"
#include <stdlib.h>
#include "sdcard\ff.h"
#include "sdcard\sdcard.h"
#include "bmpgenerator.h"
#include "wifi.h"
#include "HCSR04.h"

GPIO_InitTypeDef GPIO_InitStructure;
USART_InitTypeDef USART_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
I2C_InitTypeDef I2C_InitStructure;

#define GPS_RX_BUFFER_LEN 8192	

char deg[20];

char wifiRes[20];

uint32_t hr = 0;
uint32_t min = 0;
uint32_t sec  = 0;
uint32_t time = 0;
uint32_t cur_time = 0;

char gpsRxBuffer[GPS_RX_BUFFER_LEN];
uint16_t gps_buf_idx = 0;

int16_t accBuff[6];
char accResult[10];

char dhtResult[20];
int dhtRes;
struct DHT11_Dev dev;

char hcsr04Result[10];

char	*str;
I2C_InitTypeDef I2C_InitStructure;
ADC_InitTypeDef ADC_InitStructure;
extern uint8_t Ov7725_vsync;
SD_Error Status;
FATFS fs;
FIL image;
TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
EXTI_InitTypeDef EXTI_InitStructure;
volatile uint8_t option = 1;

typedef struct
{
	uint8_t			UTC_Hour;
	uint8_t			UTC_Min;
	uint8_t			UTC_Sec;
	uint16_t		UTC_MicroSec;
	
	float				Latitude;
	double			LatitudeDecimal;
	char				NS_Indicator;
	float				Longitude;
	double			LongitudeDecimal;
	char				EW_Indicator;
	
	uint8_t			PositionFixIndicator;
	uint8_t			SatellitesUsed;
	float				HDOP;
	float				MSL_Altitude;
	char				MSL_Units;
	float				Geoid_Separation;
	char				Geoid_Units;
	
	uint16_t		AgeofDiffCorr;
	char				DiffRefStationID[4];
	char				CheckSum[2];	
	
}GPGGA_t;

GPGGA_t GPGGA;

double convertDegMinToDecDeg (float degMin)
{
  double min = 0.0;
  double decDeg = 0.0;
 
  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);
 
  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );
 
  return decDeg;
}

volatile uint32_t msTicks;                      /* counts 1ms timeTicks       */

/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void) {
  msTicks++;
}

/*----------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *----------------------------------------------------------------------------*/
void Delayms (uint32_t dlyTicks) {                                              
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
}


void GPS_Parse(void){
	str = strstr(gpsRxBuffer,"$GPGGA,");
	if(str!=NULL)
	{
		memset(&GPGGA,0,sizeof(GPGGA));
		sscanf(str,"$GPGGA,%2hhd%2hhd%2hhd.%3hd,%f,%c,%f,%c,%hhd,%hhd,%f,%f,%c,%hd,%s,*%2s\r\n",&GPGGA.UTC_Hour,&GPGGA.UTC_Min,&GPGGA.UTC_Sec,&GPGGA.UTC_MicroSec,&GPGGA.Latitude,&GPGGA.NS_Indicator,&GPGGA.Longitude,&GPGGA.EW_Indicator,&GPGGA.PositionFixIndicator,&GPGGA.SatellitesUsed,&GPGGA.HDOP,&GPGGA.MSL_Altitude,&GPGGA.MSL_Units,&GPGGA.AgeofDiffCorr,GPGGA.DiffRefStationID,GPGGA.CheckSum);
		GPGGA.LatitudeDecimal=convertDegMinToDecDeg(GPGGA.Latitude);
		GPGGA.LongitudeDecimal=convertDegMinToDecDeg(GPGGA.Longitude);	
	}
}


void gps_init(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure); // TX
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure); // RX
	
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);
	
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init(&NVIC_InitStructure);
}


//Interrupt handler implementation
void USART1_IRQHandler()
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)//enter interrupt when STM32 receice data.
		{
			 USART_ClearITPendingBit(USART1, USART_IT_RXNE);
			 gpsRxBuffer[gps_buf_idx] = (unsigned char) USART_ReceiveData(USART1); //receive a char
			gps_buf_idx++;
			if(gps_buf_idx == GPS_RX_BUFFER_LEN)
				gps_buf_idx = 0;
		}
}

void set_time(uint32_t hr, uint32_t min, uint32_t sec) {
	uint32_t new_time = hr * 3600 + min * 60 + sec;
	RTC_SetCounter(new_time);
	RTC_WaitForLastTask();
}

void RTC_Timer_Init() {	
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	//RTC_ITConfig(RTC_IT_SEC, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	BKP_DeInit();
	RCC_LSEConfig(RCC_LSE_ON);
	while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) {}
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
	RCC_RTCCLKCmd(ENABLE);
	RTC_WaitForSynchro();
	RTC_SetPrescaler(32767);
	RTC_WaitForLastTask();
}

void timer_update(void) {
	time = RTC_GetCounter();
	hr = time / 3600;
	min = (time % 3600) / 60;
	sec  = (time % 3600) % 60;	
}

void RTC_IRQHandler() {
	
	timer_update();

	sprintf(deg, "%d:%d:%d", hr, min, sec);
	ssd1306_SetCursor(2,0);
	ssd1306_WriteString(deg, Font_7x10, White);
	ssd1306_UpdateScreen();

}

void i2c_init(){
	
/*Enable the clock for GPIO and I2C */
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

/* GPIO Pin Configuration */
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOB,&GPIO_InitStructure);

RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	
/* I2C configuration */

I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
I2C_InitStructure.I2C_OwnAddress1 = 0;
I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
I2C_InitStructure.I2C_ClockSpeed = 100000;

/* I2C Peripheral Enable */
I2C_Cmd(I2C1, ENABLE);

/* Apply I2C configuration after enabling it */
I2C_Init(I2C1, &I2C_InitStructure);
}

void mhrd_init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_55Cycles5);
	ADC_Cmd(ADC1, ENABLE);
	
	/* Enable ADC1 reset calibration register */
	ADC_ResetCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));
	/* Start ADC1 calibration */
	ADC_StartCalibration(ADC1);
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));
}

uint16_t get_mhrd_value()
{
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	return ADC_GetConversionValue(ADC1);
}

void Key_Configuration(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	 NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn; 
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;   
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	 NVIC_Init(&NVIC_InitStructure);
	 
	 GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource5);
	 EXTI_InitStructure.EXTI_Line = EXTI_Line5;
	 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	 EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	 EXTI_Init(&EXTI_InitStructure);
}

void EXTI9_5_IRQHandler(void)
{

	if (EXTI_GetITStatus(EXTI_Line5) != RESET) {
		ssd1306_Fill(Black);
		
		++option;

	/* Clear the Key Button EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line5);
	}
}

void horn_check(void)
{

	if(distance_value < 100){
		GPIO_ResetBits(GPIOA, GPIO_Pin_4);
	}
	else{
		GPIO_SetBits(GPIOA, GPIO_Pin_4);
	}
}

int main(void)
{	
	char display[100];
	uint8_t photo_taken = 1;
	
	char latitude[100];
	char longitude[100];
	char time[100];
	
  SystemCoreClockUpdate();                      /* Get Core Clock Frequency   */
  if (SysTick_Config(SystemCoreClock / 1000)) { /* SysTick 1 msec interrupts  */
    while (1);                                  /* Capture error              */
	}
	RTC_Timer_Init();
		
	// FSMC LCD
	//LCD_INIT();

	//set_time(13, 45, 30);

	wifi_init();
	gps_init();
	
	i2c_init();
	ssd1306_Init();
	Ov7725_GPIO_Config();
	
	while(Ov7725_Init() != SUCCESS){};
	
  VSYNC_Init();	
	Ov7725_vsync = 0;
	mhrd_init();
	
	Key_Configuration();
	NVIC_Configuration();

  //USARTx_Config();	

	Status = SD_Init();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	DHT11_init(&dev, GPIOA, GPIO_Pin_5);
	
	
	ssd1306_SetCursor(2,40);
	ssd1306_WriteString("Connecting to AP", Font_7x10, White);
	ssd1306_UpdateScreen();
	
	uart_send("AT+CWJAP_CUR=\"FD-76\",\"12345678\"\r\n");
	Delayms(100);
	ssd1306_SetCursor(2,40);
	ssd1306_WriteString("Done connecting to AP", Font_7x10, White);
	ssd1306_UpdateScreen();

	
	hcsr04_init();
	
	ssd1306_Fill(Black);
	MPU9250_init();
	while ( 1 ){
		
		uart_send("AT+CIPMUX=1\r\n");
		Delayms(10);
		uart_send("AT+CIPSTART=4,\"TCP\",\"rss.weather.gov.hk\",80\r\n");
		Delayms(10);
		uart_send("AT+CIPSEND=4,75\r\n");
		Delayms(10);
		esp8266ClearBuffer();
		uart_send("GET /rss/WeatherWarningSummaryv2.xml HTTP/1.0\r\nHost:rss.weather.gov.hk\r\n\r\n\r\n");
		Delayms(1000);
		
		switch(option%5) {
		
	//GPS data 
		case 0 :
		GPS_Parse();
		ssd1306_SetCursor(2,10);
		sprintf(time,"%02d:%02d:%02d",(GPGGA.UTC_Hour+8)%24,GPGGA.UTC_Min,GPGGA.UTC_Sec);
		ssd1306_WriteString(time, Font_7x10, White);
			
		sprintf(latitude, "Latitude = %f", convertDegMinToDecDeg(GPGGA.LatitudeDecimal));
		ssd1306_SetCursor(2,20);
		ssd1306_WriteString(latitude, Font_7x10, White);
			
		sprintf(longitude, "Longitude = %f", convertDegMinToDecDeg(GPGGA.LongitudeDecimal));
		ssd1306_SetCursor(2,30);
		ssd1306_WriteString(longitude, Font_7x10, White);
		
		break;
	//timer_update();
		case 1:
		ssd1306_SetCursor(2,10);
		ssd1306_WriteString("Camera Mode", Font_7x10, White);
		if( Ov7725_vsync == 2 && (GPIOA->IDR & GPIO_Pin_0) == GPIO_Pin_0 )
		{
			FIFO_PREPARE;
			sprintf(display,"test%i.bmp",photo_taken);
			generateBMP(display,&fs,&image);
			f_mount(NULL,"",1);
			//ssd1306_Fill(Black);
			sprintf(display, "%i photos taken",photo_taken);
			ssd1306_SetCursor(2,0);
			ssd1306_WriteString(display, Font_7x10, White);
			Ov7725_vsync = 0;	
			photo_taken++;
		}
		break;
	
		case 2:
			
		dhtRes = DHT11_read(&dev);
	
		if(dhtRes == DHT11_SUCCESS) {
			sprintf(dhtResult, "T %d - H %d", dev.temparature, dev.humidity);
		}
		ssd1306_SetCursor(2,0);
		ssd1306_WriteString(dhtResult, Font_7x10, White);
		
		ssd1306_SetCursor(2,10);
			
		readAccelData(accBuff);
		
		sprintf(accResult, "X:%.2f Y:%.2f", (double)accBuff[0]/(16384*2),  (double)accBuff[1]/(16384*2)); 	
			
		ssd1306_WriteString(accResult, Font_7x10, White);
			
		ssd1306_SetCursor(2,20);			
		ssd1306_WriteString(esp8266RxBuffer, Font_7x10, White);
		
	if (strstr(esp8266RxBuffer, "rainfall")) {
		
		char* des = strstr(esp8266RxBuffer, "rainfall");
		char* sentenceStart = strstr(des, "br");	
		sentenceStart = strstr(sentenceStart, "right\">");
		// Flush wifiRes 
		memset(wifiRes, '\0', 20);
		
		if (des != 0) {
			char* sentenceEnd = strstr(sentenceStart, "&nbsp");
			ssd1306_SetCursor(2,30);			
			ssd1306_WriteString("Rfall: ", Font_7x10, White);
			ssd1306_WriteString(strncpy(wifiRes, sentenceStart + 7 , sentenceEnd - sentenceStart - 7), Font_7x10, White);	
			ssd1306_WriteString(" mm", Font_7x10, White);
			
		} 
	}
		
		ssd1306_UpdateScreen();
		break;
	
		case 3:
			sprintf(display,"sweat: %i",get_mhrd_value());
		  ssd1306_SetCursor(2,30);
			ssd1306_WriteString(display, Font_7x10, White);
			break;
		
		case 4:
			horn_check();
			ssd1306_SetCursor(2,0);
			memset(hcsr04Result, '\0', 10);
		  sprintf(hcsr04Result, "%d cm ", distance_value); 	
			ssd1306_WriteString(hcsr04Result, Font_7x10, White);
			break;
				
			
		
	}
	timer_update();
	sprintf(deg, "%02d:%02d:%02d", hr, min, sec);
	ssd1306_SetCursor(2,40);
	ssd1306_WriteString(deg, Font_7x10, White);
	
	ssd1306_UpdateScreen();
	
	/*sprintf(deg, "%d:%d:%d", hr, min, sec);
	ssd1306_SetCursor(2,10);
	ssd1306_WriteString(deg, Font_7x10, White);
	
	ssd1306_SetCursor(2,0);
	memset(hcsr04Result, '\0', 10);
	sprintf(hcsr04Result, "%d cm", distance_value); 	
	ssd1306_WriteString(hcsr04Result, Font_7x10, White);
	
		
	if( Ov7725_vsync == 2 && (GPIOA->IDR & GPIO_Pin_0) == GPIO_Pin_0 )
	{
		FIFO_PREPARE;
		sprintf(display,"test%i.bmp",photo_taken);
		generateBMP(display,&fs,&image);
		f_mount(NULL,"",1);
		//ssd1306_Fill(Black);
		sprintf(display, "%i photos are taken",photo_taken);
		ssd1306_SetCursor(2,0);
		ssd1306_WriteString(display, Font_7x10, White);
		ssd1306_UpdateScreen();
		Ov7725_vsync = 0;	
		photo_taken++;
	}
	
	dhtRes = DHT11_read(&dev);
	
	if(dhtRes == DHT11_SUCCESS) {
		sprintf(dhtResult, "T %d - H %d", dev.temparature, dev.humidity);
	} else {
		sprintf(dhtResult, "Error Code: %d", dhtRes);
	}
	ssd1306_SetCursor(2,0);
	//ssd1306_WriteString(dhtResult, Font_7x10, White);
	
	ssd1306_UpdateScreen();*/
		
	}	
}
