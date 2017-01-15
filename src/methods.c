//********************************************************************
//*           EEE3061W Methods For Operation                		 *
//*                    		                                         *
//*==================================================================*
//* WRITTEN BY:   	Matthew Finlayson  		                         *
//* DATE CREATED:   11 September 2016                                *
//* MODIFIED:                                                        *
//*==================================================================*
//* PROGRAMMED IN: Eclipse Luna Service Release 1 (4.4.1)            *
//* DEV. BOARD:    UCT STM32 Development Board                       *
//*==================================================================*
//* DESCRIPTION:    Methods needed to complete project task as       *
//*                 outlined in 2016 course notes                    *
//********************************************************************

// INCLUDE FILES
//====================================================================
#include "lcd_stm32f0.h"
#include "stm32f0xx.h"
#include "methods.h"
#include <stdio.h>
//====================================================================
// GLOBAL CONSTANTS
//====================================================================
uint16_t TIM_ARR_VAL = 4799;

//====================================================================
// GLOBAL VARIABLES
//====================================================================


//====================================================================
// INITAILISE PORTS - init_Ports()
//====================================================================
// DESCRIPTION: This function configures the ports for all methods
//====================================================================
// TODO: Complete setting up all necessary ports
void init_Ports()
{

	//enable GPIOA
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// set SW0-SW3 to input mode
	GPIOA->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2 | GPIO_MODER_MODER3);

	// set SW0-SW3 to pull up
	GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0 | GPIO_PUPDR_PUPDR2_0 | GPIO_PUPDR_PUPDR3_0);

	// set pot0 (PA7) to analogue mode
	//GPIOA->MODER |= GPIO_MODER_MODER7;
	//GPIOA->MODER |= GPIO_MODER_MODER5; // for testing
}


//====================================================================
// INITIALISE LIGHT SENSOR - init_LightSensor
//====================================================================
// DESCRIPTION: Configures the ports needed for the light sensor
//				PA4 is defined to be the output from light sensor cct
//====================================================================
void init_LightSensor(){
	// enable GPIOA
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	// set PA4 to analog mode
	GPIOA->MODER |= GPIO_MODER_LIGHTSENSOR;
}

//====================================================================
// INITIALISE PROXIMITY SENSOR - init_ProxSensor
//====================================================================
// DESCRIPTION: Configures the ports needed for the proximity sensor
//				PA8 is defined to be the output from prox. sensor cct
//====================================================================
void init_ProxSensor(){

	// enable GPIOA
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	// configure to be analog mode
	GPIOB->MODER |= (GPIO_MODER_PROXSENSOR | GPIO_MODER_PROXPOT);

}

//====================================================================
// INITIALISE LINE SENSOR - init_LineSensor
//====================================================================
// DESCRIPTION: Configures the ports needed for the line sensor
//				PA5 - PA7 are the desired ports for each individual
//				sensor (from left to right).
//====================================================================
void init_LineSensor(){
	// enable GPIOA
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	// set PA5 - PA7 to analog mode
	GPIOA->MODER |= (GPIO_MODER_LINESENSOR1 | GPIO_MODER_LINESENSOR2 | GPIO_MODER_LINESENSOR3);
}

//====================================================================
// INITITALISE ADC - init_ADC()
//====================================================================
// DESCRIPTION:	This function configures the ADC
// 				returns mode if configured correctly, -1 otherwise
//====================================================================
int init_ADC(int res){
	// mode can be either 6, 8, 10 or 12

		RCC->APB2ENR |= RCC_APB2ENR_ADCEN; // enable ADC clk

		ADC1->CR |= ADC_CR_ADCAL; // calibrate
		while ((ADC1->CR & ADC_CR_ADCAL) != 0) {} // wait

		ADC1->CFGR1 &= ~(ADC_CFGR1_ALIGN); // set right alignment

		switch(res){
		case 6:
			ADC1->CFGR1 |= ADC_CFGR1_RES; // set to 6 bit mode (11)
			break;
		case 8:
			ADC1->CFGR1 |= ADC_CFGR1_RES_1; // set to 8 bit mode (10)
			break;
		case 10:
			ADC1->CFGR1 |= ADC_CFGR1_RES_0; // set to 10 bit mode (01)
			break;
		case 12:
			ADC1->CFGR1 &= ~ADC_CFGR1_RES; // set to 12 bit mode (00)
			break;
		default: // specified mode not 6, 8, 10 or 12
			return -1;
		}

		ADC1->CR |= ADC_CR_ADEN; // enable ADC
		while((ADC1->ISR & ADC_ISR_ADRDY) == 0){} // wait for ADC to be ready

		return res;
}

//====================================================================
// READ ADC - read_ADC()
//====================================================================
// DESCRIPTION:	This function returns the value detected by the ADC
//				with the resolution of it's configuration
//====================================================================
int read_ADC(int channel){

	ADC1->CHSELR = channel; 	// sample channel
	ADC1->CR = ADC_CR_ADSTART;		// start conversion
	while((ADC1->ISR & ADC_ISR_EOC) == 0){} // wait for conversion
	int output = ADC1->DR; // ADC output value
	return output;
}

//====================================================================
// INITIALISE NVIC - init_NVIC()
//====================================================================
// DESCRIPTION: Enables the TIM6 interrupts
//====================================================================
void init_NVIC(){

	NVIC_EnableIRQ(TIM6_DAC_IRQn); // enable TIM6 interrupt
	NVIC_EnableIRQ(TIM14_IRQn);

	// TODO: how to enable multiple timer interrupts?
}

//====================================================================
// INITIALISE TIMER 6 - init_TIM6()
//====================================================================
// DESCRIPTION: Configures TIM6 such that the period of timing is
//				defined to be time x 0.1s.
//				Returns period if successful, -1 otherwise
//====================================================================
int init_TIM6(int t) {

	if (t < 1 || t > 80){return -1;}

	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // enable clk
	TIM6->PSC = (uint16_t) 5999;
	TIM6->ARR = (uint16_t) (200*t-1);
	TIM6->DIER |= TIM_DIER_UIE; // enable interrupts
	TIM6->CR1 |= TIM_CR1_CEN; //start timer
	return t; // returns time variable
}

void unclk_TIM6(){
	RCC->APB1ENR &= ~(RCC_APB1ENR_TIM6EN); // enable clk
}


//====================================================================
// INITIALISE TIMER 14 - init_TIM14()
//====================================================================
// DESCRIPTION: Configures TIM14 such that the period of timing is
//				defined to be time x 0.1s.
//				Returns period if successful, -1 otherwise
//====================================================================
int init_TIM14(int t) {

	if (t < 1 || t > 80){return -1;}

	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN; // enable clk
	TIM14->PSC = (uint16_t) 5999;
	TIM14->ARR = (uint16_t) (800*t-1);
	TIM14->DIER |= TIM_DIER_UIE; // enable interrupts
	TIM14->CR1 |= TIM_CR1_CEN; //start timer
	return t; // returns time variable
}

//====================================================================
// UNCLOCK TIMER 14 - unclk_TIM14()
//====================================================================
// DESCRIPTION: Configures TIM14 such that the period of timing is
//				defined to be time x 0.1s.
//				Returns period if successful, -1 otherwise
//====================================================================
void unclk_TIM14(){
	RCC->APB1ENR &= ~(RCC_APB1ENR_TIM14EN); // disable clk
}


//====================================================================
// READ LIGHT SENSOR - read_LightSensor()
//====================================================================
// DESCRIPTION: Returns 1 if green, 2 if red, 3 if ambient detected
// 				The inputs specify the measured ADC values for each
// 				condition (calibration to be done in main.c)
//				Returns 0 if any param is zero
//====================================================================
int read_LightSensor(int green, int red, int ambient){

	// the code below assumes the following trend in ADC values
	// green > red > ambient

	if ((green == 0) | (red == 0) | (ambient == 0)){
		return 0;
	}

	int GR_diff = (green - red)/2;
	int RA_diff = (red - ambient)/2;

	// read ADC value of pin light sensor (PA4)
	int ADC_val = read_ADC(ADC_CHSELR_LIGHTSENSOR);

	// determine light being sensed based on calibration input params
	if (ADC_val > (red + GR_diff)){
		return GREEN; // sensing green
	}
	else if (ADC_val < (red - RA_diff)){
		return AMBIENT; // sensing ambient light
	}
	else {
		return RED; // sensing red
	}
}

//====================================================================
// READ PROXIMITY SENSOR - read_ProxSensor()
//====================================================================
// DESCRIPTION:	Returns the cm distance to the sensed object.
//				Takes in an array of 15 ADC values used for calibration,
//				the values are calibrated distances 1-15cm.
//				(Calibration to be done in main.c)
//====================================================================
int read_ProxSensor(int param[15]){

	// read ADC value of proximity sensor input (pin PA8)
	int ADC_val = read_ADC(ADC_CHSELR_PROXSENSOR);

	int closest = 10000000; // set closest to arbitrary high value
	int index = -2;
	// determine which value in param the ADC_val is closest to
	for (int i = 0; i  < 15; i++)
	{
		int diff = ADC_val - param[i];
		diff = diff*diff;

		if (diff < closest){
			closest = diff;
			index = i;
		}
	}

	// for testing


	//for(;;){
//
	//}

	return index+1; // returns the cm distance
}

//====================================================================
// CALCULATE PROXIMITY POT INDEX - calc_ProxPotIndex()
//====================================================================
// DESCRIPTION:	Returns the index 0-15 specified by the proxPot.
//				Linearly interpolates between 0 and the highest
//				possible pot value as per RESOLUTION
//====================================================================
int calc_ProxPotIndex(int x){

	int num_values = 2;
	for (int i = 1; i < RESOLUTION; i++){
		num_values *= 2;
	}
	int index = ((x*1000)/num_values)*15;
	index /=1000;
	return index;
}


//====================================================================
// READ LINE SENSOR - read_LineSensor()
//====================================================================
// DESCRIPTION: Returns a lineFlags type depending on the outputs for
//				each individual sensor. Threshold specifies the limit//
// 				of detecting the line
//				000 - 111 where each bit represents that sensor HIGH
//====================================================================
int read_LineSensor(int line, int noline){

	int output = 0;
	int threshold = (line + noline)/2;

	int ADC_val1 = read_ADC(ADC_CHSELR_LINESENSOR1);
	int ADC_val2 = read_ADC(ADC_CHSELR_LINESENSOR2);
	int ADC_val3 = read_ADC(ADC_CHSELR_LINESENSOR3);

	// code below assumes that the volt output goes higher for detecting line
	// this requires that resistor be connected between photodiode and Vcc
	if (ADC_val1 > threshold){
		output |= LEFT;
	}
	if (ADC_val2 > threshold){
		output |= MID;
	}
	if (ADC_val3 > threshold){
		output |= RIGHT;
	}

	return output;
}

//====================================================================
// INITALISE PWM - init_PWM()
//====================================================================
// DESCRIPTION: Initialises TIM2 to be in PWM mode on channels 3 and 4
//				which are linked to PB10 and PB11. These pins are also
// 				then configured to be in the correct mode
//====================================================================
void init_PWM(){

	//enable GPIOA
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	#define GPIO_AFRH_PIN8_AF2  	0b0010;
	#define GPIO_AFRH_PIN9_AF2 		0b0010<<4;

			RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //
			RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // ENABLE CLOCK TIM1
			GPIOA->MODER |= GPIO_MODER_MODER8_1; // SET PA8 TO AF MODE
			GPIOA->MODER |= GPIO_MODER_MODER9_1; // SET PA9 TO AF MODE
			GPIOA->AFR[1] |= GPIO_AFRH_PIN8_AF2; // SET PA8 TO AF2
			GPIOA->AFR[1] |= GPIO_AFRH_PIN9_AF2; // set PA9 to AF2

            // GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8;               // Pin A8 speed = 50MHz

			TIM1->CCMR1 &= ~TIM_CCMR1_CC1S; // config TIM1 ch1 for OC
			TIM1->CCMR1 &= ~TIM_CCMR1_CC2S; // config TIM1 ch2 for OC

			TIM1->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2); // x PWM
			TIM1->CCMR1 &= ~TIM_CCMR1_OC1M_0; // PWM mode 1 ?
			TIM1->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2); // select PWM  mode for ch2
			TIM1->CCMR1 &= ~TIM_CCMR1_OC2M_0;

			TIM1->PSC |= 2;
			TIM1->ARR = TIM_ARR_VAL;
			setPWM1(50);
			setPWM2(50);
			//TIM1->CCR2 = 24000;

			TIM1->CR1 &= ~TIM_CR1_CMS;		// set edge-alinged PWM

			TIM1->CCER &= ~TIM_CCER_CC1P;	// set ch1 active high
			TIM1->CCER &= ~TIM_CCER_CC1NP;	// clear cc1np bit for oc

			TIM1->CCER &= ~TIM_CCER_CC2P; // set ch2 active high
			TIM1->CCER &= ~TIM_CCER_CC2NP; // clear CC2NP for OC

			TIM1->CCER |= TIM_CCER_CC1E;	// x enable output compare
			TIM1->CCER |= TIM_CCER_CC2E; // enable output compare ch2

            TIM1->BDTR |= TIM_BDTR_MOE; // set bit 15 (Main Output Enable)
			TIM1->CR1 |= TIM_CR1_CEN;		// x enable counter for tim1
}

void setPWM1(int percent){
	if(percent == 100){
		TIM1->CCR1 = (TIM_ARR_VAL);
	}
	else if (percent >= 50){
		TIM1->CCR1 = (TIM_ARR_VAL/2);
	}
	else if (percent >= 30){
		TIM1->CCR1 = (TIM_ARR_VAL/3);
	}
	else if(percent >= 20){
		TIM1->CCR1 = (TIM_ARR_VAL/4);
	}
	else if(percent >=10){
		TIM1->CCR1 = (TIM_ARR_VAL/6);
		}
	else{
		TIM1->CCR1 = 0;
	}

	/*
	uint16_t val = (TIM_ARR_VAL+1)*(percent/100);
	//val = val/100;
	TIM1->CCR1 = val;*/
}

void setPWM2(int percent){
	if(percent == 100){
		TIM1->CCR2 = (TIM_ARR_VAL);
	}
	else if (percent >= 50){
		TIM1->CCR2 = (TIM_ARR_VAL/2);
	}
	else if (percent >= 30){
		TIM1->CCR2 = (TIM_ARR_VAL/3);
	}
	else if(percent >= 20){
		TIM1->CCR2 = (TIM_ARR_VAL/4);
	}
	else if(percent >=10){
		TIM1->CCR2 = (TIM_ARR_VAL/6);
	}
	else{
		TIM1->CCR2 = 0;
	}
	/*
	int val = (TIM_ARR_VAL+1)*(percent/100);
	//val = val/100;
	TIM1->CCR2 = (uint16_t)val;*/
}

void enableMotors(){
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= (GPIO_MODER_MODER10_0);
	GPIOA->ODR |= GPIO_ODR_10;
}
void disableMotors(){
	GPIOA->ODR &= ~GPIO_ODR_10;
}

void enableGripper(){
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= (GPIO_MODER_MODER11_0);
	GPIOA->ODR |= GPIO_ODR_11;
}
void disableGripper(){
	GPIOA->ODR &= ~GPIO_ODR_11;
}

void setDir(int dir){
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= (GPIO_MODER_MODER12_0);
	if (dir > 0){
		GPIOA->ODR |= GPIO_ODR_12;
	}
	else{
		GPIOA->ODR &= ~GPIO_ODR_12;
	}

}
