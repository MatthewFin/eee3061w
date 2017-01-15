//********************************************************************
//*           EEE3061W Methods For Operation                		 *
//*                    		                                         *
//*==================================================================*
//* WRITTEN BY:   	Matthew Finlayson  		                         *
//* DATE CREATED:   11 September 2016                                *
//*MODIFIED:                                                         *
//*==================================================================*
//* PROGRAMMED IN: Eclipse Luna Service Release 1 (4.4.1)            *
//* DEV. BOARD:    UCT STM32 Development Board                       *
//*==================================================================*
//* DESCRIPTION:    Methods needed to complete project task as       *
//*                 outlined in 2016 course notes                    *
//********************************************************************
//*==================================================================*
//* CONNECTIONS:                                                     *
//*------------------------------------------------------------------*
//* PIN   	| 	NAME												 *
//*------------------------------------------------------------------*
//* PA0			SW0													 *
//* PA1			SW2									                 *
//* PA2 		SW3													 *
//* PA3 		SW4													 *
//* PA4 		LIGHT SENSOR INPUT									 *
//* PA5			LINE SENSOR INPUT 1								 	 *
//* PA6 		LINE SENSOR INPUT 2								 	 *
//* PA7 		LINE SENSOR INPUT 3								 	 *
//* PB0 		PROXIMITY SENSOR INPUT								 *
//* PB1			PROXIMITY SENSOR POT CONTROLLER 					 *
//* NEED TO INCLUDE MOTOR PINS AND PWM CONTORLLER PINS ETC..		 *
//* need two PWM pins and 2 enable lines and 1 dir lines
//* PA8 		PWM 1
//* PA9			PWM 2
//* PA10		enable motors (legs)
//* PA11		enable motors (gripper)
//* PA12		direction
//* PA13
//********************************************************************

//====================================================================
// GLOBAL CONSTANTS
//====================================================================
#define SW0 GPIO_IDR_0
#define SW1 GPIO_IDR_1
#define SW2 GPIO_IDR_2
#define SW3 GPIO_IDR_3


#define __GPIO_MODER_POT0		GPIO_MODER_MODER5 // used for testing
#define __ADC_CHSELR_POT0		ADC_CHSELR_CHSEL5
#define __GPIO_MODER_POT1		GPIO_MODER_MODER6
#define __ADC_CHSELR_POT1		ADC_CHSELR_CHSEL6

// switches already defined in stm32f0xx.h correctly
#define GPIO_MODER_LIGHTSENSOR 		GPIO_MODER_MODER4
#define ADC_CHSELR_LIGHTSENSOR		ADC_CHSELR_CHSEL4

/*
#define GPIO_MODER_PROXSENSOR 		__GPIO_MODER_POT0
#define ADC_CHSELR_PROXSENSOR 		__ADC_CHSELR_POT0
#define GPIO_MODER_PROXPOT			__GPIO_MODER_POT1
#define ADC_CHSELR_PROXPOT			__ADC_CHSELR_POT1
*/

#define GPIO_MODER_PROXSENSOR		GPIO_MODER_MODER0
#define ADC_CHSELR_PROXSENSOR		ADC_CHSELR_CHSEL8 // PB0 maps to chnl 8

#define GPIO_MODER_PROXPOT			GPIO_MODER_MODER1 // PB1
#define ADC_CHSELR_PROXPOT			ADC_CHSELR_CHSEL9 // PB1 maps to chnl 9

#define GPIO_MODER_LINESENSOR1 		GPIO_MODER_MODER5
#define GPIO_MODER_LINESENSOR2 		GPIO_MODER_MODER6
#define GPIO_MODER_LINESENSOR3 		GPIO_MODER_MODER7

#define ADC_CHSELR_LINESENSOR1 		ADC_CHSELR_CHSEL5
#define ADC_CHSELR_LINESENSOR2 		ADC_CHSELR_CHSEL6
#define ADC_CHSELR_LINESENSOR3 		ADC_CHSELR_CHSEL7
// testing (set input 1 and 2 to be pot0 and pot1), note that this is already the case

//#define LEGS1_EN

#define RESOLUTION	10


#define GREEN						1
#define RED							2
#define AMBIENT						3


//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void init_Ports();

int init_ADC(int);
int read_ADC(int);

void init_NVIC(void);
int init_TIM6(int);
void unclk_TIM6(void);

int init_TIM14(int);
void unclk_TIM14(void);

void init_LightSensor(void);
int read_LightSensor(int, int, int);

void init_ProxSensor(void);
int read_ProxSensor(int[]);
int calc_ProxPotIndex(int);

void init_LineSensor(void);
int read_LineSensor(int, int);

void init_PWM(void);
void setPWM1(int);
void setPWM2(int);

void enableMotors();
void disableMotors();
void enableGripper();
void disableGripper();
void setDir(int);

void resetVars(void);
// used to determine which of the line sensors is over the line
typedef int lineFlags;
enum{
	LEFT = 1 << 2,
	MID = 1 << 1,
	RIGHT = 1 << 0
};


