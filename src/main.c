//********************************************************************
//*                    EEE3061W Project                          	 *
//*                    version 3                                     *
//*==================================================================*
//* WRITTEN BY:    	 	MATTHEW FINLAYSON         		           	 *
//* DATE CREATED:       12/09/2016                            		 *
//* MODIFIED:                                                        *
//*==================================================================*
//* PROGRAMMED IN: Eclipse Luna Service Release 1 (4.4.1)            *
//* DEV. BOARD:    UCT STM32 Development Board                       *
//*==================================================================*
//* DESCRIPTION:	Includes all code needed for operation of robot  *
//*					as per outlined specs                            *
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


/*
#define STAGE0 0
#define STAGE1 1
#define STAGE2 2
#define STAGE3 3*/

#define CALIBRATE_LIGHT	0
#define CALIBRATE_LINE	1
#define CALIBRATE_PROX	2

#define LCD_MAIN					-1
#define LCD_CALIBRATION_MODE_SELECT	0
#define LCD_CALIBRATION_LIGHT 		1
#define LCD_CALIBRATION_LINE		2
#define LCD_CALIBRATION_PROX		3
#define LCD_LIGHT					4
#define LCD_LINE_PROX				5
#define LCD_BATON_PROX				6

#define SENSING_LIGHT 				0
#define SENSING_LINE_PROX			1
#define SENSING_BATON_PROX			2
#define REVERSING					3

#define DELAY		5 // meaning 0.5s
#define PWM_CHANGETIME				4 // 4x0.5s = 2s delays
#define PWM_BASE_SPEED				50 //refers to duty cycle
#define BATON_DISTANCE				2 // cm, distance needed to grip from
//====================================================================
// GLOBAL VARIABLES
//====================================================================
int display_prox = 0;

int PWM_go = 50;
int PWM_nogo = 10;

typedef int bool;
enum{false, true};
bool nextStage = false;

typedef enum Directions{
	FORWARDS,
	BACKWARDS
} Direction;

int num_ends = 0;

Direction dir = FORWARDS;

int calibration_mode = -1;
int race_mode = -1;
// STORES THE MOST RECENT READ ADC VALUE
int ADC_val = 0;


//====================================================================
// 					CALIBRATION VARIABLES
// LIGHT SENSOR
//int green_adc, red_adc, ambient_adc = 0;
int green_adc = 100; int red_adc = 50; int ambient_adc = 25;

// LINE SENSOR
//int line_adc = 0;
//int noline_adc = 0;
int line_adc = 65; int noline_adc = 45;

// PROXIMITY SENSOR
int ADC_proxPot = 0;
//int proxParams[15] = {0};
					//1   2   3   4   5   6   7   8   9   10  11  12  13  14  15
int proxParams[15] = {50,100,150,200,250,300,350,400,450,500,550,600,650,700,750};
//====================================================================


//====================================================================
// 					OUTPUT VARIABLES
// LIGHT SENSOR
int colour_detected = -1;
// LINE SENSOR
int lineOutput = 0; // 3 bit long int
// PROXIMITY SENSOR
int distance = 15; // important that distance defined high
//====================================================================

//====================================================================
// 					CONTROL VARIABLES
// SPEED CONTROL
int PWM_L_speed = PWM_BASE_SPEED;
int PWM_R_speed = PWM_BASE_SPEED;
int PWM_cnt = PWM_CHANGETIME;
int gripper_cnt = 0;
// GRIPPER CONTROL
int PWM_gripper = 0;
//====================================================================


//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void TIM6_DAC_IRQHandler(void);
void TIM14_IRQHandler(void);
void displayC(int);
void displayR(int);
void calibrationMode(void);
void calibrateLightSensor(void);
void calibrateLineSensor(void);
void calibrateProxSensor(void);

void raceMode0(void);
void raceMode1(void);
void raceMode2(void);
void raceMode3(void);

void tdelay(void);


// deals with all calibration operations
void TIM14_IRQHandler(){

	switch(calibration_mode){

	default:
		// do nothing
		break;

	case CALIBRATE_LIGHT:
		ADC_val = read_ADC(ADC_CHSELR_LIGHTSENSOR);
		displayC(LCD_CALIBRATION_LIGHT);
		break;

	case CALIBRATE_LINE:
		ADC_val = read_ADC(ADC_CHSELR_LINESENSOR1);
		displayC(LCD_CALIBRATION_LINE);
		break;

	case CALIBRATE_PROX:
		ADC_val = read_ADC(ADC_CHSELR_PROXSENSOR);
		ADC_proxPot = read_ADC(ADC_CHSELR_PROXPOT); // read pot val
		displayC(LCD_CALIBRATION_PROX);
		break;
	}

	TIM14->SR &= ~TIM_SR_UIF;
}

// will deal with all operation and sensing interrupts
void TIM6_DAC_IRQHandler(){

	switch(race_mode){

	case SENSING_LIGHT:
		colour_detected = read_LightSensor(green_adc, red_adc, ambient_adc);
		displayR(LCD_LIGHT);
		break;

	case SENSING_LINE_PROX:

		if(dir == FORWARDS){ ////////
		ADC_val = read_ADC(ADC_CHSELR_PROXSENSOR);
		distance = read_ProxSensor(proxParams);
		}else{ ////////
			ADC_val = 0;
		}

		lineOutput = read_LineSensor(line_adc, noline_adc);

		if (lineOutput == 0b010){
			// reset pwm speed
			PWM_L_speed = PWM_go;// PWM_BASE_SPEED;
			PWM_R_speed = PWM_go;//PWM_BASE_SPEED;
			setPWM1(PWM_go);
			setPWM2(PWM_go);
			// reset timer
			//PWM_cnt = PWM_CHANGETIME;
		}

		/*
		if (PWM_cnt < PWM_CHANGETIME){
			PWM_cnt++;
			break;
		}*/

		// otherwise PWM_cnt is == PWM_CHANGETIME

		if (lineOutput == 0b110 || lineOutput == 0b100){ // right motor faster /// or left motor slower
			//PWM_R_speed = PWM_R_speed + 10; // TODO: Test and see if should change to const value
			//setPWM2(PWM_R_speed);
			PWM_L_speed = PWM_nogo;
			PWM_R_speed = PWM_go;

			setPWM1(PWM_nogo);
			setPWM2(PWM_go);
			//PWM_cnt = 0; // reset
		}
		else if (lineOutput == 0b001 || lineOutput == 0b011){ // left motor faster
			//PWM_L_speed = PWM_L_speed + 10;
			//setPWM1(PWM_L_speed);
			PWM_L_speed = PWM_go;
			PWM_R_speed = PWM_nogo;

			setPWM2(PWM_nogo);
			setPWM1(PWM_go);
			//PWM_cnt = 0; // reset
		}

		displayR(LCD_LINE_PROX);
		break;

	case SENSING_BATON_PROX:
		//TODO: THHINK OF HOW TO IMPLEMENT BATON GRIPPING
		lcd_putchar('.');
		gripper_cnt++;
		if (gripper_cnt > 6){
			nextStage = true;
		}
		break;
	}

	// branching to next stage done in displayR method

	TIM6->SR &= ~TIM_SR_UIF; // ack interrupt

}

// CREATES A SET TIME DELAY BETWEEN NORMAL CPU INSTRUCTIONS
void tdelay(){
	for(int i =0; i<1000; i++){
			for(int j=0; j<1000; j++){}
	}
}

//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void)
{

	init_LCD();
	lcd_putstring("EEE3061W v9");
	lcd_command(LINE_TWO);
	lcd_putstring("**FNLMAT001**");

	init_Ports();
	init_NVIC();
	init_ADC(RESOLUTION);
	init_LightSensor();
	init_ProxSensor();
	init_LineSensor();
	init_PWM();
	//enableGripper();
	enableMotors();
	//disableGripper();
	disableMotors();
	dir = FORWARDS;
	setDir(1);

	for(;;){ // press SW0 to move on
		if ((GPIOA->IDR & SW0) == 0){
			tdelay();
			break;
		}
	}

	int returned = 0;

	displayC(LCD_MAIN);

	tdelay();

	for(;;){

		if ((GPIOA->IDR & SW0) == 0){
			raceMode0();
			displayC(LCD_MAIN);
			// now finished with event
			returned = 1;
		}

		if ((GPIOA->IDR & SW1) == 0){ // move on only after SW1 pressed
			calibrationMode();
			displayC(LCD_MAIN);
			returned = 1;
		}
		//if ((GPIOA->IDR & SW2) == 0){
		//	// show calibration values
		//}

		if (returned == 1){
			// reset values

					dir = FORWARDS;
					calibration_mode = -1;
					race_mode = -1;
					ADC_val = 0;
					//green_adc = 800; red_adc = 500; ambient_adc = 200;
					//line_adc = 800; noline_adc = 200;
					ADC_proxPot = 0;

					colour_detected = -1;
					lineOutput = 0;
					distance = 15;
					PWM_L_speed = 30;
					PWM_R_speed = 30;
					//int PWM_cnt = PWM_CHANGETIME;
					gripper_cnt = 0;
					PWM_gripper = 0;
				returned = 0;
		}

	}

}

//====================================================================
// ALL CALIBRATION FUNCTIONS
//====================================================================
void calibrationMode (){

	displayC(LCD_CALIBRATION_MODE_SELECT);
	tdelay();

	for(;;){

		if ((GPIOA->IDR & SW0) == 0){
			calibrateLightSensor();
			displayC(LCD_CALIBRATION_MODE_SELECT);
		}
		if ((GPIOA->IDR & SW1) == 0){
			calibrateLineSensor();
			displayC(LCD_CALIBRATION_MODE_SELECT);
		}
		if ((GPIOA->IDR & SW2) == 0){
			calibrateProxSensor();
			displayC(LCD_CALIBRATION_MODE_SELECT);
		}

		if ((GPIOA->IDR & SW3) == 0){
			// return to start screen
			lcd_command(CLEAR);
			lcd_putstring("Exiting");
			lcd_command(LINE_TWO);
			lcd_putstring("  Calibration...");
			tdelay();
			return;
		}
	}
}

void calibrateLightSensor(){

	calibration_mode = CALIBRATE_LIGHT;
	displayC(LCD_CALIBRATION_LIGHT);

	init_TIM14(1);

	tdelay();

	for(;;){
		if ((GPIOA->IDR & SW0) == 0){
			// capture green
			green_adc = ADC_val;
			lcd_command(CLEAR);
			lcd_putstring("Green captured");
			lcd_command(LINE_TWO);
			char str[16];
			sprintf(str, "%u", ADC_val);
			lcd_putstring(str);
		}
		if ((GPIOA->IDR & SW1) == 0){
			// capture red
			red_adc = ADC_val;
			lcd_command(CLEAR);
			lcd_putstring("Red captured");
			lcd_command(LINE_TWO);
			char str[16];
			sprintf(str, "%u", ADC_val);
			lcd_putstring(str);
		}
		if ((GPIOA->IDR & SW2) == 0){
			// capture ambient
			ambient_adc = ADC_val;
			lcd_command(CLEAR);
			lcd_putstring("Ambient captured");
			lcd_command(LINE_TWO);
			char str[16];
			sprintf(str, "%u", ADC_val);
			lcd_putstring(str);
		}

		if((GPIOA->IDR & SW3) == 0){

			lcd_command(CLEAR);
			lcd_putstring("Returning");
			unclk_TIM14();
			tdelay();
			return; // break out of calibrateLightSensor

		}
	}
}

void calibrateLineSensor(){
	calibration_mode = CALIBRATE_LINE;
	displayC(LCD_CALIBRATION_LINE);

	init_TIM14(1);

	tdelay();

	for(;;){
		if ((GPIOA->IDR & SW0) == 0){
			// capture left line sensor when on the line
			line_adc = ADC_val;

			lcd_command(CLEAR);
			lcd_putstring("line captured");
			lcd_command(LINE_TWO);
			char str[16];
			sprintf(str, "%u", ADC_val);
			lcd_putstring(str);
		}

		if((GPIOA->IDR & SW1) == 0){
			// capture left line sensor when not on line
			noline_adc = ADC_val;

			lcd_command(CLEAR);
			lcd_putstring("noline captured");
			lcd_command(LINE_TWO);
			char str[16];
			sprintf(str, "%u", ADC_val);
			lcd_putstring(str);
		}

		if((GPIOA->IDR & SW3) == 0){
			lcd_command(CLEAR);
			lcd_putstring("Returning");
			unclk_TIM14();
			tdelay();
			return; // break out of calibrateLineSensor
		}
	}
}

void calibrateProxSensor(void){
	calibration_mode = CALIBRATE_PROX;

	lcd_command(CLEAR);
	lcd_putstring("USE POT TO SEL.");
	lcd_command(LINE_TWO);
	lcd_putstring("THE PARAM INDEX");

	init_TIM14(1);
	tdelay();

	for(;;){

		if ((GPIOA->IDR & SW0) == 0){
			// capture adc_val into proxParams array
			// index specified by proxPot
			int index = calc_ProxPotIndex(ADC_proxPot);
			proxParams[index] = ADC_val; // assign to value read off prox sensor
			lcd_command(CLEAR);
			lcd_putstring("captured");
		}

		if ((GPIOA->IDR & SW1) == 0){ // for testing purposes

			lcd_command(CLEAR);
			lcd_putstring("params[");
			int index = calc_ProxPotIndex(ADC_proxPot);
			char str[2];
			sprintf(str, "%u", index);
			lcd_putstring(str);
			lcd_putstring("]=");
			char str1[4];
			sprintf(str1, "%u", proxParams[index]); // UNLIKELY THAT ALL THIS WILL WORK
			lcd_putstring(str1);
		}

		if ((GPIOA->IDR & SW3) == 0){
			lcd_command(CLEAR);
			lcd_putstring("Returning");
			unclk_TIM14();
			tdelay();
			return; // return from calibrateProxSensor
		}
	}
}
//====================================================================
// END OF ALL CALIBRATION FUNCTIONS
//====================================================================


//====================================================================
// ALL RACING FUNCTIONS
//====================================================================
void raceMode0(void){
// light detecting stage

	race_mode = SENSING_LIGHT;

	//displayR(LCD_LIGHT);


	init_TIM6(1);
	init_LightSensor();

	tdelay();
	for(;;){

		if ((GPIOA->IDR & SW3) == 0){
			// return to start screen
			lcd_command(CLEAR);
			lcd_putstring("Exiting");
			lcd_command(LINE_TWO);
			lcd_putstring("  Race Mode0...");
			unclk_TIM6();
			tdelay();
			return;
		}
		if (nextStage == true){
			unclk_TIM6();
			dir = FORWARDS;
			raceMode1();
			return;
		}
	}
}

void raceMode1(){
// line detecting and prox detecting stage

	nextStage = false;

	race_mode = SENSING_LINE_PROX;

	//displayR(LCD_LINE_PROX);



	init_LineSensor();
	init_ProxSensor();
	init_PWM();
	enableMotors();


	if (dir == FORWARDS){
		setDir(1); // set direction of motors
	}else{
		setDir(0);
	}

	init_TIM6(1);

	tdelay();

	setPWM1(PWM_go);
	setPWM2(PWM_go);

	for(;;){

		if ((GPIOA->IDR & SW3) == 0){
			// return to start screen
			lcd_command(CLEAR);
			lcd_putstring("Exiting");
			lcd_command(LINE_TWO);

			if (dir == FORWARDS){ ////////
			lcd_putstring("  Race Mode1..F");
			}else{
				lcd_putstring("  Race Mode1..B");
			}
			unclk_TIM6();
			tdelay();
			return;
		}
		if ((GPIOA->IDR & SW1) == 0){
			display_prox = 1;
		}
		if ((GPIOA->IDR & SW2) == 0){
			display_prox = 0;
		}

		if (nextStage == true){
			unclk_TIM6();
			if (dir == FORWARDS){ ///////
			raceMode2();
			}else {
				raceMode3();
			}
			return;
		}
	}
}

void raceMode2(){
// gripper stage

	nextStage = false;

	race_mode = SENSING_BATON_PROX;

	displayR(LCD_BATON_PROX);

	init_TIM6(1);

	tdelay();
	// TODO:gripper stage


	// enable the gripper motors and turn them on
	disableMotors();

	//enableGripper();
	setPWM1(0);
	setPWM2(0);
	tdelay();	tdelay();	tdelay();
	//setPWM1(1000);
	tdelay();	tdelay();	tdelay();	tdelay();	tdelay();
	//setPWM2(100);
	//tdelay();	tdelay();	tdelay();	tdelay();

	//disableGripper(); /// TODO: possibly don't want to ude this

	for(;;){

		if ((GPIOA->IDR & SW3) == 0){

			lcd_command(CLEAR);
			lcd_putstring("Exiting");
			lcd_command(LINE_TWO);
			lcd_putstring("  Race Mode2...");
			unclk_TIM6();
			tdelay();
			return;
		}

		if (nextStage == true){
			unclk_TIM6();
			dir = BACKWARDS; ////////
			raceMode1(); //////
			return;
		}
	}
}


void raceMode3(){
// ending

	nextStage = false;

	disableMotors();
	setPWM1(0);
	setPWM2(0);

	lcd_command(CLEAR);
	lcd_putstring("Finished...");
	lcd_command(LINE_TWO);
	tdelay();
	tdelay();
	lcd_putstring("Hopefully");
	tdelay();
	tdelay();

	disableMotors();
	//disableGripper(); ///// TODO: check that gripper holds without there being power

}

//====================================================================
// END OF ALL RACING FUNCTIONS
//====================================================================



void displayC(int COMMAND){
	switch(COMMAND){

	case LCD_MAIN:
		lcd_command(CLEAR);
		lcd_putstring("1Strt|Calib");
		lcd_command(LINE_TWO);
		lcd_putstring("  SW0 |SW1");
		break;

	case LCD_CALIBRATION_MODE_SELECT:
		lcd_command(CLEAR);
		lcd_putstring("2Light|Line|Prox");
		lcd_command(LINE_TWO);
		lcd_putstring("   SW0|SW1 |SW2");
		break;

	case LCD_CALIBRATION_LIGHT:
		lcd_command(CLEAR);
		lcd_putstring("2.1Green|Red|Amb");
		lcd_command(LINE_TWO);
		char adc_x[4];
		sprintf(adc_x, "%u", ADC_val); // UNLIKELY THAT ALL THIS WILL WORK
		lcd_putstring1(adc_x,4);
		lcd_putstring(" SW0|SW1|SW2");
		break;

	case LCD_CALIBRATION_LINE:
		lcd_command(CLEAR);
		lcd_putstring("2.2 Line|noLine");
		lcd_command(LINE_TWO);
		char adc_y[4];
		sprintf(adc_y, "%u", ADC_val); // UNLIKELY THAT ALL THIS WILL WORK
		lcd_putstring1(adc_y,4);
		lcd_putstring(" SW0|SW1 ");

		char adc_b[4];
		sprintf(adc_b, "%u", line_adc); // UNLIKELY THAT ALL THIS WILL WORK
		lcd_putstring1(adc_b,4);

		break;

	case LCD_CALIBRATION_PROX:

		lcd_command(CLEAR);
		lcd_putstring("2.3Current Value:");
		lcd_command(LINE_TWO);

		lcd_putstring("params[");

		int index = calc_ProxPotIndex(ADC_proxPot);
		char str[2];
		sprintf(str, "%u", index);
		lcd_putstring(str);

		lcd_putstring("]=");

		char str1[4];
		sprintf(str1, "%u", ADC_val);
		lcd_putstring(str1);

		break;
	}

}

void displayR(int COMMAND){

	switch(COMMAND){

	case LCD_LIGHT:

		lcd_command(CLEAR);
		lcd_putstring("Detecting Light x:");
		lcd_command(LINE_TWO);

		switch(colour_detected){

		case GREEN: // green detected
			lcd_putstring("GREEENNN!!");
//			tdelay();
			nextStage = true;
			break;
		case RED: // red detected
			lcd_putstring("Red detected");
			break;
		case AMBIENT: // ambient detected
			lcd_putstring("No light");
			break;
		}

		break; // END CASE LCD_LIGHT

	case LCD_LINE_PROX:

		lcd_command(CLEAR);
		lcd_putstring("      ");
		char dist[2];
		sprintf(dist, "%u", distance);
		lcd_putstring1(dist, 2);
		lcd_putstring(" cm ");

		// to help with debugging
		char str[4];
		sprintf(str, "%u", ADC_val);
		lcd_putstring(str);

		if (display_prox == 1){
		int ADC_val1 = read_ADC(ADC_CHSELR_LINESENSOR1);
			int ADC_val2 = read_ADC(ADC_CHSELR_LINESENSOR2);
			int ADC_val3 = read_ADC(ADC_CHSELR_LINESENSOR3);

			lcd_command(CLEAR);
			/*
			 * debugging
			 */
			char a[2];
			sprintf(a, "%u", ADC_val1);
			lcd_putstring("1:");
			lcd_putstring1(a, 3);

			char b[2];
			sprintf(b, "%u", ADC_val2);
			lcd_putstring("2:");
			lcd_putstring1(b, 3);

			char c[2];
			sprintf(c, "%u", ADC_val3);
			lcd_putstring("3:");
			lcd_putstring1(c, 3);
		}
		// nicely formatted output of linesensor output
		lcd_command(LINE_TWO);

		char str1[2];
		sprintf(str1, "%u", PWM_L_speed);
		lcd_putstring1(str1,2);
		lcd_putchar('%');


		lcd_putstring("  ");
		if ((lineOutput & LEFT) == LEFT){ lcd_putstring("L");}
		else {lcd_putchar('0');}
		if ((lineOutput & MID) == MID){	lcd_putstring("M");}
		else {lcd_putchar('0');}
		if ((lineOutput & RIGHT) == RIGHT){	lcd_putstring("R");}
		else {lcd_putchar('0');}
		lcd_putstring(" ");

		// read ADC line 1 and also output to screen
		int adc_line = read_ADC(ADC_CHSELR_LINESENSOR1);
				char adc_y[4];
				sprintf(adc_y, "%u", adc_line);
				lcd_putstring1(adc_y,4);

		char str2[2];
		sprintf(str2, "%u", PWM_R_speed);
		lcd_putstring1(str2,2);
		lcd_putchar('%');

		if ((distance < BATON_DISTANCE) & (dir == FORWARDS)){///////
			nextStage = true;
			setPWM1(0);
			setPWM2(0);
		}
		if ((lineOutput == 0b111) & (dir == BACKWARDS)){ /////////
			// TODO: must add some number of times for this to occur
			num_ends++;
			if (num_ends == 3){
			nextStage = true;
			}
		}
		break;

	case LCD_BATON_PROX:

		lcd_command(CLEAR);
		lcd_putstring("Gripping Baton");
		lcd_command(LINE_TWO);
		break;
	}
}
