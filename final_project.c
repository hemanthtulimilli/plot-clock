/**
*
* @file final_project.c
*
* @author Venkata Hemanth Tulimilli and Bala Prashanth Reddy Chappidi
* @copyright Portland State University, 2015
*
* This program is made for the software part of Counter Plotter which writes numbers on a board.
* The hardware part of the project is made with three Xilinx Timer/Counter modules set in PWM mode.
* The PWM library builds on the Timer/Counter drivers provided by Xilinx and encapsulates common PWM functions.
*
* The frequency fed to the Timer/Counter is 50Hz(working Frequency of used servo motor) to drive
* three Servo motors and duty cycle is selected between 4% and 11%.
* This program uses Fixed Interval Timer which is used to implement Delay function.
*
* This program is edited and this edited version helps in calibrating three Servo motors to make the mechanical
* arm move and thus to write desired digits/alphabets with marker using look-up tables
*
* The PWM signals are generated in the hardware and send through JC[4:6] on the Nexys4DDR board.
*
* To switch between Modes of operation, we used Switches[0:2] and Switch[15]
* Switch details:
* Switch[0]		:		Up Counter
* Switch[1]		:		Down Counter
* Switch[2]		:		Time Implementation
* Switch[15]	:		Terminate the Program
*
* Sources used for reference:
* testpwm.c	by Roy Kravitz
*
* -- Venkata Hemanth Tulimilli, Bala Prashanth Reddy Chappidi
*===========================================================================================
******************************************************************************/

/************************ Include Files **************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "xparameters.h"
#include "xintc.h"
#include "xtmrctr.h"
#include "xgpio.h"
#include "mb_interface.h"
#include "platform.h"
#include "Nexys4IO.h"
#include "PMod544IOR2.h"
#include "pwm_tmrctr.h"


/************************** Constant Definitions ****************************/

// Clock frequencies
#define CPU_CLOCK_FREQ_HZ			XPAR_CPU_CORE_CLOCK_FREQ_HZ
#define AXI_CLOCK_FREQ_HZ			XPAR_CPU_M_AXI_DP_FREQ_HZ

// PWM and pulse detect timer parameters
#define PWM_TIMER_DEVICE_ID_1		XPAR_AXI_TIMER_0_DEVICE_ID
#define PWM_TIMER_DEVICE_ID_2   	XPAR_AXI_TIMER_1_DEVICE_ID
#define PWM_TIMER_DEVICE_ID_3   	XPAR_AXI_TIMER_2_DEVICE_ID

// Nexys4IO parameters
#define NX4IO_DEVICE_ID				XPAR_NEXYS4IO_0_DEVICE_ID
#define NX4IO_BASEADDR				XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define NX4IO_HIGHADDR				XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

// Pmod544IO parameters
#define PMDIO_DEVICE_ID				XPAR_PMOD544IOR2_0_DEVICE_ID
#define PMDIO_BASEADDR				XPAR_PMOD544IOR2_0_S00_AXI_BASEADDR
#define PMDIO_HIGHADDR				XPAR_PMOD544IOR2_0_S00_AXI_HIGHADDR

// GPIO1 parameters
#define GPIO_DEVICE_ID				XPAR_AXI_GPIO_0_DEVICE_ID
#define GPIO_INPUT_CHANNEL			1
#define GPIO_OUTPUT_CHANNEL			2

// Interrupt Controller parameters
#define INTC_DEVICE_ID				XPAR_INTC_0_DEVICE_ID
#define FIT_INTERRUPT_ID			XPAR_MICROBLAZE_0_AXI_INTC_FIT_TIMER_0_INTERRUPT_INTR

// PWM interrupt parameters
#define PWM_TIMER_INTERRUPT_ID_1	XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMER_0_INTERRUPT_INTR
#define PWM_TIMER_INTERRUPT_ID_2    XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMER_1_INTERRUPT_INTR
#define PWM_TIMER_INTERRUPT_ID_3    XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMER_2_INTERRUPT_INTR

// Fixed Interval timer - 100 MHz input clock, 40KHz output clock
// FIT_COUNT_1MSEC = FIT_CLOCK_FREQ_HZ * .001
#define FIT_IN_CLOCK_FREQ_HZ		CPU_CLOCK_FREQ_HZ
#define FIT_CLOCK_FREQ_HZ			40000
#define FIT_COUNT					(FIT_IN_CLOCK_FREQ_HZ / FIT_CLOCK_FREQ_HZ)
#define FIT_COUNT_1MSEC				40

// Microblaze output clock - 100MHz
#define MICROBLAZE_CLOCK_FREQ_HZ	100000000  // 100MHz reference frequency for HW *modified

#define PWM_FREQ					50

#define INITIAL_FREQUENCY			50 // Starting from 100Hz for Project 1 *modified
#define INITIAL_DUTY_CYCLE			4.0
#define DUTY_CYCLE_CHANGE			0.1

#define DELAY_VALUE				150
#define TRUE					1
#define FALSE					0
#define ERASE_DELAY_VALUE		200

// buttons masks
#define BTNR_MSK				0x01
#define BTNL_MSK				0x02
#define BTND_MSK				0x04
#define BTNU_MSK				0x08
#define BTNC_MSK				0x10
#define BTNALL_MSK				0x1F

// LCD offsets for counter
#define LCD_LINE1						1
#define LCD_LINE2						2
#define LCD_COUNTER_COLUMN_OFFSET		6
#define LCD_COUNTER_MAX_COUNT			3

// Switch masks
#define INVALID_STATE			0x0000
#define UP_COUNTER_MSK			0x0001
#define DOWN_COUNTER_MSK		0x0002
#define CLOCK_MSK				0x0004
#define QUIT_MSK				0x8000
#define SWITCH_ALL_MSK			0x0007

#define COUNTER_UPDATE_DELAY	 20 // delay between updating of counter values

#define ERASE_VALUES			35  // number of points in the 'erase points' array

#define ERASE_REST_DUTY			7.15 // duty cycle for lift servo during erase


/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/
#define MIN(a, b)  ( ((a) <= (b)) ? (a) : (b) )
#define MAX(a, b)  ( ((a) >= (b)) ? (a) : (b) )

/************************** Variable Definitions ****************************/	
// Microblaze peripheral instances
XIntc 	IntrptCtlrInst;						// Interrupt Controller instance

XTmrCtr	PWMTimerInst1;						// PWM timer instances
XTmrCtr	PWMTimerInst2;
XTmrCtr	PWMTimerInst3;
XTmrCtr	PWMTimerInst;

XGpio	GPIOInst;							// GPIO instance

// The following variables are shared between non-interrupt processing and
// interrupt processing such that they must be global(and declared volatile)
// These variables are controlled by the FIT timer interrupt handler
// "clkfit" toggles each time the FIT interrupt handler is called so its frequency will
// be 1/2 FIT_CLOCK_FREQ_HZ.  timestamp increments every 1msec and is used in delay_msecs()
volatile unsigned int	clkfit;				// clock signal is bit[0] (rightmost) of gpio 0 output port									
volatile unsigned long	timestamp;			// timestamp since the program began

volatile u32			gpio_in;			// GPIO input port

// The following variables are shared between the functions in the program
// such that they must be global

int 					pwm_out;			//pulse width modulated signal for detection in Software

/*-------------------------------------------------------------------------------------------------------------*/
// Look up table for positions to go to a particular position and write.
// @param position is the position of the number i.e.,
// 			0 - Thousands place
// 			1 - Hundreds place
// 			2 - symbol ":"
// 			3 - Tens place
// 			4 - Ones place
//
// @param duty_cycle_value is the value of the dutycycle to be set for motor to go to a particular position
// 			as in seven segment display as shown below
//
//			0-------1		0-------1				0-------1		0-------1
//			|		|		|		|		0		|		|		|		|
//			|		|		|		|		*		|		|		|		|
// 			5-------2		5-------2				5-------2		5-------2
//			|		|		|		|		*		|		|		|		|
//			|		|		|		|		1		|		|		|		|
//			4-------3		4-------3				4-------3		4-------3
//		   position[0]     position[1] position[2] position[3]     position[4]
/*-------------------------------------------------------------------------------------------------------------*/

// motor1_position[position][duty_cycle_value]
float motor1_position[5][6] = {	{9.66, 9.35, 9.86, 10.45, 10.78, 10.22},
								{9.05, 8.72, 9.135, 9.55, 10.15, 9.6},
								{8.77, 9.1, 7.5, 7.5, 7.5, 7.5},
								{8.3, 7.5, 7.95, 8.3, 9.02, 8.66},
								{7.1, 6.4, 6.64, 6.88, 7.7, 7.4}
								};

// motor2_position[position][duty_cycle_value]
float motor2_position[5][6] = {	{9.16, 8.49, 8.37, 8.34, 9.0, 9.05},
								{8.22, 7.6, 7.43, 7.26, 7.88, 8.0},
								{7.17, 7.07, 7.5, 7.5, 7.5, 7.5},
								{7.1, 6.53, 6.245, 5.96, 6.66, 6.88},
								{6.23, 5.76, 5.37, 4.98, 5.5, 5.865}
								};

// motor3 rest and lift duty_cycle_value respectively
float motor3_position[2] = {7.5, 5.0}; // 7.6,5.0 ideal

// motor1 erasing dutycycle values
float motor1_erase[] = {4.5,
						8.3, 9.05,
						9.2, 9.66, 9.35, 9.05, 8.72, 8.3, 7.5, 7.1, 6.4, 6.34,
						9.94,
						6.64,
						10.5,
						7.35, 6.88, 7.7, 8.3, 9.02, 9.55, 10.15, 10.45, 10.78, 11.05,
						6.765,
						10.22,
						6.52,
						9.2,
						11.05, 9.2, 11.05, 9.2,
						4.5};

// motor2 erasing dutycycle values
float motor2_erase[] = {6.25,
						7.1, 8.22,
						9.4, 9.16, 8.49, 8.22, 7.6, 7.1, 6.53, 6.23, 5.76, 6.35,
						9.08,
						5.37,
						9.025,
						5.20, 4.98, 5.5, 5.96, 6.66, 7.26, 7.88, 8.34, 9.0, 9.07,
					    5.175,
					    9.05,
						5.565,
						9.4,
						9.07, 9.4, 9.07, 9.4,
						6.25};

// Letter lookup table for nodes 0 to 5 as in 7-segment display in the same order explained above
// 9 is for invalid node number
// letter[0] - "E"
// letter[1] - "C"
int letter[3][8] = {{1, 0, 5, 2, 5, 4, 3},
					{1, 0, 5, 4, 3, 9, 9}};

// Digit lookup table for nodes 0 to 5 as in 7-segment display in the same order explained above
// 9 is for invalid node number
int digit[10][8] = {{0, 1, 2, 3, 4, 5, 0, 9},
					{1, 2, 3, 9, 9, 9, 9, 9},
					{0, 1, 2, 5, 4, 3, 9, 9},
					{0, 1, 2, 5, 2, 3, 4, 9},
					{0, 5, 2, 1, 2, 3, 9, 9},
					{1, 0, 5, 2, 3, 4, 9, 9},
					{1, 0, 5, 4, 3, 2, 5, 9},
					{0, 1, 4, 9, 9, 9, 9, 9},
					{5, 0, 1, 2, 3, 4, 5, 2},
					{2, 5, 0, 1, 2, 3, 4, 9}
					};

// number of nodes to cover for digits starting from 0 to 9
int digit_count[10] = {7, 3, 6, 7, 6, 6, 7, 3, 8, 7};

// Global variables which are used to make the application work
u8 initial_state_flag_reg = 0x07;
u8 first_run_flag_reg = 0x07;
u16	sw;
int display_value[3][5] = {{0, 0, 0, 0, 0}, {9, 9, 9, 9, 9}, {0, 0, 0, 0, 0}};
int clock_count = 0;
int counter_count = 0;
int hour = 0, min = 0, sec = 0;
int write_flag = TRUE, clock_update_flag = TRUE, counter_update_flag = TRUE;
int counter[3][4] = {{0,0,0,0}, {9,9,9,9}, {0,0,0,0}};
int up_counter_init_value = 0;
int up_counter_value = 0;
int down_counter_init_value = 9999;
int down_counter_value = 9999;
int update_lcd_flag = TRUE;
int init_up_counter_value = 0, init_down_counter_value = 0;

/*---------------------------------------------------------------------------*/					
int						debugen = 0;		// debug level/flag
/*---------------------------------------------------------------------------*/
		
/************************** Function Prototypes ******************************/
int				do_init(void);											// initialize system
void			delay_msecs(unsigned int);								// busy-wait delay for "msecs" miliseconds
void			FIT_Handler(void);										// fixed interval timer interrupt handler
void			update_lcd(void);										// update LCD for current values
void			course_write(void);										// writes greeting message function
void 			PositionSet(float, float);								// sets position of the mechanical arm to a position on the board
void			lift_hand(void);										// it lifts marker off the board
void			rest_hand(void);										// places marker on the board
void 			display_symbol(void);									// writes ":" on a particular position
void 			display_num(int, int);									// writes numbers in which ever position we want
void 			erase(void);											// erases the board
void 			set_initial_state(u8);									// initializes the start value for all the modes
void 			update_up_count(void);									// updates Up Count value
void 			update_down_count(void);								// updates Down Count value
void 			update_clk(void);										// updates Time

/************************** MAIN PROGRAM ************************************/
int main()
{
	XStatus 	status;
	bool		done = FALSE;
	int 		i;
	
	init_platform();

	// initialize devices and set up interrupts, etc.
 	status = do_init();
 	if (status != XST_SUCCESS)
 	{
 		PMDIO_LCD_setcursor(1,0);
 		PMDIO_LCD_wrstring("****** ERROR *******");
 		PMDIO_LCD_setcursor(2,0);
 		PMDIO_LCD_wrstring("INIT FAILED- EXITING");
 		exit(XST_FAILURE);
 	}
 	
	// initialize the global variables
	timestamp = 0;							
	clkfit = 0;
    
	// start the three PWM timers and kick of the processing by enabling the Microblaze interrupt
	PWM_SetParams(&PWMTimerInst1, PWM_FREQ, 7.75);
	PWM_Start(&PWMTimerInst1);

	PWM_SetParams(&PWMTimerInst2, PWM_FREQ, 9.5);
	PWM_Start(&PWMTimerInst2);

	PWM_SetParams(&PWMTimerInst3, PWM_FREQ, 7.0);
	PWM_Start(&PWMTimerInst3);
    microblaze_enable_interrupts();
    
	// display the greeting   
    PMDIO_LCD_setcursor(1,0);
    PMDIO_LCD_wrstring("    ECE 544    ");
	PMDIO_LCD_setcursor(2,0);
	PMDIO_LCD_wrstring(" Final Project ");
	NX4IO_setLEDs(0x0000FFFF);
	delay_msecs(1800);
	NX4IO_setLEDs(0x00000000);

    PMDIO_LCD_setcursor(1,0);
    PMDIO_LCD_wrstring("Counter Plotter");
	PMDIO_LCD_setcursor(2,0);
	PMDIO_LCD_wrstring(" by.....       ");
	NX4IO_setLEDs(0x0000FFFF);
	delay_msecs(1800);
	NX4IO_setLEDs(0x00000000);

    PMDIO_LCD_setcursor(1,0);
    PMDIO_LCD_wrstring("    VHT, BPC   ");
	PMDIO_LCD_setcursor(2,0);
	PMDIO_LCD_wrstring("    ASY, SK    ");
	NX4IO_setLEDs(0x0000FFFF);
	delay_msecs(1500);
	NX4IO_setLEDs(0x00000000);

	// write "ECE" "544" on the board
	course_write();
		
	// write the static text to the display
    // turn off the LEDs and clear the seven segment display
    NX4IO_setLEDs(0x00000000);
    NX410_SSEG_setAllDigits(SSEGLO, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
    NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
      
    // main loop
	do
	{ 
		sw = NX4IO_getSwitches();
		// if reset switch(switch 15) is ON, it will exit the main loop
		if(sw & QUIT_MSK)
		{
			done = TRUE;
		}
		// if switch 0 - up counter
		// or switch 1 - down counter
		// or switch 2 - time
		// is ON, it will write the corresponding values
		// if no switch is ON, it will display modes
		switch(sw & SWITCH_ALL_MSK)
		{
		case INVALID_STATE:
			initial_state_flag_reg = 0x07;
			PMDIO_LCD_setcursor(LCD_LINE1, 0);
			PMDIO_LCD_wrstring("Pls. select mode");
			PMDIO_LCD_setcursor(LCD_LINE2, 0);
			PMDIO_LCD_wrstring("0:UC 1:DC 2:CLK ");
			init_up_counter_value = 0;
			init_down_counter_value = 0;
			hour = 0;
			min = 0;
			sec = 0;
			for(i = 0; i < 4; i++)
			{
				counter[0][i] = 0;
				counter[1][i] = 0;
				counter[2][i] = 0;
			}
			break;
		// up counter Implementation
		case UP_COUNTER_MSK:
			if(initial_state_flag_reg & UP_COUNTER_MSK)
			{
				set_initial_state(UP_COUNTER_MSK);
			}
			if(sw == UP_COUNTER_MSK)
			{
				update_lcd();
				update_up_count();
				if((write_flag) || (first_run_flag_reg & UP_COUNTER_MSK))
				{
					first_run_flag_reg &= ~(UP_COUNTER_MSK);
					update_lcd();
					erase();
					for(i = 0; i < 5; i++)
					{
						if(i != 2)
						{
							display_num(i, display_value[0][i]);			// number depends on hours or minutes..
						}
					}
					write_flag = FALSE;
				}
			}
				break;
		// Down Counter Implementation
		case DOWN_COUNTER_MSK:
			if(initial_state_flag_reg & DOWN_COUNTER_MSK)
			{
				set_initial_state(DOWN_COUNTER_MSK);
			}
			if(sw == DOWN_COUNTER_MSK)
			{
				update_lcd();
				update_down_count();
				if((write_flag) || (first_run_flag_reg & DOWN_COUNTER_MSK))
				{
					first_run_flag_reg &= ~(DOWN_COUNTER_MSK);
					update_lcd();
					erase();
					for(i = 0; i < 5; i++)
					{
						if(i != 2)
						{
							display_num(i, display_value[1][i]);			// number depends on hours or minutes..
						}
					}
					write_flag = FALSE;
				}
			}
			break;
		// Time Counting Implementation
		case CLOCK_MSK:
			if(initial_state_flag_reg & CLOCK_MSK)
			{
				set_initial_state(CLOCK_MSK);
			}
			if(sw == CLOCK_MSK)
			{
				update_lcd();
				update_clk();
				if((write_flag) || (first_run_flag_reg & CLOCK_MSK))
				{
					first_run_flag_reg &= ~(CLOCK_MSK);
					update_lcd();
					erase();
					for(i = 0; i < 5; i++)
					{
						if(i == 2)
						{
							display_symbol();
						}
						else
						{
							display_num(i, display_value[2][i]);			// number depends on hours or minutes..
						}
					}
					write_flag = FALSE;
				}
			}
			break;
		}
	} while (!done);
	
	// we're done,  say goodbye
	xil_printf("\nCounter Plotter Terminated!!!\n\n");
	PMDIO_LCD_setcursor(LCD_LINE1, 0);
	PMDIO_LCD_wrstring("Counter  Plotter");
	PMDIO_LCD_setcursor(LCD_LINE2, 0);
	PMDIO_LCD_wrstring("  Terminated!!  ");
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_B, CC_LCY, CC_E, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGLO, CC_B, CC_LCY, CC_E, CC_BLANK, DP_NONE);
	delay_msecs(500);
    NX410_SSEG_setAllDigits(SSEGLO, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
    NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
    delay_msecs(500);
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_B, CC_LCY, CC_E, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGLO, CC_B, CC_LCY, CC_E, CC_BLANK, DP_NONE);
	delay_msecs(500);
	NX410_SSEG_setAllDigits(SSEGLO, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
	delay_msecs(500);
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_B, CC_LCY, CC_E, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGLO, CC_B, CC_LCY, CC_E, CC_BLANK, DP_NONE);
	delay_msecs(1000);
	NX410_SSEG_setAllDigits(SSEGLO, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
	PMDIO_LCD_clrd();
	cleanup_platform();
	exit(0);
 }


/**************************** HELPER FUNCTIONS ******************************/
		
/****************************************************************************/
/**
* initialize the system
* 
* This function is executed once at start-up and after resets.  It initializes
* the peripherals and registers the interrupt handler(s)
*****************************************************************************/
int do_init(void)
{
	int status;				// status from Xilinx Lib calls
	
	// initialize the Nexys4IO and Pmod544IO hardware and drivers
	// rotary encoder is set to increment from 0 by DUTY_CYCLE_CHANGE 
 	status = NX4IO_initialize(NX4IO_BASEADDR);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	status = PMDIO_initialize(PMDIO_BASEADDR);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	
	// successful initialization.  Set the rotary encoder
	// to increment from 0 by DUTY_CYCLE_CHANGE counts per
	// rotation.
 	PMDIO_ROT_init(DUTY_CYCLE_CHANGE, TRUE);
	PMDIO_ROT_clear();
	
	// initialize the GPIO instance
	status = XGpio_Initialize(&GPIOInst, GPIO_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	// GPIO channel 1 is an 8-bit input port.  bit[7:1] = reserved, bit[0] = PWM output (for duty cycle calculation)
	// GPIO channel 2 is an 8-bit output port.  bit[7:1] = reserved, bit[0] = FIT clock
	XGpio_SetDataDirection(&GPIOInst, GPIO_OUTPUT_CHANNEL, 0xFE);

	// initialize the PWM timer/counter instance but do not start it
	// do not enable PWM interrupts.  Clock frequency is the AXI clock frequency
	status = PWM_Initialize(&PWMTimerInst1, PWM_TIMER_DEVICE_ID_1, FALSE, AXI_CLOCK_FREQ_HZ);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	status = PWM_Initialize(&PWMTimerInst2, PWM_TIMER_DEVICE_ID_2, FALSE, AXI_CLOCK_FREQ_HZ);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	status = PWM_Initialize(&PWMTimerInst3, PWM_TIMER_DEVICE_ID_3, FALSE, AXI_CLOCK_FREQ_HZ);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	
	// initialize the interrupt controller
	status = XIntc_Initialize(&IntrptCtlrInst, INTC_DEVICE_ID);
    if (status != XST_SUCCESS)
    {
       return XST_FAILURE;
    }

	// connect the fixed interval timer (FIT) handler to the interrupt
    status = XIntc_Connect(&IntrptCtlrInst, FIT_INTERRUPT_ID,
                           (XInterruptHandler)FIT_Handler,
                           (void *)0);
    if (status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }
 
	// start the interrupt controller such that interrupts are enabled for
	// all devices that cause interrupts.
    status = XIntc_Start(&IntrptCtlrInst, XIN_REAL_MODE);
    if (status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }

	// enable the FIT interrupt
    XIntc_Enable(&IntrptCtlrInst, FIT_INTERRUPT_ID);

    // set the duty cycles for RGB1.  The channels will be enabled/disabled
    // in the FIT interrupt handler.  Red and Blue make purple
    NX4IO_RGBLED_setDutyCycle(RGB1, 64, 0, 64);
    NX4IO_RGBLED_setChnlEn(RGB1, FALSE, FALSE, FALSE);

	return XST_SUCCESS;
}
		

/****************************************************************************/
/**
* delay execution for "n" msecs
* 
* Uses a busy-wait loop to delay execution.  Timing is approximate but we're not 
*  looking for precision here, just a uniform delay function.  The function uses the 
*  global "timestamp" which is incremented every msec by FIT_Handler().
*
*****************************************************************************/
void delay_msecs(unsigned int msecs)
{
	unsigned long target;

	if ( msecs == 0 )
	{
		return;
	}
	target = timestamp + msecs;
	while ( timestamp != target )
	{
		// spin until delay is over
	}
}



 
/****************************************************************************/
/**
 * update the display on the LCD according to the mode selected
 * 
 * Updating an LCD display is based on the mode and
 * there are only some values which are being updated every time
 *
 *****************************************************************************/

void update_lcd()
{
	int i;
	if(update_lcd_flag)
	{
		switch(sw & SWITCH_ALL_MSK)
		{
		case UP_COUNTER_MSK:
			PMDIO_LCD_setcursor(LCD_LINE1, 0);
			PMDIO_LCD_wrstring("   Up Counter   ");
			PMDIO_LCD_setcursor(LCD_LINE2, 0);
			PMDIO_LCD_wrstring("                ");
			for(i = 0; i < 5; i++)
			{
				if(i < 2)
				{
					PMDIO_LCD_setcursor(LCD_LINE2, LCD_COUNTER_COLUMN_OFFSET + i);
					PMDIO_LCD_putnum(display_value[0][i], 10);
				}
				else if(i > 2)
				{
					PMDIO_LCD_setcursor(LCD_LINE2, LCD_COUNTER_COLUMN_OFFSET + i - 1);
					PMDIO_LCD_putnum(display_value[0][i], 10);
				}

			}
			break;
		case DOWN_COUNTER_MSK:
			PMDIO_LCD_setcursor(LCD_LINE1, 0);
			PMDIO_LCD_wrstring("  down Counter  ");
			PMDIO_LCD_setcursor(LCD_LINE2, 0);
			PMDIO_LCD_wrstring("                ");
			for(i = 0; i < 5; i++)
			{
				if(i < 2)
				{
					PMDIO_LCD_setcursor(LCD_LINE2, LCD_COUNTER_COLUMN_OFFSET + i);
					PMDIO_LCD_putnum(display_value[1][i], 10);
				}
				else if(i > 2)
				{
					PMDIO_LCD_setcursor(LCD_LINE2, LCD_COUNTER_COLUMN_OFFSET + i - 1);
					PMDIO_LCD_putnum(display_value[1][i], 10);
				}
			}
			break;
		case CLOCK_MSK:
			PMDIO_LCD_setcursor(LCD_LINE1, 0);
			PMDIO_LCD_wrstring("      TIME      ");
			PMDIO_LCD_setcursor(LCD_LINE2, 0);
			PMDIO_LCD_wrstring("   HH   :   MM  ");
			for(i = 0; i < 5; i++)
			{
				if(i < 2)
				{
					PMDIO_LCD_setcursor(LCD_LINE2, LCD_COUNTER_COLUMN_OFFSET + i);
					PMDIO_LCD_putnum(display_value[2][i], 10);
				}
				else if(i > 2)
				{
					PMDIO_LCD_setcursor(LCD_LINE2, LCD_COUNTER_COLUMN_OFFSET + i);
					PMDIO_LCD_putnum(display_value[2][i], 10);
				}

			}
			break;
		}
	}
}


	
/**************************** INTERRUPT HANDLERS ******************************/

/****************************************************************************/
/**
* Fixed interval timer interrupt handler 
*  
* updates the global "timestamp" every millisecond.  "timestamp" is used for the delay_msecs() function
* and as a time stamp for data collection and reporting.  Toggles the FIT clock which can be used as a visual
* indication that the interrupt handler is being called.  Also makes RGB1 a PWM duty cycle indicator
*
 *****************************************************************************/
void FIT_Handler(void)
{
		
	static	int			ts_interval = 0;			// interval counter for incrementing timestamp

	// toggle FIT clock
	clkfit ^= 0x01;
	XGpio_DiscreteWrite(&GPIOInst, GPIO_OUTPUT_CHANNEL, clkfit);	

	// update timestamp	
	ts_interval++;	
	if (ts_interval > FIT_COUNT_1MSEC)
	{
		timestamp++;
		clock_count++;
		ts_interval = 1;
	}

	// updating the time to set it to 1 sec timing and
	// using that in the clock to update the time and
	// displaying on the LCD as well as writing on the plate
	clock_count ++;
	if (clock_count >= 1000)
	{
		counter_count++;
		if (counter_count > COUNTER_UPDATE_DELAY)
		{
			counter_count = 0;
			counter_update_flag =TRUE;
		}
		clock_count = 0;
		clock_update_flag = TRUE;
	}
}

/********************************User defined Functions********************************/

/****************************************************************************/
/**	PositionSet()
* This function sets the two motors which are used to write to a particular position
*
 *****************************************************************************/

void PositionSet(float motor1_duty, float motor2_duty)
{
	XStatus status;
	status = PWM_SetParams(&PWMTimerInst1, PWM_FREQ, motor1_duty);
	if (status == XST_SUCCESS)
	{
 		PWM_Start(&PWMTimerInst1);
	}

	status = PWM_SetParams(&PWMTimerInst2, PWM_FREQ, motor2_duty);
	if (status == XST_SUCCESS)
	{
		PWM_Start(&PWMTimerInst2);
	}
return;
}

/****************************************************************************/
/**	lift_hand()
* This function sets the third motor which is used to move the writing arm UP
* so that it can move into another position without writing anything on the
* writing plate
*
 *****************************************************************************/

void lift_hand(void)
{
	XStatus status;
	status = PWM_SetParams(&PWMTimerInst3, PWM_FREQ, motor3_position[1]);
	if (status == XST_SUCCESS)
	{
		PWM_Start(&PWMTimerInst3);
	}
	return;
}

/****************************************************************************/
/**	rest_hand()
* This function sets the third motor which is used to move the writing arm DOWN
* so that it can move into another position for writing anything on the
* writing plate
*
 *****************************************************************************/

void rest_hand(void)
{
	XStatus status;
	status = PWM_SetParams(&PWMTimerInst3, PWM_FREQ, motor3_position[0]);
	if (status == XST_SUCCESS)
	{
		PWM_Start(&PWMTimerInst3);
	}
	return;
}

/****************************************************************************/
/**	rest_hand_symbol()
* This function sets the third motor which is used to move the writing arm DOWN
* for writing symbol ":"
*
 *****************************************************************************/

void rest_hand_symbol(void)
{
	XStatus status;
	status = PWM_SetParams(&PWMTimerInst3, PWM_FREQ, ERASE_REST_DUTY);
	if (status == XST_SUCCESS)
	{
		PWM_Start(&PWMTimerInst3);
	}
	return;
}

/****************************************************************************/
/**	display_num(int, int)
* This function displays a digit just as seen in a seven segment display
* in a particular row
*
* @param	temp_li is the position in which it should write
*
* @param	num is the digit which is to be displayed in that particular position
*
 *****************************************************************************/

void display_num(int temp_li, int num)
{
	int i;
	lift_hand();
	delay_msecs(DELAY_VALUE);
	PositionSet(motor1_position[temp_li][digit[num][0]], motor2_position[temp_li][digit[num][0]]);
	delay_msecs(DELAY_VALUE);
	rest_hand();
	delay_msecs(DELAY_VALUE);
	for(i = 1; i < digit_count[num]; i++)
	{
		PositionSet(motor1_position[temp_li][digit[num][i]], motor2_position[temp_li][digit[num][i]]);
		delay_msecs(DELAY_VALUE);
	}
	lift_hand();
	delay_msecs(DELAY_VALUE);
}

/****************************************************************************/
/**	display_symbol()
* This function is used to write the symbol in the middle position
* when time mode is set
*
 *****************************************************************************/

void display_symbol(void)
{
	int i;
	for(i = 0; i < 2; i++)
		{
		lift_hand();
		delay_msecs(DELAY_VALUE);
		PositionSet(motor1_position[2][i], motor2_position[2][i]);
		delay_msecs(DELAY_VALUE);
		rest_hand_symbol();
		delay_msecs(DELAY_VALUE);
		}
	lift_hand();
	delay_msecs(DELAY_VALUE);
}

/****************************************************************************/
/**	update_clk()
* This function updates the clock every one second and enables a flag
* such that main loop can update the time on the display plane we have under the marker
*
 *****************************************************************************/

void update_clk(void)
{
	if(clock_update_flag)
	{
		sec++;
		write_flag = FALSE;
		if(sec >= 60)
		{
			min++;
			sec = 0;
			write_flag = TRUE;
			update_lcd_flag = TRUE;
		}
		if(min >= 60)
		{
			hour++;
			min = 0;
		}
		if(hour >= 24)
		{
			hour = 0;
		}
		display_value[2][0] = hour / 10;
		display_value[2][1] = hour % 10;
		display_value[2][2] = 0x00;
		display_value[2][3] = min / 10;
		display_value[2][4] = min % 10;
		clock_update_flag = FALSE;
	}
	return;
}

/****************************************************************************/
/**	erase()
* This function erases the previous letters on the board when ever it is called.
*
 *****************************************************************************/

void erase(void)
{
	int i, j;
	lift_hand();
	delay_msecs(ERASE_DELAY_VALUE);
	PositionSet(motor1_erase[0], motor2_erase[0]);
	delay_msecs(ERASE_DELAY_VALUE);
	rest_hand();
	delay_msecs(ERASE_DELAY_VALUE);
	for(j = 0; j < ERASE_ITERATION; j++)
	{
		for(i = 1; i < (ERASE_VALUES - 1); i++)
		{
			PositionSet(motor1_erase[i], motor2_erase[i]);
			delay_msecs(ERASE_DELAY_VALUE);
		}
	}
	PositionSet(motor1_erase[ERASE_VALUES - 1], motor2_erase[ERASE_VALUES - 1]);
	delay_msecs(ERASE_DELAY_VALUE);
	lift_hand();
	delay_msecs(ERASE_DELAY_VALUE);
	return;
}

/****************************************************************************/
/**	set_initial_state(u8)
* This function updates the initial value when first set using the mode of operation.
* This function is accessed whenever we change from one mode to any other.
*
* @param	mode is used to determine which mode is it in.
*
 *****************************************************************************/

void set_initial_state(u8 mode)
{
	u8 btns;
	int column = LCD_COUNTER_COLUMN_OFFSET;
	int temp_display = 0, temp_li;
	int i;
	btns = NX4IO_getBtns();


	// ------------------setting LCD to initial position based up on mode------------------//
	// mode 0 = up counter
	// mode 1 = down
	// mode 2 = clock
	initial_state_flag_reg ^= mode;

	// set initial greeting according to the mode
	if(mode == UP_COUNTER_MSK)
	{
		for(i = 0; i < 4; i++)
		{
			counter[1][i] = 0;
			counter[2][i] = 0;
		}
		initial_state_flag_reg = 0x06;
		init_down_counter_value = 0;
		hour = 0;
		min = 0;
		sec = 0;
		temp_li = 0;
		PMDIO_LCD_setcursor(LCD_LINE1, 0);
		PMDIO_LCD_wrstring("set final val ! ");
		PMDIO_LCD_setcursor(LCD_LINE2, 0);
		PMDIO_LCD_wrstring(" for up counter ");
		delay_msecs(3000);
		PMDIO_LCD_setcursor(LCD_LINE1, 0);
		PMDIO_LCD_wrstring("      0000      ");
		PMDIO_LCD_setcursor(LCD_LINE2, 0);
		PMDIO_LCD_wrstring("      ^         ");
	}
	else if(mode == DOWN_COUNTER_MSK)
	{
		for(i = 0; i < 4; i++)
		{
			counter[0][i] = 0;
			counter[2][i] = 0;
		}
		initial_state_flag_reg = 0x05;
		init_up_counter_value = 0;
		hour = 0;
		min = 0;
		sec = 0;
		temp_li = 1;
		PMDIO_LCD_setcursor(LCD_LINE1, 0);
		PMDIO_LCD_wrstring("set initial val!");
		PMDIO_LCD_setcursor(LCD_LINE2, 0);
		PMDIO_LCD_wrstring("for down counter");
		delay_msecs(3000);
		PMDIO_LCD_setcursor(LCD_LINE1, 0);
		PMDIO_LCD_wrstring("      9999      ");
		PMDIO_LCD_setcursor(LCD_LINE2, 0);
		PMDIO_LCD_wrstring("      ^         ");
	}
	else if(mode == CLOCK_MSK)
	{
		for(i = 0; i < 4; i++)
		{
			counter[0][i] = 0;
			counter[1][i] = 0;
		}
		initial_state_flag_reg = 0x03;
		init_up_counter_value = 0;
		init_down_counter_value = 0;
		temp_li = 2;
		PMDIO_LCD_setcursor(LCD_LINE1, 0);
		PMDIO_LCD_wrstring("set current time");
		PMDIO_LCD_setcursor(LCD_LINE2, 0);
		PMDIO_LCD_wrstring("  in HH:MM mode ");
		delay_msecs(3000);
		PMDIO_LCD_setcursor(LCD_LINE1, 0);
		PMDIO_LCD_wrstring("      00:00     ");
		PMDIO_LCD_setcursor(LCD_LINE2, 0);
		PMDIO_LCD_wrstring("      ^         ");
	}
	while((btns & BTNALL_MSK) != BTNC_MSK)
	{
		write_flag = TRUE;
		btns = NX4IO_getBtns();
		sw = NX4IO_getSwitches();

		if(sw == INVALID_STATE)
		{
			return;
		}
		switch(btns & BTNALL_MSK)
		{
		case BTNR_MSK:
			column ++;
			if(temp_li < 2)
			{
				if (column > (LCD_COUNTER_COLUMN_OFFSET + LCD_COUNTER_MAX_COUNT))
				{
					column = LCD_COUNTER_COLUMN_OFFSET;
				}
				PMDIO_LCD_setcursor(LCD_LINE2, 0);
				PMDIO_LCD_wrstring("                ");
				PMDIO_LCD_setcursor(LCD_LINE2, column);
				PMDIO_LCD_wrchar(0x5E);
			}
			else if(temp_li == 2)
			{
				if (column > (LCD_COUNTER_COLUMN_OFFSET + LCD_COUNTER_MAX_COUNT))
				{
					column = LCD_COUNTER_COLUMN_OFFSET;
				}
				PMDIO_LCD_setcursor(LCD_LINE2, 0);
				PMDIO_LCD_wrstring("                ");
				if(column < 8)
				{
					PMDIO_LCD_setcursor(LCD_LINE2, column);
				}
				else
				{
					PMDIO_LCD_setcursor(LCD_LINE2, column + 1);
				}
				PMDIO_LCD_wrchar(0x5E);
			}
			break;
		case BTNL_MSK:
			column --;
			if(temp_li < 2)
			{
				if (column < LCD_COUNTER_COLUMN_OFFSET)
				{
				column = LCD_COUNTER_COLUMN_OFFSET + LCD_COUNTER_MAX_COUNT;
				}
				PMDIO_LCD_setcursor(LCD_LINE2, 0);
				PMDIO_LCD_wrstring("                ");
				PMDIO_LCD_setcursor(LCD_LINE2, column);
				PMDIO_LCD_wrchar(0x5E);
			}
			else if(temp_li == 2)
			{
				if (column < LCD_COUNTER_COLUMN_OFFSET)
				{
				column = LCD_COUNTER_COLUMN_OFFSET + LCD_COUNTER_MAX_COUNT;
				}
				PMDIO_LCD_setcursor(LCD_LINE2, 0);
				PMDIO_LCD_wrstring("                ");
				if(column < 8)
				{
					PMDIO_LCD_setcursor(LCD_LINE2, column);
				}
				else
				{
					PMDIO_LCD_setcursor(LCD_LINE2, column + 1);
				}
				PMDIO_LCD_wrchar(0x5E);
			}
			break;
		case BTNU_MSK:
			if(temp_li < 2)
			{
				switch(column)
				{
				case 6:
					counter[temp_li][0] += 1;
					if (counter[temp_li][0] > 9)
					{
						counter[temp_li][0] = 0;
					}
					break;
				case 7:
					counter[temp_li][1] += 1;
					if (counter[temp_li][1] > 9)
					{
						counter[temp_li][1] = 0;
					}
					break;
				case 8:
					counter[temp_li][2] += 1;
					if (counter[temp_li][2] > 9)
					{
						counter[temp_li][2] = 0;
					}
				break;
				case 9:
					counter[temp_li][3] += 1;
					if (counter[temp_li][3] > 9)
					{
						counter[temp_li][3] = 0;
					}
					break;
				}
				for(i = 0; i < 4; i++)
				{
					PMDIO_LCD_setcursor(LCD_LINE1, LCD_COUNTER_COLUMN_OFFSET + i);
					PMDIO_LCD_putnum(counter[temp_li][i], 10);
				}
			}
			else if (temp_li == 2)
			{
				switch(column)
				{
				case 6:
					counter[temp_li][0] += 1;
					if ((counter[temp_li][0] == 2) && (counter[temp_li][1] > 3))
						counter[temp_li][1] = 3;
					if (counter[temp_li][0] > 2)
					{
						counter[temp_li][0] = 0;
					}
					break;
				case 7:
					counter[temp_li][1] += 1;
					if ((counter[temp_li][1] > 3) && (counter[temp_li][0] == 2))
					{
						counter[temp_li][1] = 0;
					}
					else if(((counter[temp_li][1] > 9) && (counter[temp_li][0] != 2)))
					{
						counter[temp_li][1] = 0;
					}
					break;
				case 8:
					counter[temp_li][2] += 1;
					if (counter[temp_li][2] > 5)
					{
						counter[temp_li][2] = 0;
					}
					break;
				case 9:
					counter[temp_li][3] += 1;
					if (counter[temp_li][3] > 9)
					{
						counter[temp_li][3] = 0;
					}
					break;
				}
				for(i = 0; i < 4; i++)
				{
					if(i < 2)
					{
						PMDIO_LCD_setcursor(LCD_LINE1, LCD_COUNTER_COLUMN_OFFSET + i);
						PMDIO_LCD_putnum(counter[temp_li][i], 10);
					}
					else if(i < 4)
					{
						PMDIO_LCD_setcursor(LCD_LINE1, LCD_COUNTER_COLUMN_OFFSET + i + 1);
						PMDIO_LCD_putnum(counter[temp_li][i], 10);
					}
				}
			}
			break;
		case BTND_MSK:
			if(temp_li < 2)
			{
				switch(column)
				{
				case 6:
					counter[temp_li][0] -= 1;
					if (counter[temp_li][0] < 0)
					{
						counter[temp_li][0] = 9;
					}
					break;
				case 7:
					counter[temp_li][1] -= 1;
					if (counter[temp_li][1] < 0)
					{
						counter[temp_li][1] = 9;
					}
					break;
				case 8:
					counter[temp_li][2] -= 1;
					if (counter[temp_li][2] < 0)
					{
						counter[temp_li][2] = 9;
					}
				break;
				case 9:
					counter[temp_li][3] -= 1;
					if (counter[temp_li][3] < 0)
					{
						counter[temp_li][3] = 9;
					}
					break;
				}
				for(i = 0; i < 4; i++)
				{
					PMDIO_LCD_setcursor(LCD_LINE1, LCD_COUNTER_COLUMN_OFFSET + i);
					PMDIO_LCD_putnum(counter[temp_li][i], 10);
				}
			}
			else if (temp_li == 2)
			{
				switch(column)
				{
				case 6:
					counter[temp_li][0] -= 1;
					if (counter[temp_li][0] < 0)
					{
						counter[temp_li][0] = 2;
					}
					break;
				case 7:
					counter[temp_li][1] -= 1;
					if ((counter[temp_li][1] < 0) && (counter[temp_li][0] == 2))
					{
						counter[temp_li][1] = 3;
					}
					else if(((counter[temp_li][1] < 0) && (counter[temp_li][0] != 2)))
					{
						counter[temp_li][1] = 9;
					}
					break;
				case 8:
					counter[temp_li][2] -= 1;
					if (counter[temp_li][2] < 0)
					{
						counter[temp_li][2] = 5;
					}
				break;
				case 9:
					counter[temp_li][3] -= 1;
					if (counter[temp_li][3] < 0)
					{
						counter[temp_li][3] = 9;
					}
					break;
				}
				for(i = 0; i < 4; i++)
				{
					if(i < 2)
					{
						PMDIO_LCD_setcursor(LCD_LINE1, LCD_COUNTER_COLUMN_OFFSET + i);
						PMDIO_LCD_putnum(counter[temp_li][i], 10);
					}
					else if(i < 4)
					{
						PMDIO_LCD_setcursor(LCD_LINE1, LCD_COUNTER_COLUMN_OFFSET + i + 1);
						PMDIO_LCD_putnum(counter[temp_li][i], 10);
					}
				}
			}
			break;
		}
		while(btns & (BTNALL_MSK ^ BTNC_MSK))
		{
			btns = NX4IO_getBtns();
		}
	}
	while(btns & BTNC_MSK)
	{
		btns = NX4IO_getBtns();
	}
	for(i = 0; i < 4; i++)
	{
		temp_display = (temp_display * 10) + counter[temp_li][i];
	}
	if (temp_li == 2)
	{
		hour = temp_display / 100;
		min = temp_display % 100;
		sec = 0;
		PMDIO_LCD_setcursor(LCD_LINE1, 0);
		PMDIO_LCD_wrstring("The time set is:");
		PMDIO_LCD_setcursor(LCD_LINE2, 0);
		PMDIO_LCD_wrstring("    HH  :  MM   ");
		for(i = 0; i < 4; i++)
		{
			if(i < 2)
			{
				PMDIO_LCD_setcursor(LCD_LINE2, LCD_COUNTER_COLUMN_OFFSET + i);
				PMDIO_LCD_putnum(counter[temp_li][i], 10);
			}
			else if(i < 4)
			{
				PMDIO_LCD_setcursor(LCD_LINE2, LCD_COUNTER_COLUMN_OFFSET + i + 1);
				PMDIO_LCD_putnum(counter[temp_li][i], 10);
			}
		}
	}
	else if(temp_li == 1)
	{
		init_down_counter_value = temp_display;
		down_counter_value = temp_display;
		PMDIO_LCD_setcursor(LCD_LINE1, 0);
		PMDIO_LCD_wrstring("DN count set to:");
		PMDIO_LCD_setcursor(LCD_LINE2, 0);
		PMDIO_LCD_wrstring("                ");
		for(i = 0; i < 4; i++)
		{
			PMDIO_LCD_setcursor(LCD_LINE2, LCD_COUNTER_COLUMN_OFFSET + i);
			PMDIO_LCD_putnum(counter[temp_li][i], 10);
		}
	}
	else if(temp_li == 0)
	{
		init_up_counter_value = temp_display;
		up_counter_value = temp_display;
		PMDIO_LCD_setcursor(LCD_LINE1, 0);
		PMDIO_LCD_wrstring("UP count set to:");
		PMDIO_LCD_setcursor(LCD_LINE2, 0);
		PMDIO_LCD_wrstring("                ");
		for(i = 0; i < 4; i++)
		{
			PMDIO_LCD_setcursor(LCD_LINE2, LCD_COUNTER_COLUMN_OFFSET + i);
			PMDIO_LCD_putnum(counter[temp_li][i], 10);
		}
	}
	delay_msecs(3000);
	update_lcd_flag = TRUE;
	write_flag = TRUE;
	clock_count = 0;
	counter_count = 0;
	return;

}

/****************************************************************************/
/**	update_up_count()
* This function updates the counter value for up counter by incrementing by one
* every COUNTER_UPDATE_DELAY time period and then updating the data to be written
*
* This function will be incrementing the value until the user given value.
* If the maximum value is reached, it will restart from 0.
*
 *****************************************************************************/

void update_up_count(void)
{
	int temp_li;
	if(counter_update_flag)
	{
		up_counter_value++;
		if(up_counter_value > init_up_counter_value)
		{
			up_counter_value = 0;
		}
		temp_li = up_counter_value;
		display_value[0][0] = temp_li / 1000;
		temp_li %= 1000;
		display_value[0][1] = temp_li / 100;
		temp_li %= 100;
		display_value[0][3] = temp_li / 10;
		display_value[0][4] = temp_li % 10;
		display_value[0][2] = 0;
		write_flag = TRUE;
		counter_update_flag = FALSE;
		update_lcd_flag = TRUE;
	}
}

/****************************************************************************/
/**	update_down_count()
* This function updates the counter value for down counter by decrementing by one
* every COUNTER_UPDATE_DELAY time period and then updating the data to be written
*
* This function will be decrementing the value from the user given value.
* If the minimum value is reached, it will restart from the user given value.
*
 *****************************************************************************/

void update_down_count(void)
{
	int temp_li;
	if(counter_update_flag)
	{
		down_counter_value--;
		if(down_counter_value < 0)
		{
			down_counter_value = init_down_counter_value;
		}
		temp_li = down_counter_value;
		display_value[1][0] = temp_li / 1000;
		temp_li %= 1000;
		display_value[1][1] = temp_li / 100;
		temp_li %= 100;
		display_value[1][3] = temp_li / 10;
		display_value[1][4] = temp_li % 10;
		display_value[1][2] = 0;
		write_flag = TRUE;
		counter_update_flag = FALSE;
		update_lcd_flag = TRUE;
	}
}

/****************************************************************************/
/**	course_write()
* This function is made to write initial greeting on the board using marker
* i.e., "ECE"
* 		"544"
*
 *****************************************************************************/

void course_write(void)
{
	int i;
	erase();
	lift_hand();
	delay_msecs(DELAY_VALUE);
	PositionSet(motor1_position[0][letter[0][0]], motor2_position[0][letter[0][0]]);
	delay_msecs(DELAY_VALUE);
	rest_hand();
	delay_msecs(DELAY_VALUE);
	for(i = 1; i < 7; i++)
	{
		PositionSet(motor1_position[0][letter[0][i]], motor2_position[0][letter[0][i]]);
		delay_msecs(DELAY_VALUE);
	}
	lift_hand();
	delay_msecs(DELAY_VALUE);
	PositionSet(motor1_position[1][letter[1][0]], motor2_position[1][letter[1][0]]);
	delay_msecs(DELAY_VALUE);
	rest_hand();
	delay_msecs(DELAY_VALUE);
	for(i = 1; i < 5; i++)
	{
		PositionSet(motor1_position[1][letter[1][i]], motor2_position[1][letter[1][i]]);
		delay_msecs(DELAY_VALUE);
	}
	lift_hand();
	delay_msecs(DELAY_VALUE);
	PositionSet(motor1_position[3][letter[0][0]], motor2_position[3][letter[0][0]]);
	delay_msecs(DELAY_VALUE);
	rest_hand();
	delay_msecs(DELAY_VALUE);
	for(i = 1; i < 7; i++)
	{
		PositionSet(motor1_position[3][letter[0][i]], motor2_position[3][letter[0][i]]);
		delay_msecs(DELAY_VALUE);
	}
	lift_hand();
	delay_msecs(DELAY_VALUE);
	delay_msecs(3000);
	erase();

	PositionSet(motor1_position[0][digit[5][0]], motor2_position[0][digit[5][0]]);
	delay_msecs(DELAY_VALUE);
	rest_hand();
	delay_msecs(DELAY_VALUE);
	for(i = 1; i < 6; i++)
	{
		PositionSet(motor1_position[0][digit[5][i]], motor2_position[0][digit[5][i]]);
		delay_msecs(DELAY_VALUE);
	}
	lift_hand();
	delay_msecs(DELAY_VALUE);
	PositionSet(motor1_position[1][digit[4][0]], motor2_position[1][digit[4][0]]);
	delay_msecs(DELAY_VALUE);
	rest_hand();
	delay_msecs(DELAY_VALUE);
	for(i = 1; i < 6; i++)
	{
		PositionSet(motor1_position[1][digit[4][i]], motor2_position[1][digit[4][i]]);
		delay_msecs(DELAY_VALUE);
	}
	lift_hand();
	delay_msecs(DELAY_VALUE);
	PositionSet(motor1_position[3][digit[4][0]], motor2_position[3][digit[4][0]]);
	delay_msecs(DELAY_VALUE);
	rest_hand();
	delay_msecs(DELAY_VALUE);
	for(i = 1; i < 6; i++)
	{
		PositionSet(motor1_position[3][digit[4][i]], motor2_position[3][digit[4][i]]);
		delay_msecs(DELAY_VALUE);
	}
	lift_hand();
	delay_msecs(DELAY_VALUE);
	delay_msecs(3000);
}
