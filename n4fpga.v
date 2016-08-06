`timescale 1ns / 1ps
// n4fpga.v - Top level module for the ECE 544 Final project- Counter-Plotter
//
//
// Created By:	Abhishek Yadav/Saketh Kumsi
// Date:		6-March-2015
// Version:		2.0
//
// Description:
// ------------
// This module provides the top level for the Final Project hardware.
// The module assumes that a PmodCLP is plugged into the JA and JB
// expansion ports and that a PmodENC is plugged into the JD expansion 
// port (bottom row) and JC[4,5,6] are connected to servo motor signal
// inputs
//////////////////////////////////////////////////////////////////////
module n4fpga(
    input				clk,			// 100Mhz clock input
    input				btnC,			// center pushbutton
    input				btnU,			// UP (North) pusbhbutton
    input				btnL,			// LEFT (West) pushbutton
    input				btnD,			// DOWN (South) pushbutton  - used for system reset
    input				btnR,			// RIGHT (East) pushbutton
	input				btnCpuReset,	// CPU reset pushbutton
    input	[15:0]		sw,				// slide switches on Nexys 4
    output	[15:0] 		led,			// LEDs on Nexys 4   
    output [7:0]        an,             // Seven Segment display
    output [6:0]        seg,
    output              dp,
    
    input				uart_rtl_rxd,	// USB UART Rx and Tx on Nexys 4
    output				uart_rtl_txd,	
    
    output	[7:0] 		JA,				// JA Pmod connector - PmodCLP data bus
										// both rows are used
    output	[7:0] 		JB,				// JB Pmod connector - PmodCLP control signals
										// only the bottom row is used
    output	[7:0] 		JC,				// JC Pmod connector - PWM signals on 4,5,6
										// only the bottom row is used
	input	[7:0]		JD				// JD Pmod connector - PmodENC signals
);

// internal variables
wire				sysclk;
wire				sysreset_n, sysreset;
wire				rotary_a, rotary_b, rotary_press, rotary_sw;
wire	[7:0]		lcd_d;
wire				lcd_rs, lcd_rw, lcd_e;

wire	[7:0]	gpio_in;				// embsys GPIO input port  (axi_gpio_0)
wire	[7:0]	gpio_out;				// embsys GPIO output port (axi_gpio_0)

wire            pwm_out1;                // 3 PWM outputs from the axi_timer
wire            pwm_out2;
wire            pwm_out3;

wire            clk_20khz;              // used for LCD


wire    [15:0]  led_int;                // Nexys4IO drives these outputs


// make the connections

// system-wide signals
assign sysclk = clk;
assign sysreset_n = btnCpuReset;		// The CPU reset pushbutton is asserted low.  The other pushbuttons are asserted high
										// but the microblaze for Nexys 4 expects reset to be asserted low
assign sysreset = ~sysreset_n;			// Generate a reset signal that is asserted high for any logic blocks expecting it.

// PmodCLP signals
// JA - top and bottom rows
// JB - bottom row only
assign JA = lcd_d[7:0];
assign JB = {1'b0, lcd_e, lcd_rw, lcd_rs, 2'b000, clk_20khz, pwm_out1};

// PmodENC signals
// JD - bottom row only
// Pins are assigned such that turning the knob to the right
// causes the rotary count to increment.
assign rotary_a = JD[5];
assign rotary_b = JD[4];
assign rotary_press = JD[6];
assign rotary_sw = JD[7];

// PWM signals for the Servo motors
assign JC[4] = pwm_out1;
assign JC[5] = pwm_out2;
assign JC[6] = pwm_out3; 
			
// instantiate the embedded system
gsdesign_1 EMBSYS
       (.PmodCLP_DataBus(lcd_d),
        .PmodCLP_E(lcd_e),
        .PmodCLP_RS(lcd_rs),
        .PmodCLP_RW(lcd_rw),
        .PmodENC_A(rotary_a),
        .PmodENC_B(rotary_b),
        .PmodENC_BTN(rotary_press),
        .PmodENC_SWT(rotary_sw),
        .an(an),
        .btnC(btnC),
        .btnD(btnD),
        .btnL(btnL),
        .btnR(btnR),
        .btnU(btnU),
        .dp(dp),
        .led(led_int),
        .seg(seg),
        .sw(sw),
        .sysreset_n(sysreset_n),
        .sysclk(sysclk),
        .uart_rtl_rxd(uart_rtl_rxd),
        .uart_rtl_txd(uart_rtl_txd),
        .gpio_0_GPIO2_tri_o(gpio_out),
        .gpio_0_GPIO_tri_i(gpio_in),
        .pwm1(pwm_out1),
        .pwm2(pwm_out2),
        .pwm3(pwm_out3));

endmodule
