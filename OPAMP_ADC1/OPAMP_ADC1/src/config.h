/**
 * \file
 *
 * \brief SAM L21 Xplained Pro board configuration.
 *
 * Copyright (c) 2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */



#ifndef CONF_EXAMPLE_H_INCLUDED
#define CONF_EXAMPLE_H_INCLUDED


#define STX_STRING '#'
#define ETX_STRING '@'

#define CMD_WRITE 'W'
#define CMD_READ  'R'

typedef enum sb_cmd_index {ping,read_sensor,set_ncamp,set_threshold,set_analog_gain};
char sb_cmd_array[] = {'A','B','C','D','E','F','G'};
	
enum opamp_pot_mux_sel
{
	/** Gain = R2/R1 = 1/7 */
	 MUX_14R_2R = 1,
	/** Gain = R2/R1 = 1/3 */
	 MUX_12R_4R = 2,
	/** Gain = R2/R1 = 1 */
	 MUX_8R_8R = 3,
	/** Gain = R2/R1 = 1 + 2/3 */
	 MUX_6R_10R = 4,
	/** Gain = R2/R1 = 3 */
	 MUX_4R_12R = 5,
	/** Gain = R2/R1 = 4 + 1/3 */
	 MUX_3R_13R = 6,
	/** Gain = R2/R1 = 7 */
	 MUX_2R_14R = 7,
	/** Gain = R2/R1 = 15 */
	 MUX_R_15R = 8
};

#define PL_LENGTH 4
typedef struct packet
{
	char STX;
	char cmd;
	char sub_cmd;
	uint8_t payload[PL_LENGTH];
	char ETX;
}packet;

#define UART_TASK_PRIORITY      (tskIDLE_PRIORITY + 1)
#define UART_TASK_DELAY         (10 / portTICK_RATE_MS)

#define MAIN_TASK_PRIORITY      (tskIDLE_PRIORITY + 2)
#define MAIN_TASK_DELAY         (50 / portTICK_RATE_MS)

#define GRAPH_TASK_PRIORITY     (tskIDLE_PRIORITY + 1)
#define GRAPH_TASK_DELAY        (50 / portTICK_RATE_MS)

#define TERMINAL_TASK_PRIORITY  (tskIDLE_PRIORITY + 1)
#define TERMINAL_TASK_DELAY     (1000 / portTICK_RATE_MS)

#define ABOUT_TASK_PRIORITY     (tskIDLE_PRIORITY + 1)
#define ABOUT_TASK_DELAY        (33 / portTICK_RATE_MS)


#define TERMINAL_LENGTH 20

#define OPAMP_INPUT_MUX MUX_PA05B_OPAMP_OAPOS2
#define OPAMP_INPUT_PIN PIN_PA05B_OPAMP_OAPOS2


#define FRAC16(x) 32768*x
#define TAPNUMBER 20
int16_t lpFilterCoefficent[TAPNUMBER]=
{
	
 FRAC16(0.004443586424270492),
 FRAC16(0.007950412140913214),
 FRAC16(0.012610245639986008),
 FRAC16(0.018307516014938147),
 FRAC16(0.024750501636394886),
 FRAC16(0.031484247496533675),
 FRAC16(0.037935962464436299),
 FRAC16(0.043487743723806496),
 FRAC16(0.047564952316827565),
 FRAC16(0.049724232221913224),
 FRAC16(0.049724232221913224),
 FRAC16(0.047564952316827565),
 FRAC16(0.043487743723806496),
 FRAC16(0.037935962464436299),
 FRAC16(0.031484247496533675),
 FRAC16(0.024750501636394886),
 FRAC16(0.018307516014938147),
 FRAC16(0.012610245639986008),
 FRAC16(0.007950412140913214),
 FRAC16(0.004443586424270492)// FRAC16(0.002712951403180911),
// FRAC16(-0.001953514003023600),
// FRAC16(-0.012384256396527602),
// FRAC16(-0.019223510889666357),
// FRAC16(-0.006896289977616332),
// FRAC16( 0.030562878889653276),
// FRAC16( 0.072603163213511290),
// FRAC16( 0.070411352850095657),
// FRAC16(-0.045929806740464811),
// FRAC16(-0.571270370627533053),
// FRAC16( 0.571270370627533053),
// FRAC16( 0.045929806740464811),
// FRAC16(-0.070411352850095657),
// FRAC16(-0.072603163213511290),
// FRAC16(-0.030562878889653276),
// FRAC16( 0.006896289977616332),
// FRAC16( 0.019223510889666357),
// FRAC16( 0.012384256396527602),
// FRAC16( 0.001953514003023600),
// FRAC16(-0.002712951403180911)

	
	 
};



#endif /* CONF_EXAMPLE_H_INCLUDED */
