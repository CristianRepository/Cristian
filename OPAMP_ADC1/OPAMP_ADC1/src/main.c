/**
 * \file
 *
 * \brief SAM L21 OPAMP as ADC Gain Amplifier
 *
 * Copyright (C) 2016 Atmel Corporation. All rights reserved.
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


/**
 * \mainpage OPAMP as ADC Gain Amplifier Example application
 *
 * \section Purpose
 *
 * The example configures the OPAMP to act as a gain amplifier for the ADC.
 *
 * \section Requirements
 *
 * This package can be used with the SAM L21 xplained pro.
 *
 * This package has been tested on following boards:
 * - SAM L21 Xplained Pro
 *
 * \section Description
 *
 * The example setups the OPAMP to act as a gain amplifier for the ADC.
 * the OPAMP is configured as a non-inverting PGA, with a gain of 4.
 *
 * \section Usage
 *
 * -# Connect a voltage to be measured to pin PA05 (Opamp 2 positive input)
 * -# Build the program and download it to the evaluation board.
 * -# Start a debug session and run the program to completion.
 * -# The measured values can be read in adc_result_buffer.
 *
 */

#include <asf.h>
#include <FreeRTOS.h>
#include "config.h"
#include "string.h"
#include "arm_math.h"
#include <status_codes.h>

/* Buffer for ADC sample storage */
#define ADC_SAMPLES 256
uint16_t adc_result_buffer[ADC_SAMPLES];
uint16_t adc_result_buffer_filtered[ADC_SAMPLES];


packet pkt;

void configure_opamp2(enum opamp_pot_mux gain);
void configure_adc(void);
void configure_adc_callbacks(void);
void adc_complete_callback(struct adc_module *const module);

//! Interrupt handler for reception from EDBG Virtual COM Port
static void rx_handler(uint8_t instance);

int n_sample = 10;//ADC_SAMPLES;
uint16_t media(uint16_t *buffer,uint32_t n_campioni);

const arm_fir_instance_q15 sFirStru;
int16_t FirState[TAPNUMBER+ADC_SAMPLES];

	
arm_cfft_radix4_instance_q15 S;
arm_rfft_instance_q15 SQ;
arm_status status;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;


//! Queue for incoming terminal characters
static xQueueHandle terminal_in_queue;


static void main_task(void *params);
static void uart_task(void *params);

uint16_t interrupt_status;
uint16_t data;
uint8_t error_code;
int count = 0;




/* ADC interrupt flag */
volatile bool adc_read_done = false;

/* ADC module */
struct adc_module adc_instance;
struct usart_module usart_instance;

/* Configure OPAMP2 and I/O PORT */
void configure_opamp2(enum opamp_pot_mux gain)
{
	/* Creates a new configuration structure for the OPAMP2. */
	struct opamp2_config conf;

	/* Initializes OPAMP module. */
	opamp_module_init();

	/* Fill with the default settings. */
	opamp2_get_config_defaults(&conf);

	/* Set the the OPAMP2 in "Non-Inverted PGA" mode, */
	conf.negative_input           = OPAMP2_NEG_MUX_TAP2;
	conf.positive_input           = OPAMP2_POS_MUX_PIN2;
	conf.r1_connection            = OPAMP2_RES1_MUX_GND;
	conf.config_common.potentiometer_selection = gain;
	conf.config_common.r1_enable  = true;
	conf.config_common.r2_out     = true;
	conf.config_common.analog_out = true;

	/* Set up OA2POS pin as input. */
	struct system_pinmux_config opamp2_input_pin_conf;
	system_pinmux_get_config_defaults(&opamp2_input_pin_conf);
	opamp2_input_pin_conf.direction    = SYSTEM_PINMUX_PIN_DIR_INPUT;
	opamp2_input_pin_conf.mux_position = OPAMP_INPUT_MUX;
	system_pinmux_pin_set_config(OPAMP_INPUT_PIN, &opamp2_input_pin_conf);

	/* Initialize and enable the OPAMP2 with the user settings. */
	opamp2_set_config(&conf);
	opamp_enable(OPAMP_2);
	
	/* Wait for the output ready. */
	while(!opamp_is_ready(OPAMP_2));
}

/* Configure ADC */
void configure_adc(void)
{
	/* Creates a new configuration structure for the ADC */
	struct adc_config config_adc;

	adc_get_config_defaults(&config_adc);
	
	/* Setup ADC with OPAMP2 output as ADC input */
	config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV256;//ADC_CLOCK_PRESCALER_DIV8;
	config_adc.positive_input  = ADC_POSITIVE_INPUT_OPAMP2;
	
	/* Initialize and enable ADC */
	adc_init(&adc_instance, ADC, &config_adc);
	adc_enable(&adc_instance);
}

/* Enable ADC Callback Function */
void configure_adc_callbacks(void)
{
	adc_register_callback(&adc_instance,
			adc_complete_callback, ADC_CALLBACK_READ_BUFFER);
	adc_enable_callback(&adc_instance, ADC_CALLBACK_READ_BUFFER);
}

/* ADC Callback Function */
void adc_complete_callback(struct adc_module *const module)
{
	/* Set ADC conversion ended flag */
	adc_read_done = true;
}



static void configure_usart(void)
{
	uint8_t instance_index;

	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	config_usart.baudrate    = 115200;
	config_usart.mux_setting = EXT2_UART_SERCOM_MUX_SETTING;
	config_usart.pinmux_pad0 = EXT2_UART_SERCOM_PINMUX_PAD0;
	config_usart.pinmux_pad1 = EXT2_UART_SERCOM_PINMUX_PAD1;
	config_usart.pinmux_pad2 = EXT2_UART_SERCOM_PINMUX_PAD2;
	config_usart.pinmux_pad3 = EXT2_UART_SERCOM_PINMUX_PAD3;
	stdio_serial_init(&usart_instance, EXT2_UART_MODULE, &config_usart);
	
	// Inject our own interrupt handler
	instance_index = _sercom_get_sercom_inst_index(EXT2_UART_MODULE);
	_sercom_set_handler(instance_index, rx_handler);

	// Enable the UART transceiver
	usart_enable(&usart_instance);
	usart_enable_transceiver(&usart_instance, USART_TRANSCEIVER_TX);
	usart_enable_transceiver(&usart_instance, USART_TRANSCEIVER_RX);

	// ..and the RX Complete interrupt
	((SercomUsart *)EXT2_UART_MODULE)->INTENSET.reg = SERCOM_USART_INTFLAG_RXC;
		
}

struct port_config pin_conf;
	
static void configure_led_port(void)
{
	
	port_get_config_defaults(&pin_conf);
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;

	port_pin_set_config(EXT3_PIN_4, &pin_conf);
	port_pin_set_config(EXT3_PIN_6, &pin_conf);
	port_pin_set_config(EXT3_PIN_8, &pin_conf);
	port_pin_set_config(EXT3_PIN_10, &pin_conf);
	port_pin_set_config(EXT3_PIN_9, &pin_conf);
	port_pin_set_config(EXT3_PIN_15, &pin_conf);
	port_pin_set_config(EXT3_PIN_16, &pin_conf);
	port_pin_set_config(EXT3_PIN_18, &pin_conf);

	port_pin_set_output_level(EXT3_PIN_4, true);
	port_pin_set_output_level(EXT3_PIN_6, true);
	port_pin_set_output_level(EXT3_PIN_8, true);
	port_pin_set_output_level(EXT3_PIN_10, true);
	port_pin_set_output_level(EXT3_PIN_9, true);
	port_pin_set_output_level(EXT3_PIN_15, true);
	port_pin_set_output_level(EXT3_PIN_16, true);
	port_pin_set_output_level(EXT3_PIN_18, true);

	/*********************************/

}

#define QUANTUM_VALUE 4096/16//4096/8
//static void evaluate_led_output(uint16_t value)
//{
	//
//port_pin_set_output_level(EXT3_PIN_4, false);
//port_pin_set_output_level(EXT3_PIN_6, false);
//port_pin_set_output_level(EXT3_PIN_8, false);
//port_pin_set_output_level(EXT3_PIN_10, false);
//port_pin_set_output_level(EXT3_PIN_9, false);
//port_pin_set_output_level(EXT3_PIN_15, false);
//port_pin_set_output_level(EXT3_PIN_16, false);
//port_pin_set_output_level(EXT3_PIN_18, false);
	//
//if(value >= 1<<0) port_pin_set_output_level(EXT3_PIN_4, true);
//else port_pin_set_output_level(EXT3_PIN_4, false);
//
//if(value >= 1<<2) port_pin_set_output_level(EXT3_PIN_6, true);
//else port_pin_set_output_level(EXT3_PIN_6, false);
//
//if(value >= 1<<4) port_pin_set_output_level(EXT3_PIN_8, true);
//else port_pin_set_output_level(EXT3_PIN_8, false);
//
//if(value >= 1<<6) port_pin_set_output_level(EXT3_PIN_10, true);
//else port_pin_set_output_level(EXT3_PIN_10, false);
//
//if(value >= 1<<8) port_pin_set_output_level(EXT3_PIN_9, true);
//else port_pin_set_output_level(EXT3_PIN_9, false);
//
//if(value >= 1<<10) port_pin_set_output_level(EXT3_PIN_15, true);
//else port_pin_set_output_level(EXT3_PIN_15, false);
//
//if(value >= 1<<12) port_pin_set_output_level(EXT3_PIN_16, true);
//else port_pin_set_output_level(EXT3_PIN_16, false);
//
//if(value >= 1<<14) port_pin_set_output_level(EXT3_PIN_18, true);
//else port_pin_set_output_level(EXT3_PIN_18, false);
//
//
//}

static void evaluate_led_output(uint16_t value)
{
	
	port_pin_set_output_level(EXT3_PIN_4, false);
	port_pin_set_output_level(EXT3_PIN_6, false);
	port_pin_set_output_level(EXT3_PIN_8, false);
	port_pin_set_output_level(EXT3_PIN_10, false);
	port_pin_set_output_level(EXT3_PIN_9, false);
	port_pin_set_output_level(EXT3_PIN_15, false);
	port_pin_set_output_level(EXT3_PIN_16, false);
	port_pin_set_output_level(EXT3_PIN_18, false);
	
	if(value >= QUANTUM_VALUE) port_pin_set_output_level(EXT3_PIN_4, true);
	else port_pin_set_output_level(EXT3_PIN_4, false);

	if(value >= QUANTUM_VALUE * 2) port_pin_set_output_level(EXT3_PIN_6, true);
	else port_pin_set_output_level(EXT3_PIN_6, false);

	if(value >= QUANTUM_VALUE * 3) port_pin_set_output_level(EXT3_PIN_8, true);
	else port_pin_set_output_level(EXT3_PIN_8, false);

	if(value >= QUANTUM_VALUE * 4) port_pin_set_output_level(EXT3_PIN_10, true);
	else port_pin_set_output_level(EXT3_PIN_10, false);

	if(value >= QUANTUM_VALUE * 5) port_pin_set_output_level(EXT3_PIN_9, true);
	else port_pin_set_output_level(EXT3_PIN_9, false);

	if(value >= QUANTUM_VALUE * 6) port_pin_set_output_level(EXT3_PIN_15, true);
	else port_pin_set_output_level(EXT3_PIN_15, false);

	if(value >= QUANTUM_VALUE * 7) port_pin_set_output_level(EXT3_PIN_16, true);
	else port_pin_set_output_level(EXT3_PIN_16, false);

	if(value >= QUANTUM_VALUE * 8) port_pin_set_output_level(EXT3_PIN_18, true);
	else port_pin_set_output_level(EXT3_PIN_18, false);


}

void configure_dac(void);
void configure_dac_channel(void);

//! [module_inst]
struct dac_module dac_instance;
//! [module_inst]

//! [setup]
void configure_dac(void)
{
	//! [setup_config]
	struct dac_config config_dac;
	//! [setup_config]
	//! [setup_config_defaults]
	dac_get_config_defaults(&config_dac);
	//! [setup_config_defaults]

	//! [setup_set_config]
	dac_init(&dac_instance, DAC, &config_dac);
	//! [setup_set_config]
}

void configure_dac_channel(void)
{
	//! [setup_ch_config]
	struct dac_chan_config config_dac_chan;
	//! [setup_ch_config]
	//! [setup_ch_config_defaults]
	dac_chan_get_config_defaults(&config_dac_chan);
	//! [setup_ch_config_defaults]

	//! [setup_ch_set_config]
	dac_chan_set_config(&dac_instance, DAC_CHANNEL_0, &config_dac_chan);
	//! [setup_ch_set_config]

	//! [setup_ch_enable]
	dac_chan_enable(&dac_instance, DAC_CHANNEL_0);
	//! [setup_ch_enable]
}
//! [setup]

//! [packet_data]
#define DATA_LENGTH 3
static uint8_t write_buffer[DATA_LENGTH];
static uint8_t read_buffer[DATA_LENGTH];

#define VREG 0x11
#define NVREG 0x21
#define NVREGxVREG 0x61
#define VREGxNVREG 0x51

#define SLAVE_ADDRESS 0x00
#define TIMEOUT 1000
struct i2c_master_module i2c_master_instance;
void configure_i2c_master(void);
void configure_i2c_master(void)
{

	struct i2c_master_config config_i2c_master;
	i2c_master_get_config_defaults(&config_i2c_master);
	//config_i2c_master.baud_rate = I2C_MASTER_BAUD_RATE_400KHZ;
	config_i2c_master.buffer_timeout = 10000;
	#if SAMR30
	config_i2c_master.pinmux_pad0    = CONF_MASTER_SDA_PINMUX;
	config_i2c_master.pinmux_pad1    = CONF_MASTER_SCK_PINMUX;
	#endif
	i2c_master_init(&i2c_master_instance, SERCOM2, &config_i2c_master);
	i2c_master_enable(&i2c_master_instance);
}



uint16_t result = 0;
uint16_t integrale = 0;
uint8_t alarm_detected = 0;

uint16_t value_rx = 0;
uint16_t threshold_value = 0;
uint8_t analog_gain = 1;
bool change_gain = false;
uint32_t i = 0;
char cmd[10];
bool flag = false;
	
// /* Queue structure */
#define QUEUE_ELEMENTS 10
#define QUEUE_SIZE (QUEUE_ELEMENTS + 1)
uint16_t Queue[QUEUE_SIZE];
uint16_t QueueIn, QueueOut;
// 
 void QueueInit(void);
 int QueuePut(uint16_t new);
 int QueueGet(uint16_t *old);
// 
 void QueueInit(void)
 {
 	QueueIn = QueueOut = 0;
 }
// 
 int QueuePut(uint16_t new)
 {
 	if(QueueIn == (( QueueOut - 1 + QUEUE_SIZE) % QUEUE_SIZE))
 	{
 		return -1; /* Queue Full*/
 	}
 
 	Queue[QueueIn] = new;
 
 	QueueIn = (QueueIn + 1) % QUEUE_SIZE;
 
 	return 0; // No errors
 }
 
 int QueueGet(uint16_t *old)
 {
 	if(QueueIn == QueueOut)
 	{
 		return -1; /* Queue Empty - nothing to get*/
 	}
 
 	*old = Queue[QueueOut];
 
 	QueueOut = (QueueOut + 1) % QUEUE_SIZE;
 
 	return 0; // No errors
 }
	
	

	
	
	
enum status_code err;	
uint16_t timeout = 0;
/* Main function */
int main(void)
{
	
	system_init();
	
	/************************ OPAMP2 e ADC *****************************/
	/* Initialize OPAMP2 and ADC */
	configure_opamp2(OPAMP_POT_MUX_4R_12R);//OPAMP_POT_MUX_14R_2R);
	configure_adc();
	configure_adc_callbacks();
	
	
	/************************ DAC *****************************/
	configure_dac();
	configure_dac_channel();
	dac_enable(&dac_instance);
	dac_chan_write(&dac_instance, DAC_CHANNEL_0, 0xbf);
	
	/************************ HW Gain *****************************/
	configure_i2c_master();
	uint8_t HW_Gain= 1;
	write_buffer[0] = VREG;
	write_buffer[1] = HW_Gain << 3;
	
	struct i2c_master_packet packet = {
		.address     = SLAVE_ADDRESS,
		.data_length = DATA_LENGTH,
		.data        = write_buffer,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};
	
	//do{
		//err = i2c_master_write_packet_wait(&i2c_master_instance, &packet);
		///* Increment timeout counter and check if timed out. */
		//packet.address += 1; 
	//}while (err != STATUS_OK);
	/************************ UART *****************************/	
	configure_usart();
	delay_init();
	port_pin_toggle_output_level(LED_0_PIN);
	/************************ LED PORT *****************************/	
	configure_led_port();
	//UP//
	evaluate_led_output(QUANTUM_VALUE * 1);
	delay_ms(100);
	evaluate_led_output(QUANTUM_VALUE * 2);
	delay_ms(100);
	evaluate_led_output(QUANTUM_VALUE * 3);
	delay_ms(100);
	evaluate_led_output(QUANTUM_VALUE * 4);
	delay_ms(100);
	evaluate_led_output(QUANTUM_VALUE * 5);
	delay_ms(100);
	evaluate_led_output(QUANTUM_VALUE * 6);
	delay_ms(100);
	evaluate_led_output(QUANTUM_VALUE * 7);
	delay_ms(100);
	evaluate_led_output(QUANTUM_VALUE * 8);
	
	delay_ms(100);
	port_pin_toggle_output_level(LED_0_PIN);
	
	
	/************************ FIR FILTER *****************************/
	status = arm_fir_init_q15(&sFirStru,TAPNUMBER,&lpFilterCoefficent[0],&FirState[0],ADC_SAMPLES);
	//DOWN//
	evaluate_led_output(QUANTUM_VALUE * 8);
	delay_ms(100);
	evaluate_led_output(QUANTUM_VALUE * 7);
	delay_ms(100);
	evaluate_led_output(QUANTUM_VALUE * 6);
	delay_ms(100);
	evaluate_led_output(QUANTUM_VALUE * 5);
	delay_ms(100);
	evaluate_led_output(QUANTUM_VALUE * 4);
	delay_ms(100);
	evaluate_led_output(QUANTUM_VALUE * 3);
	delay_ms(100);
	evaluate_led_output(QUANTUM_VALUE * 2);
	delay_ms(100);
	evaluate_led_output(QUANTUM_VALUE * 1);
	
	delay_ms(100);
	port_pin_toggle_output_level(LED_0_PIN);
	delay_ms(100);
	port_pin_toggle_output_level(LED_0_PIN);
	delay_ms(100);
	port_pin_toggle_output_level(LED_0_PIN);
	delay_ms(100);
	port_pin_toggle_output_level(LED_0_PIN);


	

	terminal_in_queue = xQueueCreate(64, sizeof(uint8_t));

	xTaskCreate(main_task,
	(const char *) "Main",
	configMINIMAL_STACK_SIZE,
	NULL,
	MAIN_TASK_PRIORITY,
	NULL);


	xTaskCreate(uart_task,
	(const char *) "UART",
	configMINIMAL_STACK_SIZE,
	NULL,
	UART_TASK_PRIORITY,
	NULL);


	/* Enable global interrupts */
	system_interrupt_enable_global();
	
	// ..and let FreeRTOS run tasks!
	vTaskStartScheduler();

	do {
		// Intentionally left empty
	} while (true);
	
}

int sum(uint16_t* array, int size_array)
{
	int somma = 0;
	for(int i = 0; i<size_array; i++)
	{
		somma += array[i];
	}
	return somma;
}

void calculate_conv (uint16_t array1[],unsigned npts1, uint16_t array2[],unsigned npts2,uint32_t result[]) {

//unsigned npts1 = 101;
//unsigned npts2 = 101;
unsigned npts = npts1 + npts2;
//uint16_t conv_spectra[npts];


// Convolution
	for (unsigned i = 0; i < npts; i++) {
		uint32_t sum = 0;
		for (unsigned j = 0; j <= i; j++) 
		{
			if (j > npts1 - 1 || (i - j) > npts2 - 1) 
			continue;
			sum += array1[j] * array2[i-j]; 
		} 
		
		result[i] = sum;
	} 

return;

}


uint32_t conv_result[ADC_SAMPLES*2];
uint16_t mask[ADC_SAMPLES];

int compare (const void * a, const void * b)
{
  return ( *(int*)a - *(int*)b );
}

int max_array(uint32_t* array, uint32_t size_array)
{
	uint32_t maximum, c, location = 1;

	maximum = array[0];
	
	for (c = 1; c < size_array; c++)
	{
		if (array[c] > maximum)
		{
			maximum  = array[c];
			location = c;
		}
	}
	
	return location;
}


uint32_t Max_index;
	
static void main_task(void *params)
{
	uint16_t old_result;
	uint32_t Max_result;


	//QueueInit();
	//QueuePut(0xAA);

	for(;;) {
		/* Start ADC conversion */
		adc_read_buffer_job(&adc_instance, adc_result_buffer, n_sample);
		
		if (adc_read_done == true) {
			
			adc_read_done = false;
		
		
			arm_fir_fast_q15(&sFirStru,adc_result_buffer,adc_result_buffer_filtered,n_sample);
			result = media(adc_result_buffer_filtered,n_sample);
			
			//for(int i=0;i<n_sample;i++)
			//{
				//mask[i] = (uint8_t)rand();
			//}
			
			//calculate_conv(adc_result_buffer_filtered,n_sample, adc_result_buffer_filtered,n_sample,conv_result);
			//Max_index = max_array(conv_result,ADC_SAMPLES*2);
			
			//calculate_conv(adc_result_buffer_filtered,n_sample, mask,n_sample,conv_result);
			//Max_index = max_array(conv_result,sizeof(conv_result));
			

	
			//result = media(adc_result_buffer,n_sample);
			
			evaluate_led_output(result);
			
			if(integrale > threshold_value)
			{
				alarm_detected = 1;
			}
			else
			{
				alarm_detected = 0;
			}
		}
			

		
		if(change_gain == true)
		{
			enum opamp_pot_mux temp_gain = OPAMP_POT_MUX_14R_2R;
			switch(analog_gain){
				case MUX_14R_2R:temp_gain = OPAMP_POT_MUX_14R_2R;
				break;
				/** Gain = R2/R1 = 1/3 */
				case MUX_12R_4R:temp_gain = OPAMP_POT_MUX_12R_4R;
				break;
				/** Gain = R2/R1 = 1 */
				case MUX_8R_8R:temp_gain = OPAMP_POT_MUX_8R_8R;
				break;
				/** Gain = R2/R1 = 1 + 2/3 */
				case MUX_6R_10R:temp_gain = OPAMP_POT_MUX_6R_10R;
				break;
				/** Gain = R2/R1 = 3 */
				case MUX_4R_12R:temp_gain = OPAMP_POT_MUX_4R_12R;
				break;
				/** Gain = R2/R1 = 4 + 1/3 */
				case MUX_3R_13R:temp_gain = OPAMP_POT_MUX_3R_13R;
				break;
				/** Gain = R2/R1 = 7 */
				case MUX_2R_14R:temp_gain = OPAMP_POT_MUX_2R_14R;
				break;
				/** Gain = R2/R1 = 15 */
				case MUX_R_15R:temp_gain = OPAMP_POT_MUX_R_15R;
				break;
				default:
				break;
			}
			
			configure_opamp2(temp_gain);
			
			change_gain = false;
		}
		
		vTaskDelay(MAIN_TASK_DELAY);
	}
}


uint16_t media(uint16_t *buffer,uint32_t n_campioni)
{
	long tmp = 0;
	for(i=0;i<n_campioni;i++)
	{
		tmp += buffer[i];
	}
	return (uint16_t)(tmp/n_campioni);
}




	
static void uart_task(void *params)
{

	//memset(terminal_buffer,0,TERMINAL_LENGTH);
	char current_char;
	char *current_char_ptr = &current_char;
	char r;
	packet *pkt_temp_ptr;
	uint8_t buffer_temp[sizeof(packet)];
	uint32_t buffer_temp_index = 0;

	
	//uint32_t packet_length = sizeof(packet);
	//char* temporary_char = &s[0];
	
	 memset(buffer_temp,0,sizeof(packet));
	
	for (;;) {

		// Any characters queued? Handle them!
		while (xQueueReceive(terminal_in_queue, current_char_ptr, 0)) 
		{
			
			r = *current_char_ptr;
			
			switch(r)
			{
				case '@':
				
					buffer_temp[buffer_temp_index] = r;
					
					buffer_temp_index++;
					
					pkt_temp_ptr = (packet*)&buffer_temp[0];
					pkt = *pkt_temp_ptr;
					
					
					if ((pkt_temp_ptr->STX == STX_STRING) & (pkt_temp_ptr->ETX == ETX_STRING))
					{
						port_pin_toggle_output_level(LED_0_PIN);
						switch (pkt_temp_ptr->cmd)
						{
							case CMD_READ:
							if(pkt_temp_ptr->sub_cmd == sb_cmd_array[ping])
							{
									pkt.sub_cmd = pkt_temp_ptr->sub_cmd;
									pkt.payload[0]=0xAA;
									pkt.payload[1]=0xBB;
									usart_write_buffer_wait(&usart_instance,(uint8_t *)&pkt,sizeof(packet));
									delay_ms(25);
							}
							else if(pkt_temp_ptr->sub_cmd == sb_cmd_array[read_sensor])
							{
									pkt.sub_cmd = pkt_temp_ptr->sub_cmd;
									pkt.payload[0]=(uint8_t)result;
									pkt.payload[1]=(uint8_t)(result>>8);
									//pkt.payload[0]=(uint8_t)integrale;
									//pkt.payload[1]=(uint8_t)(integrale>>8);									
									pkt.payload[2]=0x00;
									pkt.payload[3]=(uint8_t)(alarm_detected);
									usart_write_buffer_wait(&usart_instance,(uint8_t *)&pkt,sizeof(packet));
									delay_ms(25);
							}
							break;
							case CMD_WRITE:
							if(pkt_temp_ptr->sub_cmd == sb_cmd_array[set_ncamp])
							{
								value_rx = (uint16_t)(pkt_temp_ptr->payload[1] << 8 | (pkt_temp_ptr->payload[0]));
								n_sample = value_rx;
								usart_write_buffer_wait(&usart_instance,(uint8_t *)&pkt,sizeof(packet));
								delay_ms(25);
							}
							else if(pkt_temp_ptr->sub_cmd == sb_cmd_array[set_threshold])
							{
								threshold_value = pkt_temp_ptr->payload[1] << 8 | (pkt_temp_ptr->payload[0]);
								usart_write_buffer_wait(&usart_instance,(uint8_t *)&pkt,sizeof(packet));
								delay_ms(25);
							}
							else if(pkt_temp_ptr->sub_cmd == sb_cmd_array[set_analog_gain])
							{
								uint8_t temp = pkt_temp_ptr->payload[0];
								if(analog_gain != temp)
								{
									analog_gain = temp;
									change_gain = true;
								}
								usart_write_buffer_wait(&usart_instance,(uint8_t *)&pkt,sizeof(packet));
								delay_ms(25);
							}
							break;
							
							default:
						
							break;
						}
					}
					
					memset(buffer_temp,0,sizeof(packet));
					buffer_temp_index = 0;
						
				break;
				default:
					if(buffer_temp_index < sizeof(buffer_temp))
					{
						buffer_temp[buffer_temp_index] = r;
						buffer_temp_index++;
					}
				break;
			}
			

			*current_char_ptr = '\0';
		}

		vTaskDelay(UART_TASK_DELAY);
	}
}



static void rx_handler(uint8_t instance)
{
	SercomUsart *const usart_hw = (SercomUsart *)EXT2_UART_MODULE;

	// Wait for synch to complete
	#if defined(FEATURE_SERCOM_SYNCBUSY_SCHEME_VERSION_1)
	while (usart_hw->STATUS.reg & SERCOM_USART_STATUS_SYNCBUSY) {
	}
	#elif defined(FEATURE_SERCOM_SYNCBUSY_SCHEME_VERSION_2)
	while (usart_hw->SYNCBUSY.reg) {
	}
	#endif

	// Read and mask interrupt flag register
	interrupt_status = usart_hw->INTFLAG.reg;

	if (interrupt_status & SERCOM_USART_INTFLAG_RXC) {
		// Check for errors
		error_code = (uint8_t)(usart_hw->STATUS.reg & SERCOM_USART_STATUS_MASK);
		if (error_code) {
			// Only frame error and buffer overflow should be possible
			if (error_code &
			(SERCOM_USART_STATUS_FERR | SERCOM_USART_STATUS_BUFOVF)){
				
				usart_hw->STATUS.reg =
				SERCOM_USART_STATUS_FERR | SERCOM_USART_STATUS_BUFOVF;
			}
			else {
				// Error: unknown failure
			}
			// All is fine, so push the received character into our queue
		}
		else {
			
			data = (usart_hw->DATA.reg & SERCOM_USART_DATA_MASK);
			//printf("%c\n\r",data);
			count++;
			if (!xQueueSendFromISR(terminal_in_queue, (uint8_t *)&data,NULL))
			{
				// Error: could not enqueue character
			}
			else
			{
				//// Echo back! Data reg. should empty fast since this is the
				//// only place anything is sent.
				//while (!(interrupt_status & SERCOM_USART_INTFLAG_DRE))
				//{
					//interrupt_status = usart_hw->INTFLAG.reg;
				//}
				//usart_hw->DATA.reg = (uint8_t)data;
			}
		}
		} else {
		// Error: only RX interrupt should be enabled
	}
}




