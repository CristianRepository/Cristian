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
#include "conf_example.h"
#include "string.h"
#include <status_codes.h>
#include "eeprom.h"
#include "crc8.h"
#include "arm_math.h"
#include "MAX5435.h"
#include "lis302dl.h"


struct configuration_param piezoboard_parameters;

/* Buffer for ADC sample storage */
#define ADC_SAMPLES 32
#define THRESHOLD_DEFAULT 100
#define PICK_COUNT_UP_DEFAULT 40
#define PICK_COUNT_DOWN_DEFAULT 20


uint16_t adc_result_buffer_ch1[ADC_SAMPLES];
uint16_t adc_result_buffer_filtered_ch1[ADC_SAMPLES/2];

uint16_t adc_result_buffer_ch2[ADC_SAMPLES];
uint16_t adc_result_buffer_filtered_ch2[ADC_SAMPLES/2];


packet pkt;
uint16_t interrupt_status;
uint16_t data;
uint8_t error_code;
int count = 0;
uint16_t result = 0;

uint16_t integrale_ch1 = 0;
uint16_t integrale_ch2 = 0;

uint32_t pick_count_up = 0;
uint32_t pick_count_down = PICK_COUNT_DOWN_DEFAULT;

uint8_t alarm_detected = 0;
uint16_t value_rx = 0;
uint16_t threshold_value = 500;
uint8_t analog_gain_1 = 1;
uint8_t analog_gain_2 = 1;
bool change_gain = false;
uint32_t i = 0;
char cmd[10];
bool flag = false;
/* ADC interrupt flag */
volatile bool adc_read_done = false;


/*************************************************************************/

const arm_fir_instance_q15 sFirStru;
int16_t FirState[TAPNUMBER+ADC_SAMPLES];
arm_cfft_radix4_instance_q15 S;
arm_rfft_instance_q15 SQ;
arm_status status;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;

/*************************************************************************/
					/*ACCELEROMETER LIS302DL*/

extern  uint8_t wr_buffer[DATA_LENGTH_WR];
extern  uint8_t rd_buffer[DATA_LENGTH_RD];

uint8_t acc_x = 0;
uint8_t acc_y = 0;
uint8_t acc_z = 0;


/************************************************************************/



void configure_opamp2(enum opamp_pot_mux gain);
void configure_adc(void);
void configure_adc_callbacks(void);
void adc_complete_callback_ch1(struct adc_module *const module);
void adc_complete_callback_ch2(struct adc_module *const module);
//! Interrupt handler for reception from EDBG Virtual COM Port
static void rx_handler(uint8_t instance);

uint16_t n_sample = ADC_SAMPLES;
uint16_t media(uint16_t *buffer,uint16_t n_init_camp,uint16_t length_buffer);
void save_configuration_param(void);
bool load_configuration_param(struct configuration_param *piezoboard_parameters_tmp);

//! Queue for incoming terminal characters
static xQueueHandle terminal_in_queue;

static void main_task(void *params);
static void uart_task(void *params);

static void accelerometer_task(void *params);

/* ADC module */
struct adc_module adc_instance_ch1;
struct adc_module adc_instance_ch2;

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

	/* Set the the OPAMP2 in "Non-Inverted PGA" mode, gain of 4 */
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
	opamp2_input_pin_conf.mux_position = OPAMP2_INPUT_MUX;
	system_pinmux_pin_set_config(OPAMP2_INPUT_PIN, &opamp2_input_pin_conf);

	/* Initialize and enable the OPAMP2 with the user settings. */
	opamp2_set_config(&conf);
	opamp_enable(OPAMP_2);
	
	/* Wait for the output ready. */
	while(!opamp_is_ready(OPAMP_2));
}

/* Configure OPAMP2 and I/O PORT */
void configure_opamp0(enum opamp_pot_mux gain)
{
	/* Creates a new configuration structure for the OPAMP2. */
	struct opamp0_config conf;

	/* Initializes OPAMP module. */
	opamp_module_init();

	/* Fill with the default settings. */
	opamp0_get_config_defaults(&conf);

	/* Set the the OPAMP2 in "Non-Inverted PGA" mode, gain of 4 */
	conf.negative_input           = OPAMP0_NEG_MUX_TAP0;
	conf.positive_input           = OPAMP0_POS_MUX_PIN0;
	conf.r1_connection            = OPAMP0_RES1_MUX_GND;
	conf.config_common.potentiometer_selection = gain;
	conf.config_common.r1_enable  = true;
	conf.config_common.r2_out     = true;
	conf.config_common.analog_out = true;

	/* Set up OA2POS pin as input. */
	struct system_pinmux_config opamp0_input_pin_conf;
	system_pinmux_get_config_defaults(&opamp0_input_pin_conf);
	opamp0_input_pin_conf.direction    = SYSTEM_PINMUX_PIN_DIR_INPUT;
	opamp0_input_pin_conf.mux_position = OPAMP0_INPUT_MUX;
	system_pinmux_pin_set_config(OPAMP0_INPUT_PIN, &opamp0_input_pin_conf);

	/* Initialize and enable the OPAMP2 with the user settings. */
	opamp1_set_config(&conf);
	opamp_enable(OPAMP_0);
	
	/* Wait for the output ready. */
	while(!opamp_is_ready(OPAMP_0));
}

enum status_code err;
/* Configure ADC */
void configure_adc(void)
{
	/* Creates a new configuration structure for the ADC */
	struct adc_config config_adc_ch1;

	adc_get_config_defaults(&config_adc_ch1);
	
	/* Setup ADC with OPAMP2 output as ADC input */
	config_adc_ch1.clock_prescaler = ADC_CLOCK_PRESCALER_DIV8;
	config_adc_ch1.positive_input  = ADC_POSITIVE_INPUT_OPAMP2;
	config_adc_ch1.negative_input  = ADC_NEGATIVE_INPUT_GND;
	
	/* Initialize and enable ADC */
	err = adc_init(&adc_instance_ch1, ADC, &config_adc_ch1);
	err = adc_enable(&adc_instance_ch1);
		
}

/* Enable ADC Callback Function */
void configure_adc_callbacks(void)
{
	adc_register_callback(&adc_instance_ch1,
			adc_complete_callback_ch1, ADC_CALLBACK_READ_BUFFER);
	adc_enable_callback(&adc_instance_ch1, ADC_CALLBACK_READ_BUFFER);
	
	adc_register_callback(&adc_instance_ch2,
			adc_complete_callback_ch2, ADC_CALLBACK_READ_BUFFER);
	adc_enable_callback(&adc_instance_ch2, ADC_CALLBACK_READ_BUFFER);
}

/* ADC Callback Function */
void adc_complete_callback_ch1(struct adc_module *const module)
{

	integrale_ch1 = media(adc_result_buffer_ch1,1,ADC_SAMPLES);
	
	/************************/
	err = adc_disable(&adc_instance_ch1);
	/* Creates a new configuration structure for the ADC */
	struct adc_config config_adc_ch2;
	adc_get_config_defaults(&config_adc_ch2);
	/* Setup ADC with OPAMP2 output as ADC input */
	config_adc_ch2.resolution = ADC_RESOLUTION_CUSTOM;
	config_adc_ch2.accumulate_samples = ADC_ACCUMULATE_SAMPLES_8;
	config_adc_ch2.divide_result = ADC_DIVIDE_RESULT_8;
	config_adc_ch2.clock_prescaler = ADC_CLOCK_PRESCALER_DIV8;
	config_adc_ch2.positive_input  = ADC_POSITIVE_INPUT_PIN6;
	config_adc_ch2.negative_input  = ADC_NEGATIVE_INPUT_GND;//ADC_NEGATIVE_INPUT_PIN6;
	/* Initialize and enable ADC */
	err = adc_init(&adc_instance_ch2, ADC, &config_adc_ch2);
	err = adc_enable(&adc_instance_ch2);
	
	adc_register_callback(&adc_instance_ch2,
			adc_complete_callback_ch2, ADC_CALLBACK_READ_BUFFER);
	adc_enable_callback(&adc_instance_ch2, ADC_CALLBACK_READ_BUFFER);
	
	memset(adc_result_buffer_ch2,0,ADC_SAMPLES);
	adc_read_buffer_job(&adc_instance_ch2, adc_result_buffer_ch2, n_sample);
	
}

void adc_complete_callback_ch2(struct adc_module *const module)
{
	integrale_ch2 = media(adc_result_buffer_ch2,1,ADC_SAMPLES);
	
	/************************/
	err = adc_disable(&adc_instance_ch2);
	/* Creates a new configuration structure for the ADC */
	struct adc_config config_adc_ch1;
	adc_get_config_defaults(&config_adc_ch1);
	/* Setup ADC with OPAMP2 output as ADC input */
	config_adc_ch1.resolution = ADC_RESOLUTION_CUSTOM;
	config_adc_ch1.accumulate_samples = ADC_ACCUMULATE_SAMPLES_8;
	config_adc_ch1.divide_result = ADC_DIVIDE_RESULT_8;
	config_adc_ch1.clock_prescaler = ADC_CLOCK_PRESCALER_DIV8;
	config_adc_ch1.positive_input  = ADC_POSITIVE_INPUT_OPAMP2;
	config_adc_ch1.negative_input  = ADC_NEGATIVE_INPUT_GND;//ADC_NEGATIVE_INPUT_PIN6;
	/* Initialize and enable ADC */
	err = adc_init(&adc_instance_ch1, ADC, &config_adc_ch1);
	err = adc_enable(&adc_instance_ch1);
	
	adc_register_callback(&adc_instance_ch1,
			adc_complete_callback_ch1, ADC_CALLBACK_READ_BUFFER);
	adc_enable_callback(&adc_instance_ch1, ADC_CALLBACK_READ_BUFFER);
	
	//memset(adc_result_buffer_ch1,0,ADC_SAMPLES);
	//adc_read_buffer_job(&adc_instance_ch1, adc_result_buffer_ch1, n_sample);
}


static void configure_usart(void)
{
	uint8_t instance_index;

	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	config_usart.baudrate    = 115200;
	config_usart.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	config_usart.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	config_usart.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	config_usart.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	config_usart.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	stdio_serial_init(&usart_instance, EDBG_CDC_MODULE, &config_usart);
	
	// Inject our own interrupt handler
	instance_index = _sercom_get_sercom_inst_index(EDBG_CDC_MODULE);
	_sercom_set_handler(instance_index, rx_handler);

	// Enable the UART transceiver
	usart_enable(&usart_instance);
	usart_enable_transceiver(&usart_instance, USART_TRANSCEIVER_TX);
	usart_enable_transceiver(&usart_instance, USART_TRANSCEIVER_RX);

	// ..and the RX Complete interrupt
	((SercomUsart *)EDBG_CDC_MODULE)->INTENSET.reg = SERCOM_USART_INTFLAG_RXC;
		
}


/* Queue structure */
#define QUEUE_ELEMENTS 100
#define QUEUE_SIZE (QUEUE_ELEMENTS + 1)
uint16_t Queue[QUEUE_SIZE];
uint16_t QueueIn, QueueOut;

void QueueInit(void);
int QueuePut(uint16_t new);
int QueueGet(uint16_t *old);

void QueueInit(void)
{
	QueueIn = QueueOut = 0;
}

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

struct i2c_master_module i2c_master_instance;
#define CONF_I2C_MASTER_MODULE    SERCOM2
void configure_i2c_master(void);
void configure_i2c_master(void)
{

	struct i2c_master_config config_i2c_master;
	i2c_master_get_config_defaults(&config_i2c_master);
	config_i2c_master.buffer_timeout = 10000;
	i2c_master_init(&i2c_master_instance, CONF_I2C_MASTER_MODULE, &config_i2c_master);
	i2c_master_enable(&i2c_master_instance);
}

/* Main function */

uint8_t tmp1,tmp2;

int main(void)
{
	
	
	system_interrupt_disable_global();
	
	system_init();
	delay_init();
	configure_i2c_master();
	configure_usart();
	configure_adc();
	configure_adc_callbacks();
	
	/************************ FIR FILTER *****************************/
	//status = arm_fir_init_q15(&sFirStru,TAPNUMBER,&lpFilterCoefficent[0],&FirState[0],ADC_SAMPLES);
		
	
 	m24FC1025_write_byte(0,0x55);
 	m24FC1025_write_byte(1,0xaa);
// 
// 
 	tmp1 = m24FC1025_read_byte(0);
 	tmp2 = m24FC1025_read_byte(1);

	MAX5435M_write_byte(MAX5435_REGISTER_VREG,trimmer_value(analog_gain_2) );
	MAX5435L_write_byte(MAX5435_REGISTER_VREG,trimmer_value(analog_gain_2) );
	
	lis302dl_write_configuration();
	
	uint8_t lis302dl_who = lis302dl_read_byte(LIS302DL_REG_WHO_AM_I);

	/* Initialize OPAMP2 and ADC */
	//configure_opamp0(OPAMP_POT_MUX_8R_8R);
	configure_opamp2(OPAMP_POT_MUX_8R_8R);



// 	if(load_configuration_param(&piezoboard_parameters))
// 	{
// 		//n_sample = piezoboard_parameters.ncamp_preset;
// 		//analog_gain = piezoboard_parameters.analog_gain_preset;
// 		//threshold_value = piezoboard_parameters.threshold_preset;
// 		//pick_count_max = piezoboard_parameters.pick_count_limit;
// 	}
	

	
	//display_mutex  = xSemaphoreCreateMutex();
	//terminal_mutex = xSemaphoreCreateMutex();
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

static void main_task(void *params)
{
	int i = 0;
	uint16_t old_result;
	QueueInit();
	QueuePut(0xAA);
	
	
	memset(adc_result_buffer_ch1,0,ADC_SAMPLES);
	adc_read_buffer_job(&adc_instance_ch1, adc_result_buffer_ch1, n_sample);

	for(;;) {
		
		lis302dl_read_all();
		
		 acc_x = rd_buffer[40];
		 acc_y = rd_buffer[42];
		 acc_z = rd_buffer[44];
		 
		 
		//memset(adc_result_buffer_ch1,0,ADC_SAMPLES);
		//adc_read_buffer_job(&adc_instance_ch1, adc_result_buffer_ch1, n_sample);
		
		//if (adc_read_done == true) {
			
			//adc_read_done = false;
			
// 			for(i = ADC_SAMPLES/2 ; i = ADC_SAMPLES; i++){
// 				adc_result_buffer_filtered[i] = adc_result_buffer[i*2];
// 			}
			
			//integrale = media(adc_result_buffer,ADC_SAMPLES/2,ADC_SAMPLES);
			
			//arm_fir_fast_q15(&sFirStru,adc_result_buffer,adc_result_buffer_filtered,n_sample);
			//integrale = media(adc_result_buffer_filtered,n_sample);
			
			
			if(integrale_ch1 < threshold_value)
			{
				if(pick_count_up < PICK_COUNT_UP_DEFAULT)
					pick_count_up++;
				else
				{
					pick_count_up = 0;
					pick_count_down = PICK_COUNT_DOWN_DEFAULT;	
					alarm_detected = 1;
					//port_pin_set_output_level(LED_0_PIN, LED_0_INACTIVE);
					//port_pin_set_output_level(LED_1_PIN, LED_1_INACTIVE);
					
					//delay_ms(3000);
				}
			}
			else
			{
				if(pick_count_down > 0)
					pick_count_down--;
				else
				{
					pick_count_down = PICK_COUNT_DOWN_DEFAULT;
					pick_count_up = 0;
					alarm_detected = 0;
					//port_pin_set_output_level(LED_0_PIN, LED_0_ACTIVE);
					//port_pin_set_output_level(LED_1_PIN, LED_1_ACTIVE);
				}
			}
			
// 			if(pick_count == pick_count_max)
// 			{
// 				alarm_detected = 1;
// 				port_pin_set_output_level(LED_0_PIN, LED_0_INACTIVE);
// 			}
// 			else
// 			{
// 				alarm_detected = 0;
// 				port_pin_set_output_level(LED_0_PIN, LED_0_ACTIVE);
// 			}
		//}
			

		
		if(change_gain == true)
		{
			enum opamp_pot_mux temp_gain = OPAMP_POT_MUX_14R_2R;
			switch(analog_gain_1){
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


uint16_t media(uint16_t *buffer,uint16_t n_init_camp,uint16_t length_buffer)
{
	long tmp = 0;
	for(i=n_init_camp;i<length_buffer;i++)
	{
		tmp += buffer[i];
	}
	return (uint16_t)(tmp/(length_buffer-n_init_camp));
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
									
									memset(adc_result_buffer_ch1,0,ADC_SAMPLES);
									adc_read_buffer_job(&adc_instance_ch1, adc_result_buffer_ch1, n_sample);
	

									pkt.payload[0]=(uint8_t)integrale_ch1;
									pkt.payload[1]=(uint8_t)(integrale_ch1>>8);									
									pkt.payload[2]=(uint8_t)integrale_ch2;
									pkt.payload[3]=(uint8_t)(integrale_ch2>>8);
									
									pkt.payload[4]=(uint8_t)(alarm_detected);
									pkt.payload[5]=(uint8_t)(acc_x);
									pkt.payload[6]=(uint8_t)(acc_y);
									pkt.payload[7]=(uint8_t)(acc_z);
									usart_write_buffer_wait(&usart_instance,(uint8_t *)&pkt,sizeof(packet));
									delay_ms(25);
							}
							else if(pkt_temp_ptr->sub_cmd == sb_cmd_array[get_ncamp])
							{
								pkt.sub_cmd = pkt_temp_ptr->sub_cmd;
								//pkt.payload[0]=(uint8_t)result;
								//pkt.payload[1]=(uint8_t)(result>>8);
								pkt.payload[0]=(uint8_t)n_sample;
								pkt.payload[1]=(uint8_t)(n_sample>>8);
								usart_write_buffer_wait(&usart_instance,(uint8_t *)&pkt,sizeof(packet));
								delay_ms(25);
							}
							else if(pkt_temp_ptr->sub_cmd == sb_cmd_array[get_threshold])
							{
								pkt.sub_cmd = pkt_temp_ptr->sub_cmd;
								//pkt.payload[0]=(uint8_t)result;
								//pkt.payload[1]=(uint8_t)(result>>8);
								pkt.payload[0]=(uint8_t)threshold_value;
								pkt.payload[1]=(uint8_t)(threshold_value>>8);
								usart_write_buffer_wait(&usart_instance,(uint8_t *)&pkt,sizeof(packet));
								delay_ms(25);
							}
							else if(pkt_temp_ptr->sub_cmd == sb_cmd_array[get_analog_gain_1])
							{
								pkt.sub_cmd = pkt_temp_ptr->sub_cmd;
								//pkt.payload[0]=(uint8_t)result;
								//pkt.payload[1]=(uint8_t)(result>>8);
								pkt.payload[0]=(uint8_t)analog_gain_1;
								usart_write_buffer_wait(&usart_instance,(uint8_t *)&pkt,sizeof(packet));
								delay_ms(25);
							}
// 							else if(pkt_temp_ptr->sub_cmd == sb_cmd_array[get_pick_count_par])
// 							{
// 								pkt.sub_cmd = pkt_temp_ptr->sub_cmd;
// 								pkt.payload[0]=(uint8_t)pick_count_max;
// 								pkt.payload[1]=(uint8_t)(pick_count_max>>8);
// 								usart_write_buffer_wait(&usart_instance,(uint8_t *)&pkt,sizeof(packet));
// 								delay_ms(25);
// 							}
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
							else if(pkt_temp_ptr->sub_cmd == sb_cmd_array[set_analog_gain_1])
							{
								uint8_t temp = pkt_temp_ptr->payload[0];
								if(analog_gain_1 != temp)
								{
									analog_gain_1 = temp;
									change_gain = true;
								}
								usart_write_buffer_wait(&usart_instance,(uint8_t *)&pkt,sizeof(packet));
								delay_ms(25);
							}
							else if (pkt_temp_ptr->sub_cmd == sb_cmd_array[save_param])
							{
								save_configuration_param();
								usart_write_buffer_wait(&usart_instance,(uint8_t *)&pkt,sizeof(packet));
								delay_ms(25);
							}
// 							else if (pkt_temp_ptr->sub_cmd == sb_cmd_array[pick_count_par])
// 							{
// 								pick_count_max = pkt_temp_ptr->payload[1] << 8 | (pkt_temp_ptr->payload[0]);
// 								usart_write_buffer_wait(&usart_instance,(uint8_t *)&pkt,sizeof(packet));
// 								delay_ms(25);
// 							}
							else if(pkt_temp_ptr->sub_cmd == sb_cmd_array[set_analog_gain_2])
							{
								uint8_t temp = pkt_temp_ptr->payload[0];
								if(analog_gain_2 != temp)
								{
									analog_gain_2 = temp;
									MAX5435M_write_byte(MAX5435_REGISTER_VREG,trimmer_value(analog_gain_2) );
									MAX5435L_write_byte(MAX5435_REGISTER_VREG,trimmer_value(analog_gain_2) );
								}
								usart_write_buffer_wait(&usart_instance,(uint8_t *)&pkt,sizeof(packet));
								delay_ms(25);
							}
							
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
	SercomUsart *const usart_hw = (SercomUsart *)EDBG_CDC_MODULE;

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


void save_configuration_param(void)
{
		//sprintf(piezoboard_parameters.STX_EEPROM,"%s",STX_EEPROM_STR);
		//sprintf(piezoboard_parameters.ETX_EEPROM,"%s",ETX_EEPROM_STR);
		
		piezoboard_parameters.STX_EEPROM = STX_EEPROM_STR;
		piezoboard_parameters.ETX_EEPROM = ETX_EEPROM_STR;
		
		piezoboard_parameters.analog_gain_preset = analog_gain_1;
		piezoboard_parameters.ncamp_preset = n_sample;
		piezoboard_parameters.threshold_preset = threshold_value;
		//piezoboard_parameters.pick_count_limit = pick_count_max;
		
		////crc8(&piezoboard_parameters.CRC,piezoboard_parameters.analog_gain_preset);
		////crc8(&piezoboard_parameters.CRC,piezoboard_parameters.ncamp_preset);
		////crc8(&piezoboard_parameters.CRC,piezoboard_parameters.threshold_preset);
		
		//at24cxx_write_continuous(0,sizeof(piezoboard_parameters),(uint8_t*)&piezoboard_parameters);
}

bool load_configuration_param(struct configuration_param *piezoboard_parameters_tmp)
{
	//at24cxx_read_continuous(0,sizeof(piezoboard_parameters),(uint8_t*)piezoboard_parameters_tmp);
	
// 	if(	(strcmp(piezoboard_parameters_tmp->STX_EEPROM,STX_EEPROM_STR) == 0)	&&
// 		(strcmp(piezoboard_parameters_tmp->ETX_EEPROM,ETX_EEPROM_STR) == 0)	)
	if(	(piezoboard_parameters_tmp->STX_EEPROM == STX_EEPROM_STR)	&&
		(piezoboard_parameters_tmp->ETX_EEPROM == ETX_EEPROM_STR)	)
	{
		return true;
	}
	return false;
}

