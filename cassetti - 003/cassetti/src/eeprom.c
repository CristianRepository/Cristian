/*
 * eeprom.c
 *
 * Created: 20/12/2016 04:33:05
 *  Author: 10014498
 */ 
#include <asf.h>
#include "eeprom.h"
#include "crc8.h"

extern struct i2c_master_module i2c_master_instance;



void m24FC1025_write_byte(uint16_t byte_address, uint8_t byte_value) {
	
	
	uint8_t pack[3];
	pack[0] = (byte_address&0xFF00)>>8;
	pack[1] = byte_address&0xFF;
	pack[2] = byte_value;

	
	struct i2c_master_packet packet = {
		.address     = m24FC1025CXX_TWI_ADDRESS,
		.data_length = sizeof(pack),
		.data        = pack,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

	while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) != STATUS_OK);

	return;
}


void m24FC1025_write_continuous(uint16_t start_address, uint16_t length, uint8_t const *wr_buffer) {
	
	uint8_t pack[2+length];
	pack[0] = (start_address&0xFF00)>>8;
	pack[1] = start_address&0xFF;
	uint16_t idx;
	for (idx=0;idx<length;idx++)
	pack[2+idx] = wr_buffer[idx];
	
	struct i2c_master_packet packet = {
		.address     = m24FC1025CXX_TWI_ADDRESS,
		.data_length = sizeof(pack),
		.data        = pack,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

	while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) != STATUS_OK);

	return;
}


uint8_t m24FC1025_read_byte(uint16_t byte_address) {
		
	uint8_t data;

	struct i2c_master_packet packet1 = {
		.address     = m24FC1025CXX_TWI_ADDRESS,
		.data_length = 2,
		.data        = (uint8_t*)&byte_address,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

	while (i2c_master_write_packet_wait(&i2c_master_instance, &packet1) != STATUS_OK);
		
	struct i2c_master_packet packet2 = {
		.address     = m24FC1025CXX_TWI_ADDRESS,
		.data_length = 1,
		.data        = &data,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

	while (i2c_master_read_packet_wait(&i2c_master_instance, &packet2) != STATUS_OK);

	return data;

}


void m24FC1025_read_continuous(uint16_t start_address, uint16_t length, uint8_t *rd_buffer) {
	
		struct i2c_master_packet packet1 = {
			.address     = m24FC1025CXX_TWI_ADDRESS,
			.data_length = 2,
			.data        = (uint8_t*)&start_address,
			.ten_bit_address = false,
			.high_speed      = false,
			.hs_master_code  = 0x0,
		};

		while (i2c_master_write_packet_wait(&i2c_master_instance, &packet1) != STATUS_OK);
		
		struct i2c_master_packet packet2 = {
			.address     = m24FC1025CXX_TWI_ADDRESS,
			.data_length = length,
			.data        = rd_buffer,
			.ten_bit_address = false,
			.high_speed      = false,
			.hs_master_code  = 0x0,
		};

		while (i2c_master_read_packet_wait(&i2c_master_instance, &packet2) != STATUS_OK);

}
