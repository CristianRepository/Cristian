/*
 * MAX5435.c
 *
 * Created: 20/12/2016 04:33:05
 *  Author: 10014498
 */ 
#include <asf.h>
#include "MAX5435.h"
#include "crc8.h"


#define MAX5435_MAX_VALUE     50000


extern struct i2c_master_module i2c_master_instance;


uint8_t trimmer_value(uint8_t value)
{
	return  value << 3;
}


void MAX5435M_write_byte(uint8_t byte_address, uint8_t byte_value) {
	
	
	uint8_t pack[2];
	pack[0] = byte_address;
	pack[1] = byte_value;

	
	struct i2c_master_packet packet = {
		.address     = MAX5435M_ADDRESS,
		.data_length = sizeof(pack),
		.data        = pack,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

	while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) != STATUS_OK);

	return;
}

void MAX5435L_write_byte(uint8_t byte_address, uint8_t byte_value) {
	
	
	uint8_t pack[2];
	pack[0] = byte_address;
	pack[1] = byte_value;

	
	struct i2c_master_packet packet = {
		.address     = MAX5435L_ADDRESS,
		.data_length = sizeof(pack),
		.data        = pack,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

	while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) != STATUS_OK);

	return;
}


// void MAX5435M_read_byte(uint8_t byte_address, uint8_t byte_value) {
// 		
// 	uint8_t data;
// 
// 	struct i2c_master_packet packet1 = {
// 		.address     = AT24CXX_TWI_ADDRESS,
// 		.data_length = 2,
// 		.data        = (uint8_t*)&byte_address,
// 		.ten_bit_address = false,
// 		.high_speed      = false,
// 		.hs_master_code  = 0x0,
// 	};
// 
// 	while (i2c_master_write_packet_wait(&i2c_master_instance, &packet1) != STATUS_OK);
// 		
// 	struct i2c_master_packet packet2 = {
// 		.address     = AT24CXX_TWI_ADDRESS,
// 		.data_length = 1,
// 		.data        = &data,
// 		.ten_bit_address = false,
// 		.high_speed      = false,
// 		.hs_master_code  = 0x0,
// 	};
// 
// 	while (i2c_master_read_packet_wait(&i2c_master_instance, &packet2) != STATUS_OK);
// 
// 	return data;
// 
// }

// void MAX5435L_read_byte(uint8_t byte_address, uint8_t byte_value) {
// 	
// 	uint8_t data;
// 
// 	struct i2c_master_packet packet1 = {
// 		.address     = AT24CXX_TWI_ADDRESS,
// 		.data_length = 2,
// 		.data        = (uint8_t*)&byte_address,
// 		.ten_bit_address = false,
// 		.high_speed      = false,
// 		.hs_master_code  = 0x0,
// 	};
// 
// 	while (i2c_master_write_packet_wait(&i2c_master_instance, &packet1) != STATUS_OK);
// 	
// 	struct i2c_master_packet packet2 = {
// 		.address     = AT24CXX_TWI_ADDRESS,
// 		.data_length = 1,
// 		.data        = &data,
// 		.ten_bit_address = false,
// 		.high_speed      = false,
// 		.hs_master_code  = 0x0,
// 	};
// 
// 	while (i2c_master_read_packet_wait(&i2c_master_instance, &packet2) != STATUS_OK);
// 
// 	return data;
// 
// }


