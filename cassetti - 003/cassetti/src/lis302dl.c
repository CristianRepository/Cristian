#include <asf.h>
#include "lis302dl.h"
#include "crc8.h"



extern struct i2c_master_module i2c_master_instance;


uint8_t wr_buffer[DATA_LENGTH_WR] = {0x80,0X80};
uint8_t rd_buffer[DATA_LENGTH_RD];


static uint8_t config_lis[] = {
	0x80|LIS302DL_REG_CTRL1,
	0x47,
	0x80,
	LIS302DL_CTRL3_IHL|LIS302DL_CTRL3_PP_OD|0x01,//0xc7
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0xf0,//FF_WU_CFG_1
	0x00,
	0x0F,//FF_WU_THS_1
	0x00,
	0x00,//0x34
	0x00,//0x35
	0x00,//0x36
	0x00,//0x37
	0x7f,//0x38
	0x00,
	0x00,
	0x88,
	0x08,
};


void lis302dl_write_configuration(void) {
	

	struct i2c_master_packet packet = {
		.address     = LIS302DL_ADDRESS,
		.data_length = sizeof(config_lis),
		.data        = config_lis,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

	while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) != STATUS_OK);

	return;
}



void lis302dl_write_byte(uint8_t reg_address, uint8_t byte_value) {
	
	
	uint8_t pack[2];
	pack[0] = reg_address;
	pack[1] = byte_value;

	
	struct i2c_master_packet packet = {
		.address     = LIS302DL_ADDRESS,
		.data_length = sizeof(pack),
		.data        = pack,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

	while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) != STATUS_OK);

	return;
}


uint8_t lis302dl_read_byte(uint8_t reg_address) {

	uint8_t data;

	struct i2c_master_packet packet1 = {
		.address     = LIS302DL_ADDRESS,
		.data_length = 2,
		.data        = (uint8_t*)&reg_address,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

	while (i2c_master_write_packet_wait(&i2c_master_instance, &packet1) != STATUS_OK);

	struct i2c_master_packet packet2 = {
		.address     = LIS302DL_ADDRESS,
		.data_length = 1,
		.data        = &data,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

	while (i2c_master_read_packet_wait(&i2c_master_instance, &packet2) != STATUS_OK);

	return data;

}



void lis302dl_read_all(void) {

	uint8_t data;

	struct i2c_master_packet packet1 = {
		.address     = LIS302DL_ADDRESS,
		.data_length = DATA_LENGTH_WR,
		.data        = wr_buffer,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

	while (i2c_master_write_packet_wait(&i2c_master_instance, &packet1) != STATUS_OK);

	struct i2c_master_packet packet2 = {
		.address     = LIS302DL_ADDRESS,
		.data_length = DATA_LENGTH_RD,
		.data        = rd_buffer,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

	while (i2c_master_read_packet_wait(&i2c_master_instance, &packet2) != STATUS_OK);

	return data;

}




