/*
 * MAX5435.h
 *
 * Created: 20/12/2016 04:33:32
 *  Author: 10014498
 */ 

#include <asf.h>

/*! The I2C address is fixed for the AT24CXX device. */
#define MAX5435L_ADDRESS                 0x28
#define MAX5435M_ADDRESS                 0x2C

#define MAX5435_REGISTER_VREG            0x11
#define MAX5435_REGISTER_NVREG           0x21
#define MAX5435_REGISTER_NVREGxVREG      0x61
#define MAX5435_REGISTER_VREGxNVREG      0x51

/*! The AT42QT1060 can do max 100kHz on the TWI. */
#define MAX5435_MASTER_SPEED 100000

uint8_t trimmer_value(uint8_t percent);

void MAX5435M_write_byte(uint8_t byte_address, uint8_t byte_value);
void MAX5435L_write_byte(uint8_t byte_address, uint8_t byte_value);

// void MAX5435M_read_byte(uint8_t byte_address, uint8_t byte_value);
/*void MAX5435L_read_byte(uint8_t byte_address, uint8_t byte_value);*/