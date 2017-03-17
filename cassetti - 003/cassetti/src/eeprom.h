/*
 * eeprom.h
 *
 * Created: 20/12/2016 04:33:32
 *  Author: 10014498
 */ 

#include <asf.h>

/*! The I2C address is fixed for the AT24CXX device. */
#define m24FC1025CXX_TWI_ADDRESS                 0x56

/*! The AT42QT1060 can do max 100kHz on the TWI. */
#define m24FC1025CXX_TWI_MASTER_SPEED 100000



void m24FC1025_write_byte(uint16_t byte_address, uint8_t byte_value);


/*! \brief Write bytes continuously to the serial EEPROM.
 *
 *  \param[in] start_address Address of first byte in transaction.
 *  \param[in] length Number of bytes to write.
 *  \param[in] wr_buffer Pointer to array where the bytes to be written are stored.
 *
 *  \retval true Bytes written successfully.
 *  \retval false Bytes could not be written.
 */
void m24FC1025_write_continuous(uint16_t start_address, uint16_t length, uint8_t const *wr_buffer) ;


/*! \brief Read single byte from serial EEPROM.
 *
 *  \param[in] byte_address Address of byte to read.
 *  \param[out] read_byte Pointer to memory where the read byte will be stored.
 *
 *  \retval true Byte read successfully.
 *  \retval false Byte could not be read.
 */
uint8_t m24FC1025_read_byte(uint16_t byte_address) ;


/*! \brief Read bytes continuously from the serial EEPROM.
 *
 *  \param[in] start_address Address of first byte to read.
 *  \param[in] length Number of bytes to read.
 *  \param[out] rd_buffer Pointer to memory where the read bytes will be stored.
 *
 *  \retval true Bytes read successfully.
 *  \retval false Bytes could not be read.
 */
void m24FC1025_read_continuous(uint16_t start_address, uint16_t length, uint8_t *rd_buffer);