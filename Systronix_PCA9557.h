/**
 PCA9557.h, based on started PCAL9535A library
 Revisions
 
 2016 May 15 bboyes Conditional comp use of i2c_t3
 2016 May 09 bboyes changing SALT1 bit mapping to reflect actual SALT 1v00 board layout
 2015 Apr 22 bboyes start, based on incomplete PCAL9535A library as used on ARV2
 
 */
 

// include this only one time
#ifndef PCA9557_H_
#define PCA9557_H_

#include<Arduino.h>

// Include the lowest level I2C library
#if defined (__MK20DX256__) || defined (__MK20DX128__) 	// Teensy 3.1 or 3.2 || Teensy 3.0
#include <i2c_t3.h>		
#else
#include <Wire.h>	// for AVR I2C library
#endif

#define		SUCCESS	0
#define		FAIL	0xFF
#define		ABSENT	0xFD


/*--------------------------- COMMAND REGISTER ----------------*/

/**
 * Input port is read-only. Reads actual state of all device
 * I/O pins, whether they are inputs or outputs.
 */
#define	PCA9557_INP_PORT_REG		0x00	// input port

/**
 * Output port is read/write, writing to input bits has no effect.
 * This internal reg outputs are qualified by the config pin dir register
 * Reading this reg returns the value of this internal register, NOT
 * the actual device pin. So reading this only confirms its setting.
 * Read the input port to read the actual value on all device I/O pins.
 */
#define	PCA9557_OUT_PORT_REG		0x01	// output port

/**
 * Polarity inversion register is read/write.
 * invert polarity of input port if bit is 1
 * POR=0xF0
 */
#define	PCA9557_INP_INVERT_REG	0x02

/**
 * Config register sets each bit's mode: 1=input, 0=output
 * POR = 0xFF (all I/O bits are inputs)
 */
#define	PCA9557_CONFIG_REG	0x03



class Systronix_PCA9557
{
	protected:
		// Instance-specific properties; protected so that they aren't trampled by outside forces
		uint8_t		_base;								// base address, eight possible values
		uint8_t		_out_reg = 0;						// reset state; or data last written to output port
		uint8_t		_inp_data;							// undefined at reset; or data last read from the device's i/o pins
		uint8_t		_invert_reg = 0xF0;					// reset state; or last setting of the invert register
		uint8_t		_config_reg = 0xFF;					// reset state; or last setting of the configuration (data direction) register
		uint8_t		_control_reg = 0xFF;				// undefined at reset; or setting last written (any write or some reads)
		
		void		tally_errors (uint8_t);
   
	public:
		struct
			{
			uint8_t		ret_val;						// i2c_t3 library return value from most recent transaction
			uint32_t	incomplete_write_count;			// Wire.write failed to write all of the data to tx_buffer
			uint32_t	data_len_error_count;			// data too long
			uint32_t	rcv_addr_nack_count;			// slave did not ack address
			uint32_t	rcv_data_nack_count;			// slave did not ack data
			uint32_t	other_error_count;				// arbitration lost or timeout
			boolean		exists;							// set false after an unsuccessful i2c transaction
			} control;

		uint8_t		BaseAddr;    // I2C address, only the low 7 bits matter

		void		setup (uint8_t);					// constructor
		void		begin (void);						// joins I2C as master
		uint8_t		init (uint8_t, uint8_t, uint8_t);	// sets regs
		
		uint8_t		control_write (uint8_t);
		uint8_t		register_write (uint8_t, uint8_t);
		uint8_t		default_read (uint8_t*);
		uint8_t		input_read (uint8_t*);
		uint8_t		output_read (uint8_t*);

		uint8_t		default_read (void);				// this function deprecated
		uint8_t		input_read (void);					// this function deprecated
		uint8_t		output_read (void);					// this function deprecated

		uint8_t		pin_pulse (uint8_t pin_mask, boolean);
		uint8_t		pin_drive (uint8_t pin_mask, boolean);
		
	private:

};


	
#endif	// PCA9557_H_


