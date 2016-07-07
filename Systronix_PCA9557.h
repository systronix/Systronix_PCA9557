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
	   // Instance-specific properties
		uint8_t _base; // base address, eight possible values
		
		/*
		 * data being written to output port, this is manipulated to toggle
		 * bits which drive '595 and '165 shift register clocks downstream
		 * This is private so instances can't trample it
		 */
		uint8_t _out_data = 0;

		/*
		 * This is a byte of data coming back from '165 shift registers
		 */
		uint8_t _inp_data = 0;
		
		uint8_t	_invert_mask = 0;

		/**
		 * Data for one instance of a Systronix_PCA9557 register.
		 * NOT USING THIS CURRENTLY
		**/
		struct data {
		  uint8_t address;    // I2C address, only the low 7 bits matter
		  uint8_t command;
		  uint8_t data;
		  uint16_t i2c_err_nak;  // total since startup
		  uint16_t i2c_err_rd;   // total read fails - data not there when expected
		};
   
	public:
		void setup(uint8_t base);		// constructor
		void begin(void);	// joins I2C as master
		void init(uint8_t out_mask, uint8_t out_data, uint8_t inp_invert);	// sets regs
		uint8_t control_write (uint8_t data);
		uint8_t register_write (uint8_t reg, uint8_t data);
		uint8_t default_read ();
		uint8_t BaseAddr;    // I2C address, only the low 7 bits matter

		uint8_t pin_pulse (uint8_t pin_mask, boolean idle_high);
		uint8_t pin_drive (uint8_t pin_mask, boolean high);
		
		uint8_t input_read ();
		uint8_t output_read ();

	private:

};


	
#endif	// PCA9557_H_


