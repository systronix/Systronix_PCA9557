/**
 PCA9557.h, based on started PCAL9535A library
 Revisions
 
 2016 May 09 bboyes changing SALT1 bit mapping to reflect actual SALT 1v00 board layout
 2015 Apr 22 bboyes start, based on incomplete PCAL9535A library as used on ARV2
 
 */
 

 

// what does this do??
#ifndef PCA9557_H_
#define PCA9557_H_

#include<Arduino.h>

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
#define	PCA9557_PIN_DIR_REG	0x03

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
		uint8_t _out_data;

		/*
		 * Here are bit masks for the bits written to _out_data
		 * We want these private because the caller should not
		 * be manipulating these directly but using access methods instead
		 * There is an error in SALT 1v00 in schematic bit assignment at the '9557
		 * and at the J28 connector.
		 */
		// These are what we intended the bits to be. Only IO 6 and 4 are correct
		// const uint8_t _RCLK595 = 0x80;	// only used by '595, must idle high
		// const uint8_t _SCLK = 0x40;		// serial clock, used by both 595 and 165
		// const uint8_t _SDOUT = 0x20;	// output from 9577 to '595 Din
		// const uint8_t _SDIN = 0x10;		// input to 9577 from '165 Qout
		// const uint8_t _SHIFT165 = 0x08;	// SHIFT(H), LOAD(L) only used by '165
		
		// In SALT1 this is how the 9557 output bits actually map to AC and DC board signals
		const uint8_t _SHIFT165 = 0x80;	// SHIFT(H), LOAD(L) only used by '165	
		const uint8_t _SCLK = 0x40;		// serial clock, used by both 595 and 165
		const uint8_t _RCLK595 = 0x20;	// only used by '595, must idle high		
		const uint8_t _SDIN = 0x10;		// input to 9577 from '165 Qout
		const uint8_t _SDOUT = 0x08;	// output from 9577 to '595 Din		

		const uint8_t _LED = 0x01;		// open drain, clearing this bit drives output low. LED on

		// bit 5 is input; bits 2 and 1 are not connected, Bit 0 is the activity LED for that device
		// so this mask has a '1' for every bit which is used as output
		const uint8_t _OUTMASK = _RCLK595|_SCLK|_SDOUT|_SHIFT165|_LED;	

		/*
		 * This is data coming back from '165 shift registers
		 */
		uint8_t _inp_data;

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
		Systronix_PCA9557(uint8_t base);		// constructor
		void begin(void);
		uint8_t control_write (uint8_t data);
		uint8_t register_write (uint8_t reg, uint8_t data);
		uint8_t default_read ();
		uint8_t BaseAddr;    // I2C address, only the low 7 bits matter

		uint8_t sclk_pulse ();	// make this private later??
		uint8_t rclk_pulse ();
		uint8_t shift_out_16bits (uint16_t data);
		uint8_t shift_out_32bits (uint32_t data);



	private:

};


	
#endif	// PCA9557_H_
