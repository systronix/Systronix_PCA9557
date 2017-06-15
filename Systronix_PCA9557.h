/**
 PCA9557.h, based on started PCAL9535A library
 Revisions
 
 2017 Jun 15 bboyes w/skendall, major review and cleanup
 2016 May 15 bboyes Conditional comp use of i2c_t3
 2016 May 09 bboyes changing SALT1 bit mapping to reflect actual SALT 1v00 board layout
 2015 Apr 22 bboyes start, based on incomplete PCAL9535A library as used on ARV2
 
 */
 

// include this only one time
#ifndef PCA9557_H_
#define PCA9557_H_

#include <Arduino.h>

// Include the lowest level I2C library
#if defined (__MK20DX256__) || defined (__MK20DX128__) 	// Teensy 3.1 or 3.2 || Teensy 3.0
#include <i2c_t3.h>		
#else
#include <Wire.h>	// for AVR I2C library
#endif

#define		SUCCESS	0
#define		FAIL	(~SUCCESS)
#define		ABSENT	0xFD

#define PCA9557_BASE_MIN 			0x18  // 7-bit address not including R/W bit
#define PCA9557_BASE_MAX 			0x1F  // 7-bit address not including R/W bit


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
		uint8_t		_out_reg = 0;						// reset state; or data last written to output port			:: initialized in init(); not necessary to do here [wsk]
		uint8_t		_inp_data = 0;						// undefined at reset; or data last read from the device's i/o pins
		uint8_t		_invert_reg = 0xF0;					// reset state; or last setting of the invert register			:: initialized in init() [wsk]
		uint8_t		_config_reg = 0xFF;					// reset state; or last setting of the configuration (data direction) register	:: initialized in init() [wsk]
		uint8_t		_control_reg = 0x03;				// undefined at reset; or setting last written (any write or some reads)		:: initialized in init() [wsk]
		char* 		_wire_name = (char*)"empty";
		i2c_t3		_wire = Wire;						// why is this assigned value = Wire? [bab]
		
		void		tally_errors (uint8_t);
   
	public:
		struct
			{
			uint8_t		ret_val;						// not clear what this really is
			boolean		exists;							// set false after an unsuccessful i2c transaction
			} control;


		/**
		Array of Wire.status() extended return code strings, 11 as of 29Dec16 i2c_t3 release
		index into this with the value of status.
		There is an array of constant text: const status_text[11]
		char * makes the decl an array of char pointers, each pointing to constant text
		the first const means that array of char pointers can't change.
		We can access this with a const char * text_ptr which means point to char(s) which happen to be const
		Note each literal string has a null terminator added by C compiler.
		See NAP_UI_key_defs.h for similar
		*/
#if defined I2C_T3_H 		
		const char * const status_text[12] =
		{
			"I2C_WAITING", 		// first four are not errors but status
			"I2C_SENDING", 
			"I2C_SEND_ADDR",
			"I2C_RECEIVING",
			"I2C_TIMEOUT", 		// start of 5 errors, status==4
			"I2C_ADDR_NAK", 
			"I2C_DATA_NAK",
			"I2C_ARB_LOST",
			"I2C_BUF_OVF",
			"I2C_SLAVE_TX", 	// slave status; not errors
			"I2C_SLAVE_RX",
			"SILLY_PROGRAMMER"	// Doh. Slap forehead.
		};
#else
		// Wire.h returns from endTransmission
		// 0=success, 1=data too long, 2=recv addr NACK, 3=recv data NACK, 4=other error
		const char * const status_text[5] =
		{
			"Success",		// TODO not an error, we should not tally it! 
			"Data length",
			"Receive addr NAK", 
			"Receive data NAK",
			"Other error"
		};		
#endif

		/** error stucture
		Note that this can be written by a library user, so it could be cleared if desired as part of 
		some error recovery or logging operation. It could also be inadvertenly erased...

		successful_count overflowed at 258.5 hours. Making this a 64-bit unsigned (long long) allows
		for 2**32 times as many hours. So not likely to ever wrap wrap.
		*/
		struct
			{
			uint8_t		error_val;						// the most recent error value, not just SUCCESS or FAIL
			uint32_t	incomplete_write_count;			// Wire.write failed to write all of the data to tx_buffer
			uint32_t	data_len_error_count;			// data too long
			uint32_t	timeout_count;					// slave response took too long
			uint32_t	rcv_addr_nack_count;			// slave did not ack address
			uint32_t	rcv_data_nack_count;			// slave did not ack data
			uint32_t	arbitration_lost_count;
			uint32_t	buffer_overflow_count;
			uint32_t	other_error_count;				// from endTransmission there is "other" error
			uint32_t	unknown_error_count;
			uint32_t	data_value_error_count;			// I2C message OK but value read was wrong; how can this be?
			uint32_t	silly_programmer_error;			// I2C address to big or something else that "should never happen"
			uint64_t	total_error_count;				// quick check to see if any have happened
			uint64_t	successful_count;				// successful access cycle
			} error;

		 

		char*		wire_name;	// name of Wire, Wire1, etc in use

					Systronix_PCA9557();		// constructor

		void		setup (uint8_t base);		// defaults to Wire net
		void		setup (uint8_t base, i2c_t3 wire, char* name);					// initialize 
		void 		begin(i2c_pins pins, i2c_rate rate);	// with pins and rate
		void		begin(void);
		uint8_t		init (uint8_t, uint8_t, uint8_t);		// sets regs
		
		uint8_t		control_write (uint8_t target_register);
		uint8_t		register_write (uint8_t target_register, uint8_t data);
		uint8_t 	register_read (uint8_t target_register, uint8_t* data_ptr);	
		uint8_t		default_read (uint8_t*);
		uint8_t		input_read (uint8_t*);		// deprecated, use register_read
		uint8_t		output_read (uint8_t*);		// deprecated, use register_read
		uint8_t		base_get(void);
		uint8_t 	self_test(uint8_t ignore_pins);		// TODO: write this!

		
//---- TODO: these deprecated since 22 August 2016; delete now?
		uint8_t		default_read (void);				// this function deprecated
		uint8_t		input_read (void);					// this function deprecated
		uint8_t		output_read (void);					// this function deprecated
//----
		uint8_t		pin_pulse (uint8_t pin_mask, boolean);
		uint8_t		pin_drive (uint8_t pin_mask, boolean);
		
	private:

};


	
#endif	// PCA9557_H_


