/****************************************************************************
	Systronix_PCA9557.cpp

	Author: bboyes
 
 Based on Rev. 06 — 11 June 2008 of the NXP data sheet
 
 Revisions
 2016 May 20 bboyes major changes to take out application-specific features and
					make this a generic library
 2015 Apr 22 bboyes start, based on incomplete PCAL9535A library as used on ARV2

***************************************************************************/
 
/****************************************************************************
	DEVICE I2C BASE ADDRESS
	
 There are three address bits for 8 addresses
 Looking at the address bits in binary:
 0b0100 xxx[R/W]
 I2C has a 7-bit device address with bit 0 actually the R(H)/W(L) bit
 Right-shifting to not include the R/W bit, the range of base addresses is
 0b0001 1xxx which is 0x18-0x1F which is how Arduino refers to it.
 
 Some include the R/W bit which, IMO, makes the addresses more confusing,
 since each actual address has both a "read" and "write" version.

 NOTES ABOUT WIRE

Wire.endTransmission() seems to be only intended for use with a master write.
Wire.requestFrom() is used to get bytes from a slave, with read().

So to read three bytes, do a requestFrom(address, 2, true). << insists the first param is int.
Compiler has major whines if called as shown in the online Wire reference.

 NOTE: We are actually using i2c_t3 optimized for Teensy 3.x but should 
 still work with Wire on Teensy or other Arduinos
 
***************************************************************************/
 
/****************************************************************************
 * Accessing the PCA9557
 *
 * Reading and writing will affect the register currently addressed in the
 * two lsbs of the "control" register, which is write-only (in spite of NXP
 * datasheet claims that it can be read) and must be written immediately
 * after sending the I2C device address.
 *
 * So every write must start with an address byte, then the command/control
 * byte, then the data to be written to the location in the control register.
 *
 * Reads don't need the command byte: they will read the location currently
 * stored in the control register.
 *
***************************************************************************/

#include <Arduino.h>
#include "Systronix_PCA9557.h"

#define _DEBUG 0

/**************************************************************************/
/*!
    @brief  Instantiates a new PCA9557 class to use the given base address
	@todo	Test base address for legal range 0x18..0x1F
*/
/**************************************************************************/
Systronix_PCA9557::Systronix_PCA9557(uint8_t base)
{
	_base = base;
	BaseAddr = base;
	static data _data;		// instance of the data struct
	_data.address = _base;	// struct

	_out_data = _SCLK | _RCLK595;	// clocks high to start

	_inp_data = 0;

}

/**************************************************************************/
/*!
    @brief  Join the I2C bus as a master
*/
/**************************************************************************/
void Systronix_PCA9557::begin(void) {
	// 
	Wire.begin();	// join I2C as master
	
	// @TODO add default read first thing to see if it is the control reg
	// but even if it is, is it any use? How would we know what to expect?

	// clear pin dir reg bits to 0 for all outputs
	// datasheet calls this config register
	// 0 bit = that bit is output
	register_write(PCA9557_PIN_DIR_REG, ~_OUTMASK);

	// don't invert any inputs
	register_write(PCA9557_INP_INVERT_REG, 0x00);

	// init outputs 
	register_write(PCA9557_OUT_PORT_REG, _out_data);
}



/*
	Write to the control register. This is the register accessed
	after the 9557 receives a valid slave address and ACKs it.
	The next data written by the master sets the 2 LSBs in the control register.
	Note that the control register itself does not have a "register address".
	It can also be written, but how?	
	
	Only 2 lsbs matter: they act as address to four device I/O registers:
	00 read only input port register (writes have no effect)
		??? Are writes ACKed?
		read the value on the actual device pin
	01 read/write output port register 
		write to output reg F/F, not the actual pins
		config reg bit must also be cleared to let that F/F drive its pin
		(if all I/O are inputs this is just an 8-bit storage register)
		Note that bit 0 (only) is open-drain.
		POR state: all bits cleared
	10 read/write polarity inversion register 
		set bit = that pin's input value is inverted when read
		POR state: bits 7..4 are set, bits 3..0 are cleared.
	11 read/write configuration register 
		low bit = that pin is output pin. 
		That bit's output F/F out is actually driven to that pin.
		POR state = all bits set (all are inputs)
	
	All this method does is set those register address bits 
	in the control register. It does not then write to or 
	read from the register which is now addressed.

	returns # of bytes written which should be 1, 2 if error?
 */
uint8_t Systronix_PCA9557::control_write (uint8_t data)
{
	uint8_t b = 0;
	  Wire.beginTransmission(_base);

	  // returns # of bytes written
	  b = Wire.write(data);
	  // returns 0 if no error
	  b+= Wire.endTransmission();

	 return b;
}

/*
 * Write data to the register at location 'reg'
 *
 * returns # of bytes written which should be 2, 3 if error?
 *
 * @TODO check reg for valid range
 */
uint8_t Systronix_PCA9557::register_write (uint8_t reg, uint8_t data)
{
	uint8_t b = 0;
	
	Wire.beginTransmission(_base);
	// write data to control register only 2 lsbs matter
	// returns # of bytes written
	b = Wire.write(reg);
	b+= Wire.write(data);

	// returns 0 if no error
	b+= Wire.endTransmission();

	return b;
}

/**
 * Read a byte from whatever register is currently pointed to by the control register
 *
 * Prints an error message if there was not a byte ready from the I2C device
 */
uint8_t Systronix_PCA9557::default_read ()
{
	byte b=0;
	uint8_t recvd = 0xFF;	// init with error value
	b = Wire.requestFrom(_base, 1, true);

	if (1 != b)
	{
		Serial.print(" Error I2C read didn't return 1 but ");
		Serial.println(b);
	}
	recvd = Wire.read();
	return recvd;
}

/*
 * These are functions which use the library in a specific way for SALT
 * and should be moved into their own library.
 * @TODO move these into their own library
 */


 /*
 * Pulse pin(s) from idle state to active state and back to idle.
 * If idle is high then they will pulse low; idle low will pulse high
 * Leave with pin(s) in idle state.
 * Example: 
 * pin_pulse (0xFF, true) will pulse ALL pins H-L-H; 
 * 0x02 will pulse output 1
 *
 * This only actually drives pins defined as outputs in the config register
 *
 * @TODO decide what to return and add support for it
 * maybe return boolean true if all bits driven are actually output bits?
 * AND pin_mask with ~config reg value, it should be equal to the pin mask
 * return TRUE if mask are outputs and no I2C errors
 *
 * @param pin [0..0xFF] the device output pin(s) you want to pulse
 * @param idle_high if true otherwise will idle low
 */
uint8_t Systronix_PCA9557::pin_pulse (uint8_t pin_mask, boolean idle_high)
{
	uint8_t b = 0;
	
	Serial.print("pin_mask=0x"); Serial.println(pin_mask, HEX);

	if (idle_high)
	{
		_out_data &= (~pin_mask);	// clear outputs to be pulsed low
	}
	else
	{
		_out_data |= (pin_mask);	// set outputs to be pulsed	high	
	}
	// pulse outputs to non-idle state
	if (_DEBUG>0) {Serial.print("out_data=0x"); Serial.println(_out_data, HEX);}
	register_write(PCA9557_OUT_PORT_REG, _out_data);

	if (idle_high)
	{
		_out_data |= (pin_mask);	// set outputs to be idled high
	}
	else
	{
		_out_data &= (~pin_mask);	// clr outputs to be idled low		
	}
	// restore outputs to idle state
	if (_DEBUG>0) {Serial.print("out_data=0x"); Serial.println(_out_data, HEX);}
	register_write(PCA9557_OUT_PORT_REG, _out_data);
	
	return b;
}

/*
 * Assume clock is high when resting.
 * Pulse the serial bit clock low then back high.
 * Leave with clock high.
 *
 * @deprecated, use pins_pulse instead
 */
uint8_t Systronix_PCA9557::sclk_pulse ()
{
	uint8_t b = 0;

	// clear the clock bit
	_out_data &= (~_SCLK);
	register_write(PCA9557_OUT_PORT_REG, _out_data);

	// set the clock bit high
	_out_data |= _SCLK;
	register_write(PCA9557_OUT_PORT_REG, _out_data);

	 return b;
}

/*
 * Pulse rclk
 * Assume clock is high when resting.
 * Pulse the serial bit clock low then back high.
 * Leave with clock high.
 *
 * @deprecated, use pins_pulse instead
 */
uint8_t Systronix_PCA9557::rclk_pulse ()
{
	uint8_t b = 0;

	// clear the clock bit
	_out_data &= (~_RCLK595);
	_out_data &= (~_LED);
	register_write(PCA9557_OUT_PORT_REG, _out_data);

	// set the clock bit high
	_out_data |= _RCLK595;
	_out_data |= _LED;
	register_write(PCA9557_OUT_PORT_REG, _out_data);

	 return b;
}

/*
 * Data is sent lsb first at least for now
 *
 * @TODO move this to SALT-specific library
 */
uint8_t Systronix_PCA9557::shift_out_16bits (uint16_t data)
{
	uint8_t b = 0;

	uint16_t bit = data;

	for (uint8_t i=0; i<16; i++)
	{
		if ((bit & 0x01) == 1)
		{
			// set the bit
			_out_data |= (_SDOUT);
		}
		else
		{
			// clear the bit
			_out_data &= (~_SDOUT);
		}
		sclk_pulse ();		// change to pins_pulse

		// shift in next bit
		bit = bit >> 1;
	}

	rclk_pulse();	// change to pins_pulse
	return b;
}

/*
 * Data is sent lsb first at least for now
 * Shift 32 bits through presumed DC and AC registers, then latch it.
 *
 * @TODO move this to SALT-specific library
 */
uint8_t Systronix_PCA9557::shift_out_32bits (uint32_t data)
{
	uint8_t b = 0;

	uint32_t bit = data;

	for (uint8_t i=0; i<32; i++)
	{
		if ((bit & 0x01) == 1)
		{
			// set the bit
			_out_data |= (_SDOUT);
		}
		else
		{
			// clear the bit
			_out_data &= (~_SDOUT);
		}
		sclk_pulse ();	// change to pins_pulse

		// shift in next bit
		bit = bit >> 1;
	}

	rclk_pulse();	// change to pins_pulse
	return b;
}


