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
Wire.requestFrom(address, quantity, stop) is used to get bytes from a slave, with read().
Be sure to set stop "true" to release the bus or it will send a restart 
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
//	static data _data;		// instance of the data struct
//	_data.address = _base;	// struct
	_inp_data = 0;
	_out_data = 0;
	_invert_mask = 0;

}

/**************************************************************************/
/*!
    @brief  Join the I2C bus as a master, call this once in setup()
*/
/**************************************************************************/
void Systronix_PCA9557::begin(void)
{
	Wire.begin();	// join I2C as master
	
	// @TODO add default read first thing to see if it is the control reg
	// but even if it is, is it any use? How would we know what to expect?
}

/**
 *  @brief Initialize the 9557 to a given state. Can be called as often as needed.
 *  
 *  Call after a hardware reset, if a reset can be caused programatically.
 *  @param outmask set bits will be outputs. 0xFF makes all pins outputs
 *  @param output value to write to output pins, writing a 1 drives outputs high
 *  except bit 0 which is open drain so drives low. 
 *  @param invertmask applies to pins set as inputs. Setting a mask bit inverts that
 *  input when it is read. After POR this reg is 0xF0
 */
void Systronix_PCA9557::init(uint8_t outmask, uint8_t output, uint8_t invertmask)
{
	// remember current invert mask
	_invert_mask = invertmask;
	// remember our current data output
	_out_data = output;

	// clear pin dir reg bits to 0 for all outputs
	// datasheet calls this config register
	// 0 bit = that bit is output
	register_write(PCA9557_CONFIG_REG, ~outmask);

	// input read bits to be inverted if the reg bit is set, 0 = not inverted
	register_write(PCA9557_INP_INVERT_REG, _invert_mask);

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
	00 read-only input port register (writes have no effect)
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

/**
 * Write data to the register at location 'reg'
 *
 * returns # of bytes written which should be 2, 3 if error?
 *
 * This can be used as a general output byte write function
 *
 * @TODO check reg for valid range
 **/
uint8_t Systronix_PCA9557::register_write (uint8_t reg, uint8_t data)
{
	uint8_t b = 0;
	
	if (PCA9557_OUT_PORT_REG == reg)
	{
		_out_data = data;	// update our remembered output reg value
	}
	
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

 /**
 * Pulse pin(s) from idle state to active state and back to idle.
 * If idle is high then they will pulse low; idle low will pulse high
 * Leave with pin(s) in idle state.
 * Example: 
 * pin_pulse (0xFF, true) will pulse ALL pins H-L-H; 
 * 0x02 will pulse output 1
 *
 * This only actually drives pins defined as outputs in the config register
 *  
 *  Note this only changes the state of the pins in pin_mask. Other outputs
 *  are tracked in a private variable and their state is unchanged.
 *
 * @TODO decide what to return and add support for it
 * maybe return boolean true if all bits driven are actually output bits?
 * AND pin_mask with ~config reg value, it should be equal to the pin mask
 * return TRUE if mask are outputs and no I2C errors
 *
 * @param pin [0..0xFF] the device output pin(s) you want to pulse
 * @param idle_high if true otherwise will idle low
 **/
uint8_t Systronix_PCA9557::pin_pulse (uint8_t pin_mask, boolean idle_high)
{
	uint8_t b = 0;
	
#if 0 < _DEBUG
	Serial.print("pin_mask=0x"); Serial.println(pin_mask, HEX);
#endif

	if (idle_high)
	{
		_out_data &= ((~pin_mask) & 0x0FF);	// clear outputs to be pulsed low
	}
	else
	{
		_out_data |= (pin_mask);	// set outputs to be pulsed	high	
	}
#if 0 < _DEBUG
	Serial.print("pulse out_data=0x"); Serial.println(_out_data, HEX);
#endif
	// pulse outputs to non-idle state
	register_write(PCA9557_OUT_PORT_REG, _out_data);

	if (idle_high)
	{
		_out_data |= (pin_mask);	// set outputs to be idled high
	}
	else
	{
		_out_data &= ((~pin_mask) & 0x0FF);	// clr outputs to be idled low		
	}
#if 0 < _DEBUG
	Serial.print("pulse out_data=0x"); Serial.println(_out_data, HEX);
#endif
	// restore outputs to idle state
	register_write(PCA9557_OUT_PORT_REG, _out_data);
	
	return b;
}

 /**
 * Drive pin(s) to a high or low voltage level
 * If boolean high is true they will be driven to a high level.
 * Leave with pin(s) in new state.
 * Example: 
 * pin_drive (0x02, true) 0x02 will drive output 1 to high level.
 * pin_drive (0xFF, true) will set pins IO7–IO1 high, IO0 will drive active low
 * NOTE: IO0 output IS opposite to all other outputs
 *
 * This only actually drives pins defined as outputs in the config register
 * 
 *  Note this only changes the state of the pins in pin_mask. Other outputs
 *  are tracked in a private variable and their state is unchanged.
 *
 * @TODO decide what to return and add support for it
 * See pin_pulse comments
 *
 * @param pin [0..0xFF] the device output pin(s) you want to drive to new state
 * @param high if true sets pin(s) high otherwise will drive to low level
 **/
uint8_t Systronix_PCA9557::pin_drive (uint8_t pin_mask, boolean high)
{
	uint8_t b = 0;
	
#if 0 < _DEBUG
	Serial.print("drive pin_mask=0x"); Serial.println(pin_mask, HEX);
#endif

	if (high)
	{
		_out_data |= (pin_mask);	// set outputs to be driven	high
	}
	else
	{
		_out_data &= ((~pin_mask) & 0x0FF);	// clear outputs to be driven low
			
	}
	// drive output(s) to new state
#if 0 < _DEBUG
	Serial.print("drive out_data=0x"); Serial.println(_out_data, HEX);
#endif
	register_write(PCA9557_OUT_PORT_REG, _out_data);
	
	return b;
}

/**
 *  Read the input register 0x00 and return its value
 *  
 *  TODO optionally filter off output bits?
 *  
 */
uint8_t Systronix_PCA9557::input_read ()
{
	uint8_t data_read;
	control_write (PCA9557_INP_PORT_REG);
	data_read = default_read();
	return data_read;
}

/**
 * Read the output register
 *  
 * Reading this reg returns the value of the internal "output register", 
 * NOT the actual device pin value. So reading this only confirms its setting.
 * Only device pins set as outputs in the PCA9557_CONFIG_REG will  
 * have the output register's pin value driven to its external device pin.
 * Read the input port to read the actual value on all device I/O pins, 
 * including any which are outputs.
 */
uint8_t Systronix_PCA9557::output_read ()
{
	uint8_t data_read;
	control_write (PCA9557_OUT_PORT_REG);
	data_read = default_read();
	return data_read;
}



 
 
 
 
 
 