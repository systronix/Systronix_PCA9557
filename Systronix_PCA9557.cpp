/****************************************************************************
	Systronix_PCA9557.cpp

Be sure to set stop "true" to release the bus or it will send a restart 
So to read three bytes, do a requestFrom(address, 2, true). << insists the first param is int.
Compiler has major whines if called as shown in the online Wire reference.

	Author: bboyes
 
 Based on Rev. 06 — 11 June 2008 of the NXP data sheet
 
 Revisions
 2017 Jun 15 bboyes w/skendall, major review and cleanup. Added register_read

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

#include <Systronix_PCA9557.h>


#define _DEBUG 0

// default constructor to keep compiler from whining
Systronix_PCA9557::Systronix_PCA9557()
{
	// _name = "empty";
	// _wire = Wire;		// default Wire, overridden in setup()
}

/**************************************************************************/
/*!
    @brief  Instantiates a new PCA9557 class to use the given base address

	@param wire instance of i2c_t3 'object' Wire, Wire1, Wire2, Wire3
	@param name is a string such as Wire, Wire1, etc used for debug output, 
		also saved as wire_name
*/
/**************************************************************************/
uint8_t Systronix_PCA9557::setup(uint8_t base, i2c_t3 wire, char* name)
	{
	if ((PCA9557_BASE_MIN > base) || (PCA9557_BASE_MAX < base))
		{
		tally_transaction (SILLY_PROGRAMMER);
		return FAIL;
		}

	_base = base;
	_wire = wire;
	_wire_name = wire_name = name;		// protected and public
	return SUCCESS;
	}


/**************************************************************************/
/*!
    @brief  Join the I2C bus as a master, call this once in setup()
	
	Wire.begin() doesn't return anything

	There's no (void) version of begin. We want to force ourselves to 
	explicitly declare which I2C net and speed we are using.
*/
/**************************************************************************/
void Systronix_PCA9557::begin(i2c_pins pins, i2c_rate rate)
	{	
	// @TODO add default read first thing to see if it is the control reg
	// but even if it is, is it any use? How would we know what to expect?
	_wire.begin(I2C_MASTER, 0x00, pins, I2C_PULLUP_EXT, rate);	// join I2C as master
	Serial.printf("Lib begin %s\r\n", _wire_name);
	_wire.setDefaultTimeout(200000); // 200ms
	}


/**
*
*/
void Systronix_PCA9557::begin(void)
	{	
	// @TODO add default read first thing to see if it is the control reg
	// but even if it is, is it any use? How would we know what to expect?
	_wire.begin();	// join I2C as master
	_wire.setDefaultTimeout(200000); // 200ms
	}	


//---------------------------< B A S E _ G E T >--------------------------------------------------------------
/**
	return the I2C base address for this instance
*/
uint8_t Systronix_PCA9557::base_get(void)
{
	return _base;
}


//---------------------------< I N I T >----------------------------------------------------------------------
/**
 *  @brief Initialize the 9557 to a given state. Can be called as often as needed.
 *  
 *  Call after a hardware reset, if a reset can be caused programatically.
 *  @param config_reg set bits will be outputs. 0xFF makes all pins outputs.
 * 	Note: the 9557 config reg (which should have been called 'direction reg')
 *  uses a 1 to indicate an input, 0 an output. We think that's backwards
 *  so this param gets inverted within this function so that 1 bits = outputs.
 *  This means if you have an output mask of 1's such as 0x0F, you can write
 *  That mask to the config reg and 0x0F will be outputs.
 *  @param out_reg value to write to output pins, writing a 1 drives outputs high
 *  On IO0, open drain, a '1' allows IO0 to be pulled up; a '0' is drive low
 *  @param invert_reg applies to pins set as inputs. Setting a mask bit inverts that
 *  input when it is read. After POR this reg is 0xF0
 *
 * Note we don't do any extensive testing of the 9557 internals or pins here. 
 * That should be a separate function.
 *
 *  @return SUCCESS/0 if OK, ABSENT if could not even address the part, or FAIL if we could address it
 *  but then could not subsequently load its registers
 *  
 *  @TODO maybe take out the inversion of config? To be consistent with the data sheet
 */ 
uint8_t Systronix_PCA9557::init(uint8_t config_reg, uint8_t out_reg, uint8_t invert_reg)
{
	uint8_t ret_val;
	error.exists = true;					// so we can use control_write; we'll find out later if device does not exist

	Serial.printf("Lib init %s at base 0x%.2X\r\n", _wire_name, _base);

	ret_val = control_write(PCA9557_OUT_PORT_REG);		// if successful this means we got two ACKs from slave device
	if (SUCCESS != ret_val)
		{
		Serial.printf("Lib init failed with 0x%.2X\r\n", error.error_val);
		error.exists = false;			// only place error.exists is set false
		return ABSENT;								
		}

	ret_val = register_write(PCA9557_OUT_PORT_REG, out_reg);		// init output reg first so that it is in correct state when config reg written
	ret_val |= register_write(PCA9557_CONFIG_REG, ~config_reg);		// clear pin dir reg bits to 0 for all outputs
	ret_val |= register_write(PCA9557_INP_INVERT_REG, invert_reg);	// 1 = input read bits inverted, 0 = not inverted

	if (ret_val)									// if anything other than SUCCESS: FAIL > ABSENT;
		return ret_val;								// should not see ABSENT here we just decided that the device exists

	tally_transaction (SUCCESS);
	return SUCCESS;
}

/**
	Invoke resetBus of whichever Wire net this class instance is using
	@return nothing
*/
void Systronix_PCA9557::reset_bus (void)
{
	_wire.resetBus();
}

/**
	Return the resetBusCount of whichever Wire net this class instance is using
	@return number of Wire net resets, clips at UINT32_MAX
*/
uint32_t Systronix_PCA9557::reset_bus_count_read(void)
{
	return _wire.resetBusCountRead();
}


//---------------------------< T A L L Y _ E R R O R S >------------------------------------------------------
/**
Here we tally errors.  This does not answer the what-to-do-in-the-event-of-these-errors question; it just
counts them.

TODO: we should decide if the correct thing to do when slave does not ack, or arbitration is lost, or
timeout occurs, or auto reset fails (states 2, 5 and 4, 7 ??? state numbers may have changed since this
comment originally added) is to declare these addresses as non-existent.

We need to decide what to do when those conditions occur if we do not declare the device non-existent.
When a device is declared non-existent, what do we do then? (this last is more a question for the
application than this library).  The questions in this TODO apply equally to other i2c libraries that tally
these errors.

Don't set error.exists = false here! These errors are likely recoverable. bab & wsk 170612

This is the only place we set error.error_val()

TODO use i2c_t3 error or status enumeration here in the switch/case
*/

void Systronix_PCA9557::tally_transaction (uint8_t value)
	{
	if (value && (error.total_error_count < UINT64_MAX))
		error.total_error_count++; 			// every time here incr total error count

	error.error_val = value;

	switch (value)
		{
		case SUCCESS:
			if (error.successful_count < UINT64_MAX)
				error.successful_count++;
			break;
		case 1:								// i2c_t3 and Wire: data too long from endTransmission() (rx/tx buffers are 259 bytes - slave addr + 2 cmd bytes + 256 data)
			error.data_len_error_count++;
			break;
#if defined I2C_T3_H
		case I2C_TIMEOUT:
			error.timeout_count++;			// 4 from i2c_t3; timeout from call to status() (read)
#else
		case 4:
			error.other_error_count++;		// i2c_t3 and Wire: from endTransmission() "other error"
#endif
			break;
		case 2:								// i2c_t3 and Wire: from endTransmission()
		case I2C_ADDR_NAK:					// 5 from i2c_t3
			error.rcv_addr_nack_count++;
			break;
		case 3:								// i2c_t3 and Wire: from endTransmission()
		case I2C_DATA_NAK:					// 6 from i2c_t3
			error.rcv_data_nack_count++;
			break;
		case I2C_ARB_LOST:					// 7 from i2c_t3; arbitration lost from call to status() (read)
			error.arbitration_lost_count++;
			break;
		case I2C_BUF_OVF:
			error.buffer_overflow_count++;
			break;
		case I2C_SLAVE_TX:
		case I2C_SLAVE_RX:
			error.other_error_count++;		// 9 & 10 from i2c_t3; these are not errors, I think
			break;
		case WR_INCOMPLETE:					// 11; Wire.write failed to write all of the data to tx_buffer
			error.incomplete_write_count++;
			break;
		case SILLY_PROGRAMMER:				// 12
			error.silly_programmer_error++;
			break;
		default:
			error.unknown_error_count++;
			break;
		}
	}


//---------------------------< C O N T R O L _ W R I T E >----------------------------------------------------
/*
	Write to the control register. This is the register accessed
	after the 9557 receives a valid slave address and ACKs it.
	The next data written by the master sets the 2 LSBs in the control register.
	Note that the control register itself does not have a "register address".
	It can also be read, according to NXP data sheet, but how?

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

	@param target_register the 9557 register which we want to access
	@return SUCCESS, FAIL, or ABSENT. 
	@side_effect if SUCCESS we update _control_reg
 */

uint8_t Systronix_PCA9557::control_write (uint8_t target_register)
	{
	uint8_t ret_val;

	if (!error.exists)						// exit immediately if device does not exist
		return ABSENT;

	if (target_register > PCA9557_CONFIG_REG)
		tally_transaction(SILLY_PROGRAMMER);			// upper bits not fatal. We think. TODO: test this in 9557 example code.

	_wire.beginTransmission (_base);
	ret_val = _wire.write (target_register);	// returns # of bytes written to i2c_t3 buffer
	if (1 != ret_val)
		{
		tally_transaction (WR_INCOMPLETE);			// only here we make 0 an error value
		return FAIL;
		}

	ret_val = _wire.endTransmission ();			// endTransmission() returns 0 if successful
  	if (SUCCESS != ret_val)
		{
		tally_transaction (ret_val);					// increment the appropriate counter
		return FAIL;							// calling function decides what to do with the error
		}

	_control_reg = target_register;				// remember where the control register is pointing

	tally_transaction (SUCCESS);
	return SUCCESS;
	}


//---------------------------< R E G I S T E R _ W R I T E >--------------------------------------------------
/**
	Write data to the register at location 'reg'
	This can be used as a general output byte write function to write to output register, assuming at least
	some of its bits are configured as output pins on the device.

	@param target_register
	@param data

	returns returns SUCCESS, FAIL, or ABSENT
	return of 0 == SUCCESS

 **/

uint8_t Systronix_PCA9557::register_write (uint8_t target_register, uint8_t data)
	{
	uint8_t ret_val;

	if (target_register > PCA9557_CONFIG_REG)
		tally_transaction(SILLY_PROGRAMMER);					// upper bits not fatal. We think. TODO: test this in 9557 example code.

	if (!error.exists)								// exit immediately if device does not exist
		return ABSENT;	

	_wire.beginTransmission (_base);
	ret_val = _wire.write (target_register);			// write control register; only 2 lsbs matter
	ret_val += _wire.write (data);						// and write the data to the tx_buffer
	if (2 != ret_val)
		{
		tally_transaction (WR_INCOMPLETE);					// only here 0 is error value since we expected to write more than 0 bytes
		return FAIL;
		}

	ret_val = _wire.endTransmission();
  	if (SUCCESS != ret_val)
		{
		tally_transaction (ret_val);							// increment the appropriate counter
		return FAIL;									// calling function decides what to do with the error
		}

	_control_reg = target_register;						// remember where the control register is pointing

	if (PCA9557_OUT_PORT_REG == target_register)		// update our remembered data value in the affected register
		_out_reg = data;								// these won't be updated if we already returned due to FAIL [bab]
	else if (PCA9557_INP_INVERT_REG == target_register)
		_invert_reg = data;
	else if (PCA9557_CONFIG_REG == target_register)
		_config_reg = data;

	tally_transaction (SUCCESS);
	return SUCCESS;
	}

//---------------------------< R E G I S T E R _ R E A D >------------------------------------------------------
/**
	Read a byte from target register

	local register and register contents tracking variables are updated in the functions called from here, so we don't
	need to do that here. So are any error values.

	@note after a register_read you can do additional default_reads at the same target_register. But at least
	check that _control_reg has not changed. No wait, that's not public.

	@param target_register
	@param data

	returns SUCCESS, FAIL, or ABSENT
	return of 0 == SUCCESS

*/

uint8_t Systronix_PCA9557::register_read (uint8_t target_register, uint8_t* data_ptr)
{

	if (target_register > PCA9557_CONFIG_REG)
		tally_transaction(SILLY_PROGRAMMER);						// upper bits not fatal. We think. TODO: test this in 9557 example code.

	if (!error.exists)						// exit immediately if device does not exist
		return ABSENT;

	if (control_write (target_register))		// put address of target_register into the control register
		return FAIL;
	
	if (default_read (data_ptr))				// read the target_register
		return FAIL;

	// the actual functions above update successful_count so don't duplicate that here
	return SUCCESS;
}	


//---------------------------< D E F A U L T _ R E A D >------------------------------------------------------
/**
Read a byte from whatever register is currently pointed to by the control register, the value of which is
held in our local private _control_reg

TODO have some way to verify that _control_reg is still what we think it is?

@param *data_ptr pointer to a uint8_t where the read data will be written

	returns SUCCESS, FAIL, or ABSENT
	return of 0 == SUCCESS
*/

uint8_t Systronix_PCA9557::default_read (uint8_t* data_ptr)
	{
	uint8_t ret_val;

	if (!error.exists)								// exit immediately if device does not exist
		return ABSENT;

	if (1 != _wire.requestFrom (_base, 1, I2C_STOP))
		{
		ret_val = _wire.status();						// to get error value
		tally_transaction (ret_val);							// increment the appropriate counter
		return FAIL;
		}

	*data_ptr = _wire.read();

	if (PCA9557_INP_PORT_REG == _control_reg)			// update our remembered reg value
		_inp_data = *data_ptr;
	else if (PCA9557_OUT_PORT_REG == _control_reg)
		_out_reg = *data_ptr;
	else if (PCA9557_INP_INVERT_REG == _control_reg)
		_invert_reg = *data_ptr;
	else if (PCA9557_CONFIG_REG == _control_reg)
		_config_reg = *data_ptr;

	tally_transaction (SUCCESS);
	return SUCCESS;
	}

 
 //--------------------------< P I N _ P U L S E >------------------------------------------------------------
 /**
 * Pulse pin(s) from idle state to active state and back to idle.
 * If idle is high then will pulse low; idle low will pulse high
 * Leave with pin(s) in idle state.
 * Example: 
 * pin_pulse (0xFF, true) will pulse ALL pins H-L-H; 
 * 0x02 will pulse output 1
 *
 * This only actually drives pins defined as outputs in the configuration register
 *  
 *  Note this only changes the state of the pins in pin_mask. Other outputs
 *  are tracked in a private variable and their state is unchanged.
 *
 * @TODO decide what to return and add support for it
 * maybe return boolean true if all bits driven are actually output bits?
 * AND pin_mask with ~config reg value, it should be equal to the pin mask
 * return TRUE if mask are outputs and no I2C errors

	@TODO I don't like this function [bab]
	if we get an error and return FAIL then we don't know the state of _out_reg for sure.
	So the application would need to consider that... how... and it can't "restore" _out_reg
	It could do a register_write to PCA9557_OUT_PORT_REG
 *
 * @param pin_mask [0..0xFF] the device output pin(s) you want to pulse
 * @param idle_high if true otherwise will idle low

 @return SUCCESS, FAIL, ABSENT. If FAIL then we don't know the state of our local _out_reg
 **/

uint8_t Systronix_PCA9557::pin_pulse (uint8_t pin_mask, boolean idle_high)
	{
	uint8_t	out_val = _out_reg;							// take a copy in case one or both writes fail
	
	if (!error.exists)								// exit immediately if device does not exist
		return ABSENT;

	if (idle_high)
		out_val &= ~(pin_mask);							// clear outputs to be pulsed low
	else
		out_val |= (pin_mask);							// set outputs to be pulsed	high	

	if (register_write (PCA9557_OUT_PORT_REG, out_val))	// pulse outputs to non-idle state
		return FAIL;

	if (idle_high)
		out_val |= (pin_mask);							// set outputs to be idled high
	else
		out_val &= ~(pin_mask);							// clr outputs to be idled low		
	
	if (register_write(PCA9557_OUT_PORT_REG, out_val))	// restore outputs to idle state
		return FAIL;
	
	return SUCCESS;
	}


//---------------------------< P I N _ D R I V E >------------------------------------------------------------
 /**
 * Drive pin(s) to a high or low voltage level
 * If boolean high is true they will be driven to a high level.
 * Leave with pin(s) in new state.
 * Example: 
 * pin_drive (0x02, true) 0x02 will drive output 1 to high level.
 * pin_drive (0xFF, true) will set pins IO7–IO1 high, IO0 is pulled up
 *
 * This only actually drives pins defined as outputs in the config register
 * 
 *  Note this only changes the state of the pins in pin_mask. Other outputs
 *  are tracked in a private variable and their state is unchanged.
 *
 * @See pin_pulse comments
 *
 * @param pin [0..0xFF] the device output pin(s) you want to drive to new state
 * @param high if true sets pin(s) high otherwise will drive to low level

  @return SUCCESS, FAIL, ABSENT. If FAIL then we don't know the state of our local _out_reg

 **/

uint8_t Systronix_PCA9557::pin_drive (uint8_t pin_mask, boolean high)
	{
	uint8_t		out_val = _out_reg;						// take a copy in case the write fails

	if (!error.exists)								// exit immediately if device does not exist
		return ABSENT;

	if (high)
		out_val |= (pin_mask);							// set outputs to be driven	high
	else
		out_val &= ~(pin_mask);							// clear outputs to be driven low

	if (register_write (PCA9557_OUT_PORT_REG, out_val))		// drive output(s) to new state
		return FAIL;

	return SUCCESS;
	}


//---------------------------< S E L F _ T E S T >------------------------------------------------------------
/**

TODO TODO TODO
This function fully self-tests the 9557:
1- write and read all 256 values of the output register internally
2- write all values to output pins and read the inputs to see if output pins on device really changed. Ignore any
bits which are '1' in the ignore_pins mask
3- write and read all values of invert register
3a- write output pin pattern and read it inverted or not to verify the inversion register actually works
4- write and read all values of the direction register

@param ignore_pins	ignore any output pins which are a '1' in this mask, which are presumably driven by an external force, so don't set them
to outputs, but make them inputs
@return SUCCESS or FAIL

*/

uint8_t Systronix_PCA9557::self_test(uint8_t ignore_pins)
{

	// decide if we need to update success counter here
	// if (error.successful_count < UINT64_MAX) error.successful_count++;
	return SUCCESS;		// for now until actually written
}


//---------------------------< P I N _ M O B I L I Y _ T E S T >----------------------------------------------
//
// A simple test to see that we can wiggle the PCA9557 port pins and that the pins aren't tied together or
// shorted to ground or a supply rail.
//


uint8_t Systronix_PCA9557::pin_mobility_test (uint8_t ignore_pins_mask)
	{
	int32_t write = 0xFEFF0100;									// walking 1 followed by walking zero (24 right shifts)
	uint8_t	write_val;
	uint8_t	read = 0;

	init (~ignore_pins_mask, 0, 0);								// set proper test configuration
	for (uint8_t i=0; i<25; i++, write >>=1)					// initial write plus 24 shifts leaves with all bits set
		{
		write_val = (uint8_t)(write | ignore_pins_mask);		// low 8 bits; force bit 4 to be set
		register_write (PCA9557_OUT_PORT_REG, write_val);		// write test value to the output register
		register_read(PCA9557_INP_PORT_REG, &read);				// read the 9557's i/o pins
		if ((read | ignore_pins_mask) != write_val)				// what we read should be the same as what we wrote
			{
			Serial.printf ("pin mobility test: expected: 0x%.2X; got: 0x%.2X\n", write_val, read);
			return FAIL;
			}
		}

	// the actual function(s) above update successful_count so don't duplicate that here
	return SUCCESS;
	}
