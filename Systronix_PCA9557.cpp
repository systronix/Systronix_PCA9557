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
	@todo	Test base address for legal range 0x18..0x1F

	@param wire instance of i2c_t3 'object' Wire, Wire1, Wire2, Wire3
	@param name is a string such as Wire, Wire1, etc used for debug output, 
		also saved as wire_name
*/
/**************************************************************************/
void Systronix_PCA9557::setup(uint8_t base, i2c_t3 wire, char* name)
	{
	_base = base;
	_wire = wire;
	_wire_name = wire_name = name;		// protected and public
	}

/**
	Default setup, where itc_t3 &wire is Wire and the name is "Wire"
	This makes the library backwards compatible with older applications.
*/
void Systronix_PCA9557::setup(uint8_t base)
	{
	_base = base;
	_wire = Wire;
	_wire_name = wire_name = (char*) "Wire";		// protected and public
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
 *  @param config_set set bits will be outputs. 0xFF makes all pins outputs
 * 	Note: the 9557 config reg (which should have been called 'direction reg')
 *  uses a 1 to indicates an input, 0 an output. We think that's backwards
 *  so this param gets inverted in the init routine so that 1 bits = outputs.
 *  This means if you have an output mask of 1's such as 0x0F, you can write
 *  That mask to the config reg and 0x0F will be outputs.
 *  @param out_reg value to write to output pins, writing a 1 drives outputs high
 *  On IO0, open drain, a '1' allows IO0 to be pulled up; a '0' is drive low
 *  @param invert_reg applies to pins set as inputs. Setting a mask bit inverts that
 *  input when it is read. After POR this reg is 0xF0
 *  @return 0 if OK, and an error code if not
 *  
 *  @TODO maybe take out the inversion of config? To be consistent with the data sheet
 */ 
uint8_t Systronix_PCA9557::init(uint8_t config_reg, uint8_t out_reg, uint8_t invert_reg)
	{
	uint8_t ret_val = SUCCESS;
	uint8_t ret_cnt = 0;

	Serial.printf("Lib init %s at base 0x%.2X\r\n", _wire_name, _base);
	
	_wire.beginTransmission (_base);				// see if the device is communicating by writing to control register	:: this is a write to the library [wsk]
	ret_cnt += _wire.write (PCA9557_OUT_PORT_REG);	// write returns # of bytes written to local buffer					:: also a write to the library [wsk]
	// Serial.printf("Lib init wrote %u bytes to i2c buffer\r\n", ret_cnt);
	ret_val = _wire.endTransmission();
	if (ret_val)							// returns 0 if no error		:: library writes to the device [wsk]
		{
		Serial.printf("Lib init endTrans failed with 0x%.2X\r\n", ret_val);
		control.exists = false;
		return FAIL;
		}
	
	control.exists = true;

	ret_val |= register_write(PCA9557_OUT_PORT_REG, out_reg);			// init output reg first so that it is i correct state when config reg written
	ret_val |= register_write(PCA9557_CONFIG_REG, ~config_reg);			// clear pin dir reg bits to 0 for all outputs	
	ret_val |= register_write(PCA9557_INP_INVERT_REG, invert_reg);		// 1 = input read bits inverted, 0 = not inverted

	if (ret_val)											// if anything other than SUCCESS
		return FAIL;										// TODO: should return ret_val which can be FAIL or ABSENT
	
	return SUCCESS;											// 
	}

//---------------------------< T A L L Y _ E R R O R S >------------------------------------------------------
//
// Here we tally errors.  This does not answer the what-to-do-in-the-event-of-these-errors question; it just
// counts them.  If the device does not ack the address portion of a transaction or if we get a timeout error,
// exists is set to false.  We assume here that the timeout error is really an indication that the automatic
// reset feature of the i2c_t3 library failed to reset the device in which case, the device no longer 'exists'
// for whatever reason.
//
// TODO: we should decide if the correct thing to do when slave does not ack, or arbitration is lost, or
// timeout occurs, or auto reset fails (states 2, 5 and 4, 7) is to declare these addresses as non-existent.
// We need to decide what to do when those conditions occur if we do not declare the device non-existent.
// When a device is declared non-existent, what do we do then? (this last is more a question for the
// application than this library).  The questions in this TODO apply equally to other i2c libraries that tally
// these errors
//
// Don't set control.exists = false here! These errors are likely recoverable. bab & wsk 170612
//

void Systronix_PCA9557::tally_errors (uint8_t value)
	{
	
	if (error.total_error_count < UINT64_MAX) error.total_error_count++; 	// every time here incr total error count
		switch (value)
		{
		case 0:					// Wire.write failed to write all of the data to tx_buffer
			error.incomplete_write_count++;
			break;
		case 1:					// data too long from endTransmission() (rx/tx buffers are 259 bytes - slave addr + 2 cmd bytes + 256 data)
			error.data_len_error_count++;
			break;
		case 4:					 
#if defined I2C_T3_H		
			error.timeout_count++;			// error 4 = i2c_t3 timeout
#else
			error.other_error_count++;		// error 4 = Wire "other error"
#endif
			break;
		case 2:
		case 5:
			error.rcv_addr_nack_count++;
			break;
		case 3:
		case 6:
			error.rcv_data_nack_count++;
			break;
		case 7:					// arbitration lost from call to status() (read)
			error.arbitration_lost_count++;
			break;
		case 8:
			error.buffer_overflow_count++;
			break;
		case 9:
		case 10:
			error.other_error_count++;		// i2c_t3 these are not errors, I think
			break;
		case 11:
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
	It can also be written, but how?											:: did you mean 'read'? [wsk]
	
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

	returns # of bytes written which should be 1, 2 if error?					:: does not; returns SUCCESS, FAIL, or ABSENT [wsk]
 */

uint8_t Systronix_PCA9557::control_write (uint8_t data)
	{
	if (!control.exists)								// exit immediately if device does not exist
		return ABSENT;

	_wire.beginTransmission (_base);
	error.error_val = _wire.write (data);				// returns # of bytes written to i2c_t3 buffer
	if (1 != error.error_val)
		{
		error.error_val = 0;
		tally_errors (error.error_val);
		return FAIL;
		}

	error.error_val = _wire.endTransmission ();
  	if (SUCCESS != error.error_val)
		{
		tally_errors (error.error_val);					// increment the appropriate counter
		return FAIL;									// calling function decides what to do with the error
		}

	_control_reg = data;								// remember where the control register is pointing
	return SUCCESS;
	}


//---------------------------< R E G I S T E R _ W R I T E >--------------------------------------------------
/**
 * Write data to the register at location 'reg'
 *
 * returns # of bytes written which should be 2, 3 if error?					:: does not; returns SUCCESS, FAIL, or ABSENT [wsk]
  return of 0 == SUCCESS
 *
 * This can be used as a general output byte write function
 *
 * @TODO check reg for valid range
 **/

 uint8_t Systronix_PCA9557::register_write (uint8_t reg, uint8_t data)
	{
	if (!control.exists)								// exit immediately if device does not exist
		return ABSENT;

	_wire.beginTransmission (_base);
	error.error_val = _wire.write (reg);				// write control register; only 2 lsbs matter
	error.error_val += _wire.write (data);				// and write the data to the tx_buffer
	if (2 != error.error_val)
		{
		error.error_val = 0;							// what? now a value of 0 is an error, not SUCCESS?
		tally_errors (error.error_val);					// why can't we be consistent on what return==0 means? [bab]
		return FAIL;
		}

	error.error_val = _wire.endTransmission();
  	if (SUCCESS != error.error_val)
		{
		tally_errors (error.error_val);					// increment the appropriate counter
		return FAIL;									// calling function decides what to do with the error
		}

	_control_reg = reg;									// remember where the control register is pointing
	if (PCA9557_OUT_PORT_REG == reg)					// update our remembered reg values
		_out_reg = data;								// these won't be updated if we already returned due to FAIL [bab]
	if (PCA9557_INP_INVERT_REG == reg)
		_invert_reg = data;
	if (PCA9557_CONFIG_REG == reg)
		_config_reg = data;

	return SUCCESS;
	}


//---------------------------< D E F A U L T _ R E A D >------------------------------------------------------
/**
 * Read a byte from whatever register is currently pointed to by the control register
 *
 * Prints an error message if there was not a byte ready from the I2C device
 */
//
// THIS FUNCTION DEPRECATED - except by printing to the serial monitor there is no good way of notifying the
// calling function that the read was or was not successful.
//
// TODO: deprecated since 22 August 2016; delete now?
//

uint8_t Systronix_PCA9557::default_read (void)
	{
	uint8_t recvd = 0xFF;	// init with error value; 0xFF is not an illegal value

	if (!control.exists)								// exit immediately if device does not exist
		return ABSENT;

	error.error_val = _wire.requestFrom(_base, 1, true);

	if (1 != error.error_val)
		{
		error.error_val = _wire.status();				// to get error value
		tally_errors (error.error_val);					// increment the appropriate counter

		Serial.print(" Error I2C read didn't return 1 but ");
		Serial.println (error.error_val);
		}

	recvd = _wire.read();	// if there was an error this assignment is undefined

	if (PCA9557_INP_PORT_REG == _control_reg)			// update our remembered reg value
		_inp_data = recvd;
	if (PCA9557_OUT_PORT_REG == _control_reg)
		_out_reg = recvd;
	if (PCA9557_INP_INVERT_REG == _control_reg)
		_invert_reg = recvd;
	if (PCA9557_CONFIG_REG == _control_reg)
		_config_reg = recvd;

	return recvd;
	}


//---------------------------< D E F A U L T _ R E A D >------------------------------------------------------
//
// Read a byte from whatever register is currently pointed to by the control register
//

uint8_t Systronix_PCA9557::default_read (uint8_t* data_ptr)
	{
	if (!control.exists)								// exit immediately if device does not exist
		return ABSENT;

	if (1 != _wire.requestFrom (_base, 1, I2C_STOP))
		{
		error.error_val = _wire.status();				// to get error value
		tally_errors (error.error_val);					// increment the appropriate counter
		return FAIL;
		}

	*data_ptr = _wire.read();

	if (PCA9557_INP_PORT_REG == _control_reg)			// update our remembered reg value
		_inp_data = *data_ptr;
	if (PCA9557_OUT_PORT_REG == _control_reg)
		_out_reg = *data_ptr;
	if (PCA9557_INP_INVERT_REG == _control_reg)
		_invert_reg = *data_ptr;
	if (PCA9557_CONFIG_REG == _control_reg)
		_config_reg = *data_ptr;

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
 *
 * @param pin [0..0xFF] the device output pin(s) you want to pulse
 * @param idle_high if true otherwise will idle low
 **/

uint8_t Systronix_PCA9557::pin_pulse (uint8_t pin_mask, boolean idle_high)
	{
	uint8_t		out_val = _out_reg;						// take a copy in case one or both writes fail
	
	if (!control.exists)								// exit immediately if device does not exist
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
 * pin_drive (0xFF, true) will set pins IO7–IO1 high, IO0 will drive active low		:: IO0, pulled up, not driven low [wsk]
 * NOTE: IO0 output IS opposite to all other outputs								:: no [wsk]
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
	uint8_t		out_val = _out_reg;						// take a copy in case the write fails

	if (!control.exists)								// exit immediately if device does not exist
		return ABSENT;

	if (high)
		out_val |= (pin_mask);							// set outputs to be driven	high
	else
		out_val &= ~(pin_mask);							// clear outputs to be driven low

	if (register_write (PCA9557_OUT_PORT_REG, out_val))		// drive output(s) to new state
		return FAIL;
	return SUCCESS;
	}


//---------------------------< I N P U T _ R E A D >----------------------------------------------------------
/**
 *  Read the input register 0x00 and return its value
 *  
 *  TODO optionally filter off output bits?
 *  
 */
//
// THIS FUNCTION DEPRECATED - this function has no good way of notifying the calling function that the read
// was or was not successful.
//
// TODO: deprecated since 22 August 2016; delete now?
//

uint8_t Systronix_PCA9557::input_read ()
	{
	uint8_t data_read;
	if (!control.exists)								// exit immediately if device does not exist
		return ABSENT;

	control_write (PCA9557_INP_PORT_REG);
	data_read = default_read();
	return data_read;
	}


//---------------------------< I N P U T _ R E A D >----------------------------------------------------------
//
// Read the input register 0x00 and return its value
// TODO: optionally filter off output bits?
//

uint8_t Systronix_PCA9557::input_read (uint8_t* data_ptr)
	{
	if (!control.exists)								// exit immediately if device does not exist
		return ABSENT;

	if (control_write (PCA9557_INP_PORT_REG))
		return FAIL;
	
	if (default_read (data_ptr))
		return FAIL;
	return SUCCESS;
	}


//---------------------------< O U T P U T _ R E A D >--------------------------------------------------------
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
//
// THIS FUNCTION DEPRECATED - this function has no good way of notifying the calling function that the read
// was or was not successful.
//
// TODO: deprecated since 22 August 2016; delete now?
//

uint8_t Systronix_PCA9557::output_read ()
	{
	uint8_t data_read;
	if (!control.exists)								// exit immediately if device does not exist
		return ABSENT;

	control_write (PCA9557_OUT_PORT_REG);
	data_read = default_read();
	return data_read;
	}


//---------------------------< O U T P U T _ R E A D >--------------------------------------------------------
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

uint8_t Systronix_PCA9557::output_read (uint8_t* data_ptr)
	{
	if (!control.exists)								// exit immediately if device does not exist
		return ABSENT;

	if (control_write (PCA9557_OUT_PORT_REG))
		return FAIL;

	if (default_read(data_ptr))
		return FAIL;
	return SUCCESS;
	}
