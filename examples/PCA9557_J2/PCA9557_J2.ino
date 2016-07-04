
/** ---------- PCA9557_J2 ------------------------
 *  
 *  Controller must be Teensy 3.1 or 3.2
 *  Copyright 2016 Systronix Inc www.systronix.com
 *  
 *  This code uses the PCA9557 library directly
 *  
 *  The custom SALT2 board is used. It has four PCA9557 at the 
 *  addresses #define below.
 *  
 *  Other users won't likely have this board, but the code
 *  is still a useful example.
 *  
 */

 
/** ---------- REVISIONS ----------
 *  
 *  2016 Jun 28 bboyes, adding use of NAP_pod_load_defs.h for BIT0, etc.
 *  2016 May 26 bboyes moved into Systronix_PCA9557 examples folder
 *				Changed name to PCA9557_J2
 *  2016 May 15 bboyes Start with new SALT 2v0 board
 *	2016 Jul 4	wsk		complete thrash of this example used as a test bed for SALT_JX class
 */


#include <Arduino.h>
#include <Systronix_PCA9557.h>
#include <SALT_JX.h>
#include <NAP_pod_load_defs.h>

/**
 * These are the addresses of the I2C devices which drive the related
 * Core DB15 connector and the Core FETs
 * @TODO make these addresses manifest constants in some appropriate way; put them in a common .h file somewhere if they are needed in more than one file
 */
#define I2C_J2 0x1B
#define I2C_J3 0x1A
#define I2C_J4 0x19
#define I2C_FET 0x1C

// TODO: move to common .h file
#define JX_LED_ON	false		// yes, opposite to the other bits.  writing a 1 to the 9557 output register
#define JX_LED_OFF	true		// turns the output FET OFF


/**
 *  These are signal names specific to our custom SALT2 hardware; TODO: move to common .h file
 */

// SALT2 Core SDOUT on J2,3,4 pins 1/2 to '595 SDIN of DC and AC boards: 9557 IO7, 0x80 
// output from 9557 to '595 Din
#define SDOUT BIT7 	// 0x80

// SALT2 Core SCLK on J2,3,4 pins 5/6 to '595 and '165 SCLK of DC and AC boards: 9557 IO6, 0x40,
// serial clock, used by both 595 and 165
#define SCLK BIT6	// 0x40

// SALT2 J2,3,4 pins 9/10 to '165 Shift/Load of DC boards: Core SH/LD is 9557 IO5, 0x20
// SHIFT(H), LOAD(L) only used by DC board '165
#define SHIFT165 BIT5	// 0x20

// SALT2 Core SDIN on J2,3,4 pins 3/4 from '165 SDOUT of DC boards: 9557 IO4, 0x10 
// input to 9577 from '165 Qout
#define SDIN BIT4	// 0x10

// SALT2 RCLK on J2,3,4 pins 7/8 to DC and AC boards '595 RCLK: 9557 IO3, 0x08 
// only used by '595, must idle high
#define RCLK595 BIT3	// 0x08		

// bits 1 and 2 (0x02, 0x04) not connected

// SALT2 yellow LED for J2,3,4 intended to show activity on that port: 9557 IO0, 0x01 
// open drain, SETTING (yes, opposite to the other bits) this bit drives output low = LED on
#define LED BIT0	// 0x01

// coreFET defines: TODO: move to appropriate .h file
// LED on bit 0 uses the LED define above
#define	LIGHTS		BIT2		// when set HIGH, turns on the habitats' fluorescent lights
#define	ALARM		BIT3		// when set HIGH, sounds the alarm at the top of the A habitat
#define	FAN3		BIT5		// when set HIGH, turns on the left-most fan
#define	FAN2		BIT6		// when set HIGH, turns on the middle fan
#define	FAN1		BIT7		// when set HIGH, turns on the right-most fan


/**
 *  bit 5 is input; bits 2 and 1 are not connected, Bit 0 is the activity LED for that device
 *  so this mask has a '1' for every bit which is used as output
 *  Note: enclosing parentheses are required to force the '~' operator to operate on the whole quantity
 *  not just the first manifest constant! TODO: move to common .h file
 */
#define OUTMASK (RCLK595 | SCLK | SDOUT | SHIFT165 | LED)

/**
 *  Only one actual input, serial data in; TODO: move to common .h file
 */
#define INMASK (SDIN)

SALT_JX coreJ2 (2);
//SALT_JX coreJ3 (3);
//SALT_JX coreJ4 (4);

Systronix_PCA9557 coreJ2reg(I2C_J2);
Systronix_PCA9557 coreJ3reg(I2C_J3);
Systronix_PCA9557 coreJ4reg(I2C_J4);

Systronix_PCA9557 coreFET(I2C_FET);

// TODO: move to common .h file as #defines;
uint8_t periph_rst = 22;	// peripheral reset asserted LOW to PCA9557, etc on board
uint8_t ether_rst = 8;		// Wiznet module reset LOW

struct J_data
	{
	uint32_t	DC_data_in;			// data read from the DC board '165 shift register
	uint16_t	AC_data_out;		// data sent to the AC board; send this before sending DC
	uint16_t	DC_data_out;		// data sent to the DC board
	} J2_data , J3_data, J4_data;

	
//---------------------------< S E T U P >--------------------------------------------------------------------

void setup(void) 
	{
  	pinMode(periph_rst, OUTPUT);     
	pinMode (ether_rst, OUTPUT);
	
	digitalWrite(periph_rst, LOW);		// resets asserted
	digitalWrite(ether_rst, LOW);	
	
	delay (2000);      // give some time to open monitor window
	Serial.begin(115200);     // use max baud rate
  
	// Teensy3 doesn't reset with Serial Monitor as do Teensy2/++2, or wait for Serial Monitor window
	// Wait here for 10 seconds to see if we will use Serial Monitor, so output is not lost
	while((!Serial) && (millis()<10000));    // wait until serial monitor is open or timeout
	
	digitalWrite(periph_rst, HIGH);		// resets not asserted
	digitalWrite(ether_rst, HIGH);

	Serial.print("SALT2 J2 I2C Register Test Code at 0x");
	Serial.println(coreJ2reg.BaseAddr, HEX);

	/**
	* Start, now inits invert and output bit enable, clears outputs
	*/
	coreJ2reg.begin();
	coreJ3reg.begin();
	coreJ4reg.begin();
	coreFET.begin();
	
	// outmask, outdata, inputinvert
	coreJ2reg.init(OUTMASK, (SCLK | RCLK595 | LED), 0);
	coreJ3reg.init(OUTMASK, (SCLK | RCLK595 | LED), 0);
	coreJ4reg.init(OUTMASK, (SCLK | RCLK595 | LED), 0);
	// FETs don't have clocks to downstream devices
	// set all as outputs, all outputs off/low, no input inversion (none are inputs anyway)
	coreFET.init(0xFF, 0x00, 0x00);

	coreJ2.AC_data_out = 0;				// clear DC and AC 595 registers
	coreJ2.DC_data_out = 0;
//	coreJ3.AC_data_out = 0;				// clear DC and AC 595 registers
//	coreJ3.DC_data_out = 0;
//	coreJ4.AC_data_out = 0;				// clear DC and AC 595 registers
//	coreJ4.DC_data_out = 0;
	coreJ2.update ();					// do it
//	coreJ3.update (coreJ3reg);			// do it
//	coreJ3.update (coreJ4reg);			// do it
	coreJ2.AC_data_out = 0x8000;	// setup for walking-ones test
//	coreJ3.AC_data_out = 0x8000;	// setup for walking-ones test
//	coreJ3.AC_data_out = 0x8000;	// setup for walking-ones test
	}

	uint8_t fet_test = 0x80;


//---------------------------< L O O P >----------------------------------------------------------------------

void loop(void) 
	{
	if (0x80 & fet_test)
		fet_test = 4;								// reset to bit 2, fluorescents
	else
		fet_test <<= 1;

	coreFET.pin_drive ((fet_test | LED), HIGH);		// HIGH turn the LED off and the FET on
	delay(1000);
	
	coreFET.pin_drive ((fet_test | LED), LOW);		// LOW turn the LED on and the FET off
//	coreJ2reg.pin_drive (LED, true);
	// coreFET.pin_drive (LED, true);
	
	if (0x8000 & coreJ2.AC_data_out)			// when the '1' is in the MSB (bit 31)
		{
		coreJ2.AC_data_out = 0;				// clear so there is only one '1'
		coreJ2.DC_data_out = 1;				// shift the '1' to lsb of DC data
//		coreJ3.AC_data_out = 0;				// clear so there is only one '1'
//		coreJ3.DC_data_out = 1;				// shift the '1' to lsb of DC data
//		coreJ4.AC_data_out = 0;				// clear so there is only one '1'
//		coreJ4.DC_data_out = 1;				// shift the '1' to lsb of DC data
		}
	else if (0x8000 & coreJ2.DC_data_out)		// when the '1' is in the DC board's MSB (bit 15)
		{
		coreJ2.AC_data_out = 1;				// shift the '1' to lsb of DC data
		coreJ2.DC_data_out = 0;				// clear so there is only one '1'
//		coreJ3.AC_data_out = 1;				// shift the '1' to lsb of DC data
//		coreJ3.DC_data_out = 0;				// clear so there is only one '1'
//		coreJ4.AC_data_out = 1;				// shift the '1' to lsb of DC data
//		coreJ4.DC_data_out = 0;				// clear so there is only one '1'
		}
	else
		{
		coreJ2.AC_data_out <<= 1;			// shift both registers toward msb
		coreJ2.DC_data_out <<= 1;
//		coreJ3.AC_data_out <<= 1;			// shift both registers toward msb
//		coreJ3.DC_data_out <<= 1;
//		coreJ4.AC_data_out <<= 1;			// shift both registers toward msb
//		coreJ4.DC_data_out <<= 1;
		}


	coreJ2.update ();						// do it
//	coreJ3.update (coreJ3reg);				// do it
//	coreJ4.update (coreJ4reg);				// do it

/*	if (0x8000 & J2_data.AC_data_out)		// when the '1' is in the MSB (bit 31)
		{
		J2_data.AC_data_out = 0;
		J2_data.DC_data_out = 1;
		}
	else if (0x8000 & J2_data.DC_data_out)	// when the '1' is in the DC board's MSB (bit 15)
		{
		J2_data.AC_data_out = 1;			// shift the '1' to lsb of DC data
		J2_data.DC_data_out = 0;			// clear so there is only one '1'
		}
	else
		{
		J2_data.AC_data_out <<= 1;			// shift both registers toward msb
		J2_data.DC_data_out <<= 1;
		}
	coreJ2.update ();
*/
	Serial.print("------INPUT------0x");
	Serial.println(coreJ2.DC_data_in, HEX);

	delay(1000);
	}