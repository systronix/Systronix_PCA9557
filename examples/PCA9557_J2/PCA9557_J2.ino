
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
 *	2016 Jul 4	wsk		complete thrash of this example used as a test bed for SALT_JX and SALT_FETs classes
 */


#include <Arduino.h>
#include <Systronix_PCA9557.h>
#include <SALT_JX.h>
#include <SALT_FETs.h>
#include <NAP_pod_load_defs.h>

#ifdef USE_CLASS
SALT_JX coreJ2 (2);
//SALT_JX coreJ3 (3);
//SALT_JX coreJ4 (4);
#else
extern void J2_update (void);
extern void J3_update (void);
extern void J4_update (void);
extern void JX_init (void);
extern struct J_data J2_data, J3_data, J4_data;
#endif

SALT_FETs coreFETs (1);

Systronix_PCA9557 coreJ2reg(I2C_J2);
Systronix_PCA9557 coreJ3reg(I2C_J3);
Systronix_PCA9557 coreJ4reg(I2C_J4);

// TODO: move to common .h file as #defines;
uint8_t periph_rst = 22;	// peripheral reset asserted LOW to PCA9557, etc on board
uint8_t ether_rst = 8;		// Wiznet module reset LOW

	
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
	coreJ2reg.begin();	// join I2C as master
	coreJ3reg.begin();
	coreJ4reg.begin();
	
	// outmask, outdata, inputinvert
//	coreJ2reg.init(OUTMASK, LED, 0);	// all outputs low, LED off
//	coreJ3reg.init(OUTMASK, LED, 0);
//	coreJ4reg.init(OUTMASK, LED, 0);
	JX_init();			// initialize the three JX 9557 registers
	coreFETs.init ();

#ifdef USE_CLASS
	coreJ2.AC_data_out = 0;				// clear DC and AC 595 registers
	coreJ2.DC_data_out = 0;
//	coreJ3.AC_data_out = 0;				// clear DC and AC 595 registers
//	coreJ3.DC_data_out = 0;
//	coreJ4.AC_data_out = 0;				// clear DC and AC 595 registers
//	coreJ4.DC_data_out = 0;
	coreJ2.update ();					// do it
//	coreJ3.update (coreJ3reg);			// do it
//	coreJ3.update (coreJ4reg);			// do it
	coreJ2.AC_data_out = 0x8000;		// setup for walking-ones test
//	coreJ3.AC_data_out = 0x8000;		// setup for walking-ones test
//	coreJ3.AC_data_out = 0x8000;		// setup for walking-ones test
#else
	J2_data.outdata.as_word = 0;		// clear DC and AC 595 registers
	J3_data.outdata.as_word = 0;
	J4_data.outdata.as_word = 0;

	J2_update ();						// do it
	J3_update ();						// do it
	J4_update ();						// do it

	J2_data.outdata.as_word = 0x80000000;	// setup for walking-one test
	J3_data.outdata.as_word = 0x80000000;
	J4_data.outdata.as_word = 0x80000000;

#endif
	coreFETs.FET_settings = 0x80;		// init for walking-one test
	}


//---------------------------< L O O P >----------------------------------------------------------------------

void loop(void) 
	{
	if (0x80 & coreFETs.FET_settings)
		coreFETs.FET_settings = 4;								// reset to bit 2, fluorescents
	else
		coreFETs.FET_settings <<= 1;

	coreFETs.update();		// update the fets
	delay(20);
	
	coreFETs.update();		// update the fets
	delay(1000);

	Serial.print("------INPUT------0x");

#ifdef USE_CLASS
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
	Serial.println(coreJ2.DC_data_in, HEX);

#else
	if (0x80000000 & J2_data.outdata.as_word)		// when the '1' is in the MSB (bit 31) (AC data)
		{
		J2_data.outdata.as_word = 1;		// shift the '1' to lsb of DC data
		J3_data.outdata.as_word = 1;
		J4_data.outdata.as_word = 1;
		}
	else
		{
		J2_data.outdata.as_word <<= 1;		// shift both registers toward msb
		J3_data.outdata.as_word <<= 1;
		J4_data.outdata.as_word <<= 1;
		}

	J2_update ();							// do it
	J3_update ();							// do it
	J4_update ();							// do it
	Serial.println(J4_data.DC_data_in, HEX);
#endif

	delay(1000);
	}