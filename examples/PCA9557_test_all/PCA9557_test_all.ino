
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
 */


#include <Arduino.h>
#include <Systronix_PCA9557.h>
#include <NAP_pod_load_defs.h>

 /**
 * debug level
 * 0 = quiet, suppress everything
 * 3 = max, even more or less trivial message are emitted
 * 4 = emit debug info which checks very basic data conversion, etc
 */
 byte DEBUG = 1;


uint16_t dtime = 1000;  // delay in loop

/**
 * These are the addresses of the I2C devices which drive the related
 * Core DB15 connector and the Core FETs
 * @TODO make these addresses manifest constants in some appropriate way
 */
#define I2C_J2 0x1B
#define I2C_J3 0x1A
#define I2C_J4 0x19
#define I2C_FET 0x1C


/**
 *  These are signal names specific to our custom SALT2 hardware
 */

// SALT2 Core SDOUT on J2,3,4 pins 1/2 to '595 SDIN of DC and AC boards: 9557 IO7, 0x80 
// output from 9577 to '595 Din
//#define SDOUT BIT7 	// 0x80
#define SDOUT 0x80

// SALT2 Core SCLK on J2,3,4 pins 5/6 to '595 and '165 SCLK of DC and AC boards: 9557 IO6, 0x40,
// serial clock, used by both 595 and 165
//#define SCLK BIT6	// 0x40
#define SCLK 0x40

// SALT2 J2,3,4 pins 9/10 to '165 Shift/Load of DC boards: Core SH/LD is 9557 IO5, 0x20
// SHIFT(H), LOAD(L) only used by DC board '165
//#define SHIFT165 BIT5	// 0x20
#define SHIFT165 0x20

// SALT2 Core SDIN on J2,3,4 pins 3/4 from '165 SDOUT of DC boards: 9557 IO4, 0x10 
// input to 9577 from '165 Qout
//#define SDIN BIT4	// 0x10
#define SDIN 0x10

// SALT2 RCLK on J2,3,4 pins 7/8 to DC and AC boards '595 RCLK: 9557 IO3, 0x08 
// only used by '595, must idle high
//#define RCLK595 BIT3	// 0x08		
#define RCLK595 0x08		

// bits 1 and 2 (0x02, 0x04) not connected

// SALT2 yellow LED for J2,3,4 intended to show activity on that port: 9557 IO0, 0x01 
// open drain, clearing this bit drives output low = LED on
//#define LED BIT0	// 0x01
#define LED 0x01

/**
 *  bit 5 is input; bits 2 and 1 are not connected, Bit 0 is the activity LED for that device
 *  so this mask has a '1' for every bit which is used as output
 *  Note: enclosing parentheses are required to force the '~' operator to operate on the whole quantity
 *  not just the first manifest constant!
 */
#define OUTMASK (RCLK595 | SCLK | SDOUT | SHIFT165 | LED)

/**
 *  Only one actual input, serial data in
 */
#define INMASK (SDIN)

// Now these are just declared but no class has been instantiated yet
Systronix_PCA9557 coreJ2;
Systronix_PCA9557 coreJ3;
Systronix_PCA9557 coreJ4;

Systronix_PCA9557 coreFET;

uint8_t periph_rst = 22;	// peripheral reset asserted LOW to PCA9557, etc on board
uint8_t ether_rst = 8;		// Wiznet module reset LOW

/* ========== SETUP ========== */
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
	
	// now get an instance at the specific I2C address
	coreJ2.setup(I2C_J2);
	coreJ3.setup(I2C_J3);
	coreJ4.setup(I2C_J4);
	coreFET.setup(I2C_FET);

	Serial.print("SALT2 J2 I2C Register Test Code at 0x");
	Serial.println(coreJ2.BaseAddr, HEX);
	
	Serial.print("SALT2 J3 I2C Register Test Code at 0x");
	Serial.println(coreJ3.BaseAddr, HEX);

	/**
	 *  Serial1 is a proprietary LCD and cap touch keypad
	 */
	Serial1.begin(9600);			// UI Side 0 = LCD and keypad
	while((!Serial1) && (millis()<10000));		// wait until serial port is open or timeout
	Serial.print("Serial1 ready at ");
	Serial.println(millis());
	Serial1.println("d");	// clear lcd
	Serial1.flush();
	delay(20);	// for LCD, 5 msec not always enough
	
	Serial2.begin(9600);			// UI Side 0 = LCD and keypad
	while((!Serial2) && (millis()<10000));		// wait until serial port is open or timeout
	Serial.print("Serial2 ready at ");
	Serial.println(millis());
	Serial2.println("d");	// clear lcd
	Serial2.flush();
	delay(20);	// for LCD, 5 msec not always enough
	
	// delay(1000);

	              // 0123456789ABCDEF 
	Serial1.println("dSALT J2 I2C TestSerial 1");
	Serial1.flush();
	
		              // 0123456789ABCDEF 
	Serial2.println("dSALT J2 I2C TestSerial 2");
	Serial2.flush();

	Serial.println("Ready to instantiate coreJ2,3,4");
	
	/**
	* Start, now inits invert and output bit enable, clears outputs
	*/
	coreJ2.begin();
	coreJ3.begin();
	coreJ4.begin();
	coreFET.begin();
	
	Serial.println("Ready to init coreJ2,3,4");
	
	uint8_t init_flag=0;
	// outmask, outdata, inputinvert
	init_flag += coreJ2.init(OUTMASK, (SCLK | RCLK595 | LED), 0);
	init_flag += coreJ3.init(OUTMASK, (SCLK | RCLK595 | LED), 0);
	init_flag += coreJ4.init(OUTMASK, (SCLK | RCLK595 | LED), 0);
	// FETs don't have clocks to downstream devices
	// set all as outputs, all outputs off/low, no input inversion (none are inputs anyway)
	init_flag += coreFET.init(0xFF, 0x00, 0x00);
	
	// flag should be 0 if no errors
	if (init_flag) 
	{
		Serial.print("Init error on core I2C devices: 0x");
		Serial.println(init_flag, HEX);
	}

	Serial.print(" Interval is ");
	Serial.print(dtime/1000);
	Serial.print(" sec, ");

	Serial.println("Setup Complete!");
	Serial.println(" "); 
  
}



/* ========== LOOP ========== */
void loop(void) 
{
	uint8_t read1=0;

	Serial.print("@");
	Serial.print(millis()/1000);
	Serial.println(" ");
	
	for (uint8_t i = 0; i<4; i++) {
		coreJ2.control_write(i);
		Serial.print("At reg 0x");
		Serial.print(i, HEX);
		read1 = coreJ2.default_read();
		Serial.print(" rd 0x");
		Serial.println(read1, HEX);
	}

	Serial.print("OUTMASK=0x");
	Serial.println(OUTMASK, HEX);
	
	read1 = coreJ2.output_read();
	Serial.print("Output read=0x");
	Serial.println(read1, HEX);
	
	read1 = coreJ2.input_read();
	Serial.print("Input=0x");
	Serial.println(read1, HEX);
	
	coreJ2.pin_drive (LED, false);
	coreJ3.pin_drive (LED, false);
	coreJ4.pin_drive (LED, false);
	// coreFET.pin_drive (LED, false);
	coreFET.pin_drive ((0x80 | LED), false);
	delay(1000);
	
	coreFET.pin_drive ((0x80 | LED), true);
	coreJ2.pin_drive (LED, true);
	coreJ3.pin_drive (LED, true);
	coreJ4.pin_drive (LED, true);
	// coreFET.pin_drive (LED, true);
	delay(1000);
	
	Serial1.print("d");
	Serial1.println(millis());
	
	Serial2.print("d");
	Serial2.println(millis());
	
	
}