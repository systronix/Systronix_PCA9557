# Systronix_PCA9557
Arduino library for the I2C 8-bit I/O register PCA9557
Specifically we are using this to drive some legacy custom hardware, through multiple RS485 interfaces (one per signal).
## 
Note there is no 595 or 165 specific code in this library, it is just a generic PCA9557 library usable for any purpose.
## Use register_write() to write a byte to the output register. There is no "output_write" function.