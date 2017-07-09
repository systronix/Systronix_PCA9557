# Systronix_PCA9557
Arduino library for the I2C 8-bit I/O register PCA9557
Specifically we are using this to drive some legacy custom hardware, through multiple RS485 interfaces (one per signal). But that's really not relevant to this library; that is just where the outputs and inputs of the 9557 go.
## Branches
i2c_t3_by_ref will be merged into master really soon
### i2c_t3_by_ref
- The 'working but still not merged back into the master branch' [i2c_t3_by_ref](https://github.com/systronix/Systronix_PCA9557/tree/i2c_t3_by_ref) assumes use of the fabulous [i2c_t3](https://github.com/systronix/i2c_t3) library by [nox771](https://github.com/nox771), with minor mods by systronix.
- This branch supports using this library on any i2c_t3 Wire net, through a specific reference to the i2c_t3 structure
### master
old, we are no longer using this.

## Examples
### PCA9557_test_all
Won't work with i2c_t3_by_ref branchs. Will fix soon.
### SALT_Power_FRU
This has up to date examples using the i2c_t3 reference on Wire or Wire1 

## Notes
- This is a generic PCA9557 library usable for any purpose.
- Use register_write() to write a byte to the output register. There is no "output_write" function.
- We try to use the "noun-verb" format for functions to make it easier to search for related things in the code by searching on the noun. So it is "register_write" vs "write_register".

## TODO
- Merge setup and begin? They seem redundant and sort of clashing.
This readme could be a lot better
- bring all the code comments into [doxygen](http://www.stack.nl/~dimitri/doxygen/) compliance and generate the html docs