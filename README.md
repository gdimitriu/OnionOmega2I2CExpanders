# OnionOmega2I2CExpanders

## Environment to compile
 
Compiler in makefiles is set to the cross-compiler.

To compile the sources on cross-compiler or direct on device:

- dependencies

	- two environment variable with full path should be exported (a settings.sh is provided with my defaults, those should be replaced) 
	
	- i2c-exp-drivers obtained from omega github https://github.com/OnionIoT/i2c-exp-driver as PREFIX_I2C_EXP_DRIVER
	
	- fast-gpio obtained from omega github https://github.com/OnionIoT/fast-gpio as PREFIX_FAST_GPIO
	
- in the eclipse settings (located in settings/onion.include.xml) replace the path to the correct location of crosscompiler and libraries

## DRIVERS

- [Adafruit PCA9685 PWM Servo Driver](https://www.adafruit.com/product/815)
 
- [Sparkfun SX1509 expander](https://www.sparkfun.com/products/13601)

 - PCF8574 IO expander
 
 - [PCF8591 ADC expander](https://brainfyre.wordpress.com/2012/10/25/pcf8591-yl-40-ad-da-module-review/)
 