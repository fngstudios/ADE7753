# ADE7753

## Work in progress, needs cleanup and testing on other platforms.
## Now working with esp8266 all types changed since esp is 32 bits.
### WARNING! If using this library with esp8266 change SPI mode to 3 since arduino core for esp8266 has an error in SPI.cpp or do this:https://github.com/esp8266/Arduino/issues/2416

Interfaces with ADE7753 single phase energy meter IC.

###Reads:

*Voltage  
*Current  
*Frequency  
*Energy  
*Real power  
*Reactive power  
*Apparent power  
*IC temperature  

