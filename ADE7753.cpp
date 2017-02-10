/* ADE7753.cpp
====================================================================================================
V0.01
fng
By: Ezequiel Pedace
Created:     7 Dic 2012
Last update: 7 Dic 2012


Analog Front End para el sistema Mew monofasico.

*/
#include "ADE7753.h"
#include <string.h>
#include <Arduino.h>
#include <SPI.h>

/** === ADE7753 ===
* Class constructor, sets chip select pin (CS define) and SPI communication with MCU.
* @param none
* @return void
*/

  ADE7753::ADE7753() {

  }

void ADE7753::Init(unsigned char AFECS){
	// SPI Init
	pinMode(AFECS,OUTPUT);
	digitalWrite(AFECS, HIGH);//disabled by default
	SPI.setDataMode(SPI_MODE2);
	SPI.setClockDivider(SPI_CLOCK_DIV2);
	SPI.setBitOrder(MSBFIRST);
	SPI.begin();
	delay(10);
}

void ADE7753::setSPI(void) {
	SPI.setDataMode(SPI_MODE2);
//	SPI.setClockDivider(SPI_CLOCK_DIV32);
//	SPI.setBitOrder(MSBFIRST);
	delay(10);
}

void ADE7753::closeSPI(void) {
	SPI.setDataMode(SPI_MODE0);
//	SPI.setClockDivider(SPI_CLOCK_DIV32);
//	SPI.setBitOrder(MSBFIRST);
	delay(10);
}

/*****************************
 *
 * private functions
 *
 *****************************/

/** === enableChip ===
* Enable chip, setting low ChipSelect pin (AFECS)
* @param none
*
*/
void ADE7753::enableChip(void){
  digitalWrite(AFECS,LOW);
}


/** === disableChip ===
* Disable chip, setting high ChipSelect pin (AFECS)
* @param none
*
*/
void ADE7753::disableChip(void){
  digitalWrite(AFECS,HIGH);
}


/**
 * Read 8 bits from the device at specified register
 * @param char containing register direction
 * @return char with contents of register
 *
 */
unsigned char ADE7753::read8(char reg){
    enableChip();
    delayMicroseconds(5);
    SPI.transfer(reg);
    delayMicroseconds(5);
    disableChip();
    return (unsigned long)SPI.transfer(0x00);

}


/**
 * Read 16 bits from the device at specified register
 * @param char containing register direction
 * @return int with contents of register
 *
 */
unsigned int ADE7753::read16(char reg){
    enableChip();
    unsigned char b1,b0;
    delayMicroseconds(5);
    SPI.transfer(reg);
    delayMicroseconds(5);
    b1=SPI.transfer(0x00);
    delayMicroseconds(5);
    b0=SPI.transfer(0x00);
    disableChip();
    return (unsigned int)b1<<8 | (unsigned int)b0;

}


/**
 * Read 24 bits from the device at specified register
 * @param: char containing register direction
 * @return: char with contents of register
 *
 */
unsigned long ADE7753::read24(char reg){
    enableChip();
    unsigned char b2,b1,b0;
    delayMicroseconds(10);
    SPI.transfer(reg);
    delayMicroseconds(25);
    b2=SPI.transfer(0x00);
    delayMicroseconds(5);
    b1=SPI.transfer(0x00);
    delayMicroseconds(5);
    b0=SPI.transfer(0x00);
    disableChip();
    return (unsigned long)b2<<16 | (unsigned long)b1<<8 | (unsigned long)b0;

}



/**
 * write8: Write 8 bits to the device at specified register
 * @param reg char containing register direction
 * @param data char, 8 bits of data to send
 *
 */
void ADE7753::write8(char reg, char data){
    enableChip();
    //we have to send a 1 on the 8th bit in order to perform a write
    reg |= WRITE;
    delayMicroseconds(10);
    SPI.transfer((unsigned char)reg);          //register selection
    delayMicroseconds(5);
    SPI.transfer((unsigned char)data);
    delayMicroseconds(5);
    disableChip();
}


/**
 * write16: Write 16 bits to the device at specified register
 * @param reg: char containing register direction
 * @param data: int, 16 bits of data to send
 *
 */
void ADE7753::write16(char reg, int data){
    enableChip();
    char aux=reg;
    unsigned char data0=0,data1=0;
    reg |= WRITE;
    //split data
    data0 = (unsigned char)data;
    data1 = (unsigned char)(data>>8);

    //register selection, we have to send a 1 on the 8th bit to perform a write
    delayMicroseconds(10);
    SPI.transfer((unsigned char)reg);
    delayMicroseconds(5);
    //data send, MSB first
    SPI.transfer((unsigned char)data1);
    delayMicroseconds(5);
    SPI.transfer((unsigned char)data0);
    delayMicroseconds(5);
    disableChip();
}

/*****************************
 *
 *     public functions
 *
 *****************************/


/**
 * In general:
 * @params:  void
 * @return: register content (measure) of the proper type depending on register width
 */

unsigned char ADE7753::getVersion(){
return read8(DIEREV);
}

/**=== setMode / getMode ===
MODE REGISTER (0x09)
The ADE7753 functionality is configured by writing to the mode register. Table 14 describes the functionality of each bit in the register.


Bit Location	Bit Mnemonic	Default Value 		Description
0				DISHPF			0					HPF (high-pass filter) in Channel 1 is disabled when this bit is set.
1				DISLPF2			0					LPF (low-pass filter) after the multiplier (LPF2) is disabled when this bit is set.
2				DISCF			1					Frequency output CF is disabled when this bit is set.
3				DISSAG			1					Line voltage sag detection is disabled when this bit is set.
4				ASUSPEND		0					By setting this bit to Logic 1, both ADE7753 A/D converters can be turned off.
													In normal operation, this bit should be left at Logic 0.
													All digital functionality can be stopped by suspending the clock signal at CLKIN pin.
5				TEMPSEL			0					Temperature conversion starts when this bit is set to 1.
													This bit is automatically reset to 0 when the temperature conversion is finished.
6				SWRST			0					Software Chip Reset. A data transfer should not take place to the
													ADE7753 for at least 18 µs after a software reset.
7				CYCMODE			0					Setting this bit to Logic 1 places the chip into line cycle energy accumulation mode.
8				DISCH1			0					ADC 1 (Channel 1) inputs are internally shorted together.
9				DISCH2			0					ADC 2 (Channel 2) inputs are internally shorted together.
10				SWAP			0					By setting this bit to Logic 1 the analog inputs V2P and V2N are connected to ADC 1
													and the analog inputs V1P and V1N are connected to ADC 2.
11 to 12		DTRT			0					These bits are used to select the waveform register update rate.
													DTRT1	DTRT0	Update Rate
													0		0		27.9 kSPS (CLKIN/128)
													0		1		14 kSPS (CLKIN/256)
													1		0		7 kSPS (CLKIN/512)
													1		1		3.5 kSPS (CLKIN/1024)
13 to 14		WAVSEL			0					These bits are used to select the source of the sampled data for the waveform register.
													WAVSEL[1:0]		Source
													0	0			24 bits active power signal (output of LPF2)
													0	1			Reserved
													1	0			24 bits Channel 1
													1	1			24 bits Channel 2
15				POAM			0					Writing Logic 1 to this bit allows only positive active power to be accumulated in the ADE7753.
*/
void ADE7753::setMode(int m){
    write16(MODE, m);
}
int ADE7753::getMode(){
    return read16(MODE);
}

/** === gainSetup ===

GAIN REGISTER (0x0F)
The PGA configuration of the ADE7753 is defined by writing to the GAIN register.
Table 18 summarizes the functionality of each bit in the GAIN register.

Bit Location		Bit Mnemonic		Default Value		Description
0 to 2				PGA1				0					Current GAIN
															PGA1[2:0]			Description
															0	0	0			x1
															0	0	1			x2
															0	1	0			x4
															0	1	1			x8
															1	0	0			x16
3 to 4				SCALE				0					Current input full-scale select
															SCALE[1:0]			Description
															0	0				0.5v
															0	1				0.25v
															1	0				0.125v
															1	1				Reserved
5 to 7				PGA2				0					Voltage GAIN
															PGA2[2:0]			Description
															0	0	0			x1
															0	0	1			x2
															0	1	0			x4
															0	1	1			x8
															1	0	0			x16
*/

void ADE7753::gainSetup(char integrator, char scale, char PGA2, char PGA1){
char pgas = (PGA2<<5) | (scale<<3) | (PGA1);
write8(GAIN,pgas);//write GAIN register, format is |3 bits PGA2 gain|2 bits full scale|3 bits PGA1 gain
char ch1os = (integrator<<7);
write8(CH1OS,ch1os);
}

/**	getStatus()/resetStatus()/getInterrupts()/setInterrupts(int i)
INTERRUPT STATUS REGISTER (0x0B), RESET INTERRUPT STATUS REGISTER (0x0C), INTERRUPT ENABLE REGISTER (0x0A)
The status register is used by the MCU to determine the source of an interrupt request (IRQ).
When an interrupt event occurs in the ADE7753, the corresponding flag in the interrupt status register is set to logic high.
If the enable bit for this flag is Logic 1 in the interrupt enable register, the IRQ logic output goes active low.
When the MCU services the interrupt, it must first carry out a read from the interrupt status register to determine the source of the interrupt.


Bit Location	Interrupt Flag		Description
0				AEHF				Indicates that an interrupt occurred because the active energy register, AENERGY, is more than half full.
1				SAG					Indicates that an interrupt was caused by a SAG on the line voltage.
2				CYCEND				Indicates the end of energy accumulation over an integer number of half line cycles as defined by
									the content of the LINECYC register—see the Line Cycle Energy Accumulation Mode section.
3				WSMP				Indicates that new data is present in the waveform register.
4				ZX					This status bit is set to Logic 0 on the rising and falling edge of the the voltage waveform.
									See the Zero-Crossing Detection section.
5				TEMP				Indicates that a temperature conversion result is available in the temperature register.
6				RESET				Indicates the end of a reset (for both software or hardware reset).
									The corresponding enable bit has no function in the interrupt enable register, i.e.,
									this status bit is set at the end of a reset, but it cannot be enabled to cause an interrupt.
7				AEOF				Indicates that the active energy register has overflowed.
8				PKV					Indicates that waveform sample from Channel 2 has exceeded the VPKLVL value.
9				PKI					Indicates that waveform sample from Channel 1 has exceeded the IPKLVL value.
A				VAEHF				Indicates that an interrupt occurred because the active energy register, VAENERGY, is more than half full.
B				VAEOF				Indicates that the apparent energy register has overflowed.
C				ZXTO				Indicates that an interrupt was caused by a missing zero crossing on the line voltage for the
									specified number of line cycles—see the Zero-Crossing Timeout section.
D				PPOS				Indicates that the power has gone from negative to positive.
E				PNEG				Indicates that the power has gone from positive to negative.
F				RESERVED			Reserved.

*/
int ADE7753::getInterrupts(void){
    return read16(IRQEN);
}
void ADE7753::setInterrupts(int i){
    write16(IRQEN,i);
}
int ADE7753::getStatus(void){
    return read16(STATUSR);
}
int ADE7753::resetStatus(void){
    return read16(RSTSTATUS);
}

/** === getIRMS ===
* Channel 2 RMS Value (Current Channel).
* The update rate of the Channel 2 rms measurement is CLKIN/4.
* To minimize noise, synchronize the reading of the rms register with the zero crossing
* of the voltage input and take the average of a number of readings.
* @param none
* @return long with the data (24 bits unsigned).
*/
long ADE7753::getIRMS(void){
	long lastupdate = 0;
	char t_of = 0;
	resetStatus(); // Clear all interrupts
	lastupdate = millis();
	while(!(getStatus()&ZX)){   // wait Zero-Crossing
	if ((millis() - lastupdate) > 20){
		t_of = 1;
		break;
		}
	}
	if (t_of){
	return 0;
	}else{
	return read24(IRMS);
	}
}

/** === getVRMS ===
* Channel 2 RMS Value (Voltage Channel).
* The update rate of the Channel 2 rms measurement is CLKIN/4.
* To minimize noise, synchronize the reading of the rms register with the zero crossing
* of the voltage input and take the average of a number of readings.
* @param none
* @return long with the data (24 bits unsigned).
*/
long ADE7753::getVRMS(void){
	long lastupdate = 0;
	char t_of = 0;
	resetStatus(); // Clear all interrupts
	lastupdate = millis();
	while(!(getStatus()&ZX)){// wait Zero-Crossing
	if((millis()-lastupdate) > 20                        ){
		t_of = 1;
		break;
		}
	}
	if(t_of){
		return 0;
		}else{
		return read24(VRMS);
		}
}

/** === vrms ===
* Returns the mean of last 100 readings of RMS voltage. Also supress first reading to avoid
* corrupted data.
* rms measurement update rate is CLKIN/4.
* To minimize noise, synchronize the reading of the rms register with the zero crossing
* of the voltage input and take the average of a number of readings.
* @param none
* @return long with RMS voltage value
*/
long ADE7753::vrms(){
	char i=0;
	long v=0;
	if(getVRMS()){//Ignore first reading to avoid garbage

	for(i=0;i<2;++i){
		v+=getVRMS();
	}
	return v/2;
	}else{
	return 0;
	}
}

/** === irms ===
* Returns the mean of last 100 readings of RMS current. Also supress first reading to avoid
* corrupted data.
* rms measurement update rate is CLKIN/4.
* To minimize noise, synchronize the reading of the rms register with the zero crossing
* of the voltage input and take the average of a number of readings.
* @param none
* @return long with RMS current value in hundreds of [mA], ie. 6709=67[mA]
*/
long ADE7753::irms(){
	char n=0;
	long i=0;
	if(getIRMS()){//Ignore first reading to avoid garbage
	for(n=0;n<2;++n){
		i+=getIRMS();
	}
	return i/2;
	}else{
	return 0;
	}
}

/**
 * Period of the Channel 2 (Voltage Channel) Input Estimated by Zero-Crossing Processing. The MSB of this register is always zero.
 * @param none
 * @return int with the data (16 bits unsigned).
 */
int ADE7753::getPeriod(void){
    return read16(PERIOD);
}

/**
 * Line Cycle Energy Accumulation Mode Line-Cycle Register.
 * This 16-bit register is used during line cycle energy accumulation  mode
 * to set the number of half line cycles for energy accumulation
 * @param none
 * @return int with the data (16 bits unsigned).
 */
void ADE7753::setLineCyc(int d){
    write16(LINECYC,d);
}

/**
 * Zero-Crossing Timeout. If no zero crossings are detected
 * on Channel 2 within a time period specified by this 12-bit register,
 * the interrupt request line (IRQ) is activated
 * @param none
 * @return int with the data (12 bits unsigned).
 */
void ADE7753::setZeroCrossingTimeout(int d){
    write16(ZXTOUT,d);
}
int ADE7753::getZeroCrossingTimeout(){
    return read16(ZXTOUT);
}

/**
 * Sag Line Cycle Register. This 8-bit register specifies the number of
 * consecutive line cycles the signal on Channel 2 must be below SAGLVL
 * before the SAG output is activated.
 * @param none
 * @return char with the data (8 bits unsigned).
 */
char ADE7753::getSagCycles(){
    return read8(SAGCYC);
}
void ADE7753::setSagCycles(char d){
    write8(SAGCYC,d);
}

/**
 * Sag Voltage Level. An 8-bit write to this register determines at what peak
 * signal level on Channel 2 the SAG pin becomes active. The signal must remain
 * low for the number of cycles specified in the SAGCYC register before the SAG pin is activated
 * @param none
 * @return char with the data (8 bits unsigned).
 */
char ADE7753::getSagVoltageLevel(){
    return read8(SAGLVL);
}
void ADE7753::setSagVoltageLevel(char d){
    write8(SAGLVL,d);
}

/**
 * Channel 1 Peak Level Threshold (Current Channel). This register sets the levelof the current
 * peak detection. If the Channel 1 input exceeds this level, the PKI flag in the status register is set.
 * @param none
 * @return char with the data (8 bits unsigned).
 */
char ADE7753::getIPeakLevel(){
    return read8(IPKLVL);
}
void ADE7753::setIPeakLevel(char d){
    write8(IPKLVL,d);
}

/**
 * Channel 2 Peak Level Threshold (Voltage Channel). This register sets the level of the
 * voltage peak detection. If the Channel 2 input exceeds this level,
 * the PKV flag in the status register is set.
 * @param none
 * @return char with the data (8bits unsigned).
 */
char ADE7753::getVPeakLevel(){
    return read8(VPKLVL);
}
void ADE7753::setVPeakLevel(char d){
    write8(VPKLVL,d);
}

/**
 * Same as Channel 1 Peak Register except that the register contents are reset to 0 after read.
 * @param none
 * @return long with the data (24 bits 24 bits unsigned).
 */
long ADE7753::getIpeakReset(void){
    return read24(RSTIPEAK);
}

/**
 * Same as Channel 2 Peak Register except that the register contents are reset to 0 after a read.
 * @param none
 * @return long with the data (24 bits  unsigned).
 */
long ADE7753::getVpeakReset(void){
    return read24(RSTVPEAK);
}

/** === setPotLine(Phase) ===
Setea las condiciones para Line accumulation.
Luego espera la interrupccion y devuelve 1 cuando ocurre.
Si no ocurre por mas de 1,5 segundos devuelve un 0.
**/
char ADE7753::setPotLine(int Ciclos){
long lastupdate = 0;
char t_of = 0;
int m = 0;
m = m | DISCF | DISSAG | CYCMODE;
setMode(m);
resetStatus();
setLineCyc(Ciclos);
lastupdate = millis();
while(!(getStatus() & CYCEND))   // wait to terminar de acumular
	{ // wait for the selected interrupt to occur
		if ((millis() - lastupdate) > (Ciclos*20))
		{
		t_of = 1;
		break;
		}
	}
if(t_of){
	return 0;
}
resetStatus();
lastupdate = millis();
while(!(getStatus() & CYCEND))   // wait to terminar de acumular
	{ // wait for the selected interrupt to occur
		if ((millis() - lastupdate) > (Ciclos*20))
		{
		t_of = 1;
		break;
		}
	}
	if(t_of){
	return 0;
	}else{
	return 1;
	}
}
/** === getWatt() / getVar() / getVa() ===
Devuelve los valores de la potencia requerida.
Utilizar antes setPotLine() para generar los valores.
**/
long ADE7753::getWatt(){
return read24(LAENERGY);
}
long ADE7753::getVar(){
return read24(LVARENERGY);
}
long ADE7753::getVa(){
return read24(LVAENERGY);

}
