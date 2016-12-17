/* ADE7753.h
====================================================================================================
V0.01
fng
By: Ezequiel Pedace
Created:     7 Dic 2012
Last update: 7 Dic 2012

Analog Front End para el sistema Mew monofasico.


*/  
#ifndef ADE7753_H
#define ADE7753_H

/***=================================================================================================
Defines
=================================================================================================*/

// Class Atributes
#define AFECS 1                	//Chip Select ADE7753 se le suma luego 8 para compatibilizar con la funcion digitalWrite()  
#define WRITE 0x80				//Valor del addres para la funcion Write.
#define CLKIN 3579545         	//ADE7753 frec, max 4MHz
#define PERIODO 50				//Frecuencia de Red   
 
 
//Register address
 
//------Name--------Address---------Lenght
#define WAVEFORM 	0x01//			24
#define AENERGY 	0x02//			24
#define RAENERGY 	0x03//			24
#define LAENERGY 	0x04//			24
#define VAENERGY 	0x05//			24
#define RVAENERGY 	0x06//			24
#define LVAENERGY 	0x07//			24
#define LVARENERGY 	0x08//			24
#define MODE 		0x09//			16
#define IRQEN 		0x0A//			16
#define STATUS 		0x0B//			16
#define RSTSTATUS 	0x0C//			16
#define CH1OS 		0x0D//			8
#define CH2OS 		0x0E//			8
#define GAIN 		0x0F//			8
#define PHCAL 		0x10//			6
#define APOS 		0x11//			16
#define WGAIN 		0x12//			12
#define WDIV 		0x13//			8
#define CFNUM 		0x14//			12
#define CFDEN 		0x15//			12
#define IRMS 		0x16//			24
#define VRMS 		0x17//			24
#define IRMSOS 		0x18//			12
#define VRMSOS 		0x19//			12
#define VAGAIN 		0x1A//			12
#define VADIV 		0x1B//			8
#define LINECYC 	0x1C//			16
#define ZXTOUT 		0x1D//			12
#define SAGCYC 		0x1E//			8
#define SAGLVL 		0x1F//			8
#define IPKLVL 		0x20//			8
#define VPKLVL 		0x21//			8
#define IPEAK 		0x22//			24
#define RSTIPEAK 	0x23//			24
#define VPEAK 		0x24//			24
#define RSTVPEAK 	0x25//			24
#define TEMP 		0x26//			8
#define PERIOD 		0x27//			16
#define TMODE 		0x3D//			8
#define CHKSUM 		0x3E//			6
#define DIEREV 		0X3F//			8


//bits

/*MODE REGISTER (0x09)
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

#define DISHPF     0x0001
#define DISLPF2    0x0002 
#define DISCF      0x0004
#define DISSAG     0x0008
#define ASUSPEND   0x0010
#define TEMPSEL    0x0020
#define SWRST      0x0040
#define CYCMODE    0x0080
#define DISCH1     0x0100
#define DISCH2     0x0200
#define SWAP	   0x0400      
#define DTRT1      0x0800
#define DTRT0      0x1000
#define WAVSEL1    0x2000
#define WAVSEL0    0x4000
#define POAM       0x8000

/*INTERRUPT STATUS REGISTER (0x0B), RESET INTERRUPT STATUS REGISTER (0x0C), INTERRUPT ENABLE REGISTER (0x0A)
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

#define		AEHF	0x01
#define		SAG		0x02
#define		CYCEND	0x04
#define		WSMP	0x08
#define		ZX		0x10
#define		TEMP	0x20
#define		RESET	0x40
#define		AEOF	0x80
#define		PKV		0x0100
#define		PKI		0x0200
#define		VAEHF	0x0400
#define		VAEOF	0x0800
#define		ZXTO	0x1000
#define		PPOS	0x2000
#define		PNEG	0x4000

/*
CH1OS REGISTER (0x0D)
The CH1OS register is an 8-bit, read/write enabled register. 
The MSB of this register is used to switch on/off the digital integrator in Channel 1, 
and Bits 0 to 5 indicates the amount of the offset correction in Channel 1.

Bit Location	Bit Mnemonic		Description
5 to 0			OFFSET				The six LSBs of the CH1OS register control the amount of dc offset correction in Channel 1 ADC. 
									The 6-bit offset correction is sign and magnitude coded. 
									Bits 0 to 4 indicate the magnitude of the offset correction. 
									Bit 5 shows the sign of the offset correction. 
									A 0 in Bit 5 means the offset correction is positive and a 1 indicates the offset correction is negative.
6				Not Used			This bit is unused.
7				INTEGRATOR			This bit is used to activate the digital integrator on Channel 1. 
									The digital integrator is switched on by setting this bit. 
									This bit is set to be 0 on default.
*/

#define 	INTEGRATOR		0x80

/*
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


//constants
#define GAIN_1    0x00
#define GAIN_2    0x01
#define GAIN_4    0x02
#define GAIN_8    0x04
#define GAIN_16   0x08
#define INTEGRATOR_ON 1
#define INTEGRATOR_OFF 0
#define FULLSCALESELECT_0_5V    0x00
#define FULLSCALESELECT_0_25V   0x01
#define FULLSCALESELECT_0_125V  0x02







class ADE7753 {
  
//public methods
   public:
    ADE7753();
	void Init(void);
    void setSPI(void);
    void closeSPI(void);

//----------------------------------------------------------------------------      
// Modos y configs
//----------------------------------------------------------------------------      
	unsigned char getVersion();
	void setMode(int m);
	int getMode();
	void gainSetup(char integrator, char scale, char PGA2, char PGA1);
	int getInterrupts(void);
	void setInterrupts(int i);
	int getStatus(void);
	int resetStatus(void);
	long getIRMS(void);
	long getVRMS(void);
	long vrms();
	long irms();
	int getPeriod(void);
	void setLineCyc(int d);
	void setZeroCrossingTimeout(int d);
	int getZeroCrossingTimeout();
	char getSagCycles();
	void setSagCycles(char d);
	char getSagVoltageLevel();
	void setSagVoltageLevel(char d);
	char getIPeakLevel();
	void setIPeakLevel(char d);
	char getVPeakLevel();
	void setVPeakLevel(char d);
	long getIpeakReset(void);
	long getVpeakReset(void);
	char setPotLine(int Ciclos);
	long getWatt();
	long getVar(void);
	long getVa(void);
            
//private methods
   private:
      unsigned char read8(char reg);
      unsigned int read16(char reg);
      unsigned long read24(char reg);
      void write16(char reg, int data);
      void write8(char reg, char data);
      void enableChip(void);  
      void disableChip(void);

};


#endif
