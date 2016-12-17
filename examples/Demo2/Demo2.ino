

/* El siguiente programa es una demostración de las mediciones que el Energy Shield es capaz de realizar. Se abarcan medidas de Voltaje RMS 
* Corriente RMS , Energía Activa Acumulada durante un período de tiempo particular y Período de la señal de voltaje (Canal 2), cabe destacar que la energía acumulada durante un segundo 
* es igual a la potencia activa.

* Para ejecutar las mediciones se ocupó la proporcionalidad existente entre el valor entregado por el Energy Shield y el valor real de la magnitud 
* calculada. 

* Las constantes de proporcionalidad fueron calculadas de forma experimental (una demostración de aquellas pruebas se encuentran en el sketch DetConst), para el caso del Período la 
constante viene definida desde el datasheet.

*Las constante son las siguientes:

* Voltaje: kv=0.000000171849155 [volt/LSB]
* Corriente: ki=0.00000039122624397277 [amp/LSB]
* Energía Activa Acumulada: ke=0.0.00041576242446899414 [J/LSB]
* Período: kt=2.2*pow(10,-6) [seg/LSB]
*/

#include "ADE7753.h"  
#include "Rtc.h"
#include <SPI.h>   
#include <Wire.h>
#include <SoftwareSerial.h>

void setup(){
}

void loop() { 

ADE7753 meter;  
Rtc reloj = Rtc();
meter.analogSetup(GAIN_1,GAIN_1, 0  ,0,   0,0); 
meter.resetStatus();

long v1,i1,e1,e2,e3,t1;

float kv,ki,ke,kt;

//Constantes de proporcionalidad.

kv=0.000000171849155;                        
ki=0.00000039122624397277;
ke=0.00041576242446899414;
kt=2.2*pow(10,-6);

while(1){

//--Reloj de tiempo real--
  reloj.GetDate();
  
//--Medición de Voltaje--
  v1=meter.vrms();

//--Medición de Corriente--
  i1=meter.irms();
  
//--Medición de Período--
  t1=meter.getPeriod();
  
//Medición de Energía Acumulada
meter.setMode(0x0080); //Se inicia el modo de acumulación de energía.
meter.setLineCyc(100); //Se fija el número de medios ciclos ocupados en la medición. 100 medio ciclos equivalen a 1 segundo trabajando en una red de 50 Hz (Chile).
e1=meter.getLAENERGY(); //Extrae la energía activa acumulada, sincronizando la medición con los cruces por cero de la señal de voltaje. 

meter.setMode(0x0080);
meter.setLineCyc(200); // 2 segundos de medición.
e2=meter.getLAENERGY();

meter.setMode(0x0080);
meter.setLineCyc(300); // 3 segundos de medición.
e3=meter.getLAENERGY();
  
  Serial.println("\n------Nueva Medicion------");
  Serial.print(reloj.Date());//fecha
  Serial.print(" - ");
  Serial.print(reloj.Time());//hora
  Serial.print("\nVoltaje RMS [V]: ");
  Serial.print(kv*v1*51,15);
  Serial.print("\nCorriente RMS [A]: ");
  Serial.print(ki*i1*51,15);
  Serial.print("\nEnergia Activa Acumulada [J] en 1 seg : ");
  Serial.print(ke*(e3-e2)*51,15);                           //Se ve la diferencia entre la energía activa acumulada en 3 segundos y la acumulada en 2 segundos. Se desprecia la energía activa acumulada en 1 segundo ya que los valores de mediciones iniciales no siempre son correctas. 
  Serial.print("\nPeriodo de la senial de voltaje [seg]: ");
  Serial.println(t1*kt,5);     
}
}

