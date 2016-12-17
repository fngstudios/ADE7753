/**El siguiente sketch muestra como determinar las constantes de proporcionalidad de manera experimental, 
* luego se observan los valores entregados por el CI ADE7753 antes y después de aplicar la constante de 
* proporcionalidad.
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

float V,I,E;  

float kv,ki,ke,kt;

while(1){

/* Cálculo de la constante de proporcionalidad para la medición de Voltaje RMS, para ello en este caso se ocupó 
* una entrada de Voltaje RMS de 9 [Volts]
*/

V=9; // Valor predeterminado de entrada de voltaje, en el caso que se ocupe otra entrada solo basta cambiar esta variable.

meter.setKV(V); //Se calcula el valor de la constante de proporcionalidad de forma interna.

kv=meter.getKV();

v1=meter.vrms(); //Voltaje RMS a la salida del ADE7753 (Palabra de 24 Bits).

Serial.println("\n---Voltaje RMS---");
Serial.print("Constante de Proporcionalidad Kv: ");
Serial.println(kv,15);
Serial.print("Voltaje RMS a la salida del ADE7753 (24 bits): ");
Serial.println(v1,DEC);
Serial.print("Voltaje RMS a la salida del Energy Shield (x 51): ");
Serial.println(v1*51,DEC);
Serial.print("Voltaje RMS a la salida del Energy Shield [Volts]: ");
Serial.println(v1*51*kv,DEC);

/* Cálculo de la constante de proporcionalidad para la medición de Corriente RMS, para ello en este caso se ocupó 
* una entrada de Corriente RMS de 0.454 [Amperes]
*/

I=0.454; // Valor predeterminado de entrada de corriente, en el caso que se ocupe otra entrada solo basta cambiar esta variable.

meter.setKI(I); //Se calcula el valor de la constante de proporcionalidad de forma interna.

ki=meter.getKI();

i1=meter.irms(); //Corriente RMS a la salida del ADE7753 (Palabra de 24 Bits).

Serial.println("\n---Corriente RMS---");
Serial.print("Constante de Proporcionalidad Ki: ");
Serial.println(ki,15);
Serial.print("Corriente RMS a la salida del ADE7753 (24 bits): ");
Serial.println(i1,DEC);
Serial.print("Corriente RMS a la salida del Energy Shield (x 51): ");
Serial.println(i1*51,DEC);
Serial.print("Corriente RMS a la salida del Energy Shield [Amperes]: ");
Serial.println(i1*51*ki,DEC);

/* Cálculo de la constante de proporcionalidad para la medición de Energía Acumulada en 1 segundo (Potencia Activa), para ello en este caso se ocupó 
* un consumo a la entreda del Shield de 4.086 [Joules/seg].
*/

E=4.086; // Valor predeterminado de entrada de corriente, en el caso que se ocupe otra entrada solo basta cambiar esta variable.

meter.setKE(E); //Se calcula el valor de la constante de proporcionalidad de forma interna.

ke=meter.getKE();

//Medición de Energía Acumulada
meter.setMode(0x0080); //Se inicia el modo de acumulación de energía.
meter.setLineCyc(100); //Se fija el número de medios ciclos ocupados en la medición. 100 medio ciclos equivalen a 1 segundo trabajando en una red de 50 Hz (Chile).
e1=meter.getLAENERGY(); //Extrae la energía activa acumulada, sincronizando la medición con los cruces por cero de la señal de voltaje. 

meter.setLineCyc(200); // 2 segundos de medición.
e2=meter.getLAENERGY();

meter.setLineCyc(300); // 3 segundos de medición.
e3=meter.getLAENERGY(); //Energía Acumulada en 1 seg, a la salida del ADE7753 (Palabra de 24 Bits).

/** La energía acumulada en 1 segundo que se considera es la que transcurre entre el segundo 2 y el segundo 3, 
* la energía acumulada durante el primer segundo no se considera ya que entrega resultados incosistentes.
*/

Serial.println("\n---Energia Acumulada en 1 segundo---");
Serial.print("Constante de Proporcionalidad Ke: ");
Serial.println(ke,15);
Serial.print("Energia Acumulada en 1 seg, a la salida del ADE7753 (24 bits): ");
Serial.println(e3-e2,DEC);
Serial.print("Energia Acumulada en 1 seg, a la salida del Energy Shield (x 51): ");
Serial.println((e3-e2)*51,DEC);
Serial.print("Energia Acumulada en 1 seg, a la salida del Energy Shield [Joules/seg]: ");
Serial.println((e3-e2)*51*ke,DEC);

}
}
