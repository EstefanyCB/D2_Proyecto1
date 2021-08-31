//*******************************************************************
//Universidad del Valle de Guatemala
//BE3015: Electronica Digital 2
//Estefany Eleuteria Batz Cantor
//Proyecto 1
//Estado: Parte1-4. Sensor de temperatura y displays
//*******************************************************************

//*********************************************
//Librerias
//*********************************************
#include <Arduino.h>

//*********************************************
//Definición de pines
//*********************************************
#define LM35 4 //Senal del sensor de temperatura


//*********************************************
//Prototipo de funciones
//*********************************************


//*********************************************
//Variables Globales
//*********************************************
float Temperatura; //Para la temperatura medida
int ValorTemp; //Para almacenar la conversion ADC
//*********************************************
//Configuracion
//*********************************************
void setup() {
  Serial.begin(115200); 
}

//*********************************************
//Loop principal
//*********************************************
void loop() {
  //Paso 1 Temperatura. Leer la entrada analogica
  ValorTemp = analogRead(LM35); //Se almacena el valor entre 0-1023 que representa la temperatura
 
  //El voltaje que se lee en en el pin es ValorTemp=(ValorLeido)*3.3V/1023
  //En el LM35 1C equivale a 10mV, entonces temperatura=VoltajePin/10mV
 
  Temperatura=((ValorTemp*3.3)/1023.0) / 0.01;
  Serial.println(Temperatura);
  delay(100);
}