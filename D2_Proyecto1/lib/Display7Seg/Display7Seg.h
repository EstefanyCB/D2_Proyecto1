#ifndef __DISPLAY7SEG_H__
#define __DISPLAY7SEG_H__

#include <Arduino.h>

extern uint8_t pinA, pinB, pinC, pinD, pinE, pinF, pinG, pindp;

//Funcion para configurar display de 7 segmentos
void configurarDisplay(uint8_t A, uint8_t B, uint8_t C, uint8_t D, uint8_t E, uint8_t F, uint8_t G, uint8_t dp);

//Funcion pra desplegar el digito en el display de 7 segmentos
void desplegar7Segmentos(uint8_t digito);

//Funcion para desplegar el punto decimal
void desplegarPunto(boolean punto); //Punto solo va a se 0/1 para desplegarlo

#endif // __DISPLAY7SEG_H__