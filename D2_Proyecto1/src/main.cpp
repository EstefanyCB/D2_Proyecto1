//******************************************************************************************
//Universidad del Valle de Guatemala
//BE3015: Electronica Digital 2
//Estefany Eleuteria Batz Cantor
//Proyecto 1
//Estado: Parte1, 2 y 3 funcionales
//******************************************************************************************

//******************************************************************************************
//Librerias
//******************************************************************************************
#include <Arduino.h>
#include "esp_adc_cal.h"
#include "Display7Seg.h"

//******************************************************************************************
//Definición de pines
//******************************************************************************************
#define LM35 15 //Senal del sensor de temperatura

//Boton
#define Boton 22

//Pines de salida del Led RGB
#define LedR 19
#define LedG 21
#define LedB 3

//parametros de la senal PWM del led y Servo
#define PWMServo 5       //Este canal se conecta al pin GPIO12
#define frequPWMServo 50 //Frecuencia del servomotor SG90 en Hz

#define resolution 8 //1-16 bits de resolucion, se usara la misma para los leds y el servo

#define PWMLedR 1        //Salida en GPIO13
#define PWMLedG 2        //Salida en GPIO14
#define PWMLedB 3        //Salida en GPIO27
#define frequPWMLed 5000 //frecuencia de los leds

#define Servo 4 //salida del PWM

//Pines del Display
#define DisplayA 17
#define DisplayB 5
#define DisplayC 32
#define DisplayD 33
#define DisplayE 25
#define DisplayF 12
#define DisplayG 13
#define Displaydp 16

//Transistores
#define Decena 14
#define Unidad 27
#define Decima 26

//Para el timer
#define prescaler 80

//******************************************************************************************
//Prototipo de funciones
//******************************************************************************************
void ConfiguracionSLPWM(void); //PWM de los leds y el servo

void ConfigurarLedsServo(void); //Configuracion del led y el servo
void SensorTemperaturaLedServo(void);

void TempDisplay(void);

void IRAM_ATTR TimerISR();

void ConfigutacionTimer(void);
float ReadVoltage(int ADC_Raw);

//******************************************************************************************
//Variables Globales
//******************************************************************************************

//Sensor de temperatura
float Temperatura = 0.0; //Para la temperatura medida
float Voltaje = 0.0;
int ValorTemp = 0; //Para almacenar la conversion ADC

long lastTime;
int sampleTime = 150;

int StateBoton = 0;

//Display 7 segmentos
int VDecena = 0;
int VUnidad = 0;
int VDecima = 0;

//Temporizador
hw_timer_t * timer = NULL;
int CTimer = 0;

//******************************************************************************************
//Interrupciones ISR
//******************************************************************************************
void IRAM_ATTR TimerISR(){
  CTimer++;
  if (CTimer >2)
  {
    CTimer = 0;
  }
}

//******************************************************************************************
//Configuracion
//******************************************************************************************
void setup()
{
  Serial.begin(115200);

  lastTime = millis();

  pinMode(Boton, INPUT_PULLDOWN);

  pinMode(LedB, OUTPUT);
  pinMode(LedR, OUTPUT);
  pinMode(LedG, OUTPUT);
  pinMode(Servo, OUTPUT);

  ConfigutacionTimer();
  ConfiguracionSLPWM();
  SensorTemperaturaLedServo();


  configurarDisplay(DisplayA, DisplayB, DisplayC, DisplayD, DisplayE, DisplayF, DisplayG, Displaydp);

  pinMode(Decena, OUTPUT);
  pinMode(Unidad, OUTPUT);
  pinMode(Decima, OUTPUT);

  digitalWrite(Decena, LOW);
  digitalWrite(Unidad, LOW);
  digitalWrite(Decima, LOW);

  desplegar7Segmentos(0);
}

//******************************************************************************************
//Loop principal
//******************************************************************************************
void loop()
{
  if (millis() - lastTime >= sampleTime)
  {
    lastTime = millis();
    SensorTemperaturaLedServo(); //Funcion que se encarga de leer la temperatura y sincronizar el led+servo
  }
}

//******************************************************************************************
//Configuracion de la senal ADC
//******************************************************************************************
float ReadVoltage(int ADC_Raw)
{
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, 1100, &adc_chars);
  return (esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

//******************************************************************************************
//Configuracion del sensor de Temperatura
//******************************************************************************************
void SensorTemperaturaLedServo(void)
{
  if (digitalRead(Boton) == HIGH)
  {
    //Paso 1 Temperatura. Leer la entrada analogica
    ValorTemp = analogReadMilliVolts(LM35); //Se almacena el valor entre 0-4095 que representa la temperatura
    //El voltaje que se lee en en el pin es ValorTemp=(ValorLeido)*3.3V/4095
    //En el LM35 1C equivale a 10mV, entonces temperatura=VoltajePin/10mV
    Temperatura = ValorTemp / 10.0;
    Serial.print(Temperatura);
    Serial.print("; ");

    ConfigurarLedsServo(); //Dependiendo del valor de la temperatura se sincroniza el LED y el Servo
  }
}

//******************************************************************************************
//Configuracion de Leds y Servo dependiendo del valor de Temperatura
//******************************************************************************************
void ConfigurarLedsServo(void)
{
  if (Temperatura < 25.0) //Rango para el Led Verde
  {
    Serial.println("Primer rango");
    ledcWrite(PWMLedR, 200); //Led Red se enciende segun su DutyCycle
    ledcWrite(PWMLedG, 0);   //Apagada
    ledcWrite(PWMLedB, 0);   //Apagada
    ledcWrite(PWMServo, 8);  //Los rangos del servo son de 7-32 ya que se utilizo una resolucion de 8
  }

  else if (Temperatura >= 25.0 && Temperatura < 27.5) //Rango para el Led Amarillo
  {
    Serial.println("Segundo Rango");
    ledcWrite(PWMLedR, 0);   //Led Red se enciende segun su DutyCycle
    ledcWrite(PWMLedG, 255); //Apagada
    ledcWrite(PWMLedB, 0);   //Apagada
    ledcWrite(PWMServo, 15);
  }

  else if (Temperatura >= 27.5) //Rango para el Led Rojo
  {
    Serial.println("Tercer Rango");
    ledcWrite(PWMLedR, 0);   //Led Red se enciende segun su DutyCycle
    ledcWrite(PWMLedG, 0);   //Apagada
    ledcWrite(PWMLedB, 255); //Apagada
    ledcWrite(PWMServo, 32);
  }
}

//******************************************************************************************
//Configuracion modulo PWM del servo
//******************************************************************************************
void ConfiguracionSLPWM(void)
{

  // Paso 1: Configurar el módulo PWM
  ledcSetup(PWMServo, frequPWMServo, resolution); //Unimos las variables determinadas arriba

  // Paso 2: seleccionar en que GPIO tendremos nuestra señal PWM
  ledcAttachPin(Servo, PWMServo); //Aqui es la señal del PWM, entonces seleccionamos el pin

  // Paso 1: Configurar el módulo PWM
  ledcSetup(PWMLedR, frequPWMLed, resolution); //Unimos las variables determinadas arriba
  ledcSetup(PWMLedG, frequPWMLed, resolution); //Unimos las variables determinadas arriba
  ledcSetup(PWMLedB, frequPWMLed, resolution); //Unimos las variables determinadas arriba

  // Paso 2: seleccionar en que GPIO tendremos nuestra señal PWM
  ledcAttachPin(LedR, PWMLedR); //Aqui es la señal del PWM, entonces seleccionamos el pin
  ledcAttachPin(LedG, PWMLedG); //Aqui es la señal del PWM, entonces seleccionamos el pin
  ledcAttachPin(LedB, PWMLedB); //Aqui es la señal del PWM, entonces seleccionamos el pin
}

//******************************************************************************************
//Configuracion modulo PWM del servo
//******************************************************************************************
void ConfigutacionTimer(void)
{
  //Seleccionar el timer
    //Fosc/prescaler = 80 000 000/ 1 000 000
    //Tosc=1/Fosc=1 us
  timer = timerBegin(0, prescaler, true);
  
  //Paso 3: Handler de la interrupcion
  timerAttachInterrupt(timer, &TimerISR, true);
  
  //Paso 4: programacion de la alarma
  timerAlarmWrite(timer, 5000, true);
  
  timerAlarmEnable(timer); //Inicialzacion de la alarma
}

//******************************************************************************************
//Configuracion Temperatura de los display
//******************************************************************************************
void TempDisplay(void)
{
 int VTemp = Temperatura * 10;
 VDecena = VTemp/100;

 VTemp = VTemp - (VDecena*100);
 VUnidad = VTemp/10;

 VTemp = VTemp - (VUnidad * 10);
 VDecima = VTemp;
}

//******************************************************************************************
//Configuracion Temperatura de los display
//******************************************************************************************