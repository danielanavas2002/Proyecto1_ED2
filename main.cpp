//****************************************************************
// Librerías
//****************************************************************
#include <Arduino.h>
#include "driver/ledc.h"
#include "config.h"
//****************************************************************
// Definición de etiquetas
//****************************************************************
//Definicion de Canales
#define servoChannel 0 // 16 canales 0-15
#define ledRChannel 1
#define ledGChannel 2
#define ledBChannel 3
//Definicion de Frecuencias
#define freqLED 1000 // Frecuencia en Hz
#define freqServo 50 // Frecuencia en Hz
//Definicion de Resoluciones
#define resolution 8 // 1-16 bits de resolución
//Definicion Pin de Boton
#define pinBoton 22 //Boton toma de Temperatura
//Definicion Pin de Sensor (Analogico)
#define pinADC 36 //Sensor de Temperatura
//Definicion Pines PWM
#define pinLedR 19 //Led Rojo
#define pinLedG 18 //Led Verde
#define pinLedB 5 //Led Azul
#define pinServo 23 //Servo
//Definicion Pines de Displays
#define pinA 33 //A
#define pinB 32 //B
#define pinC 27 //C
#define pinD 26 //D
#define pinE 13 //E
#define pinF 25 //F
#define pinG 14 //G
//Definicion Transistores
#define pinT1 4 //Transistor 1
#define pinT2 16 //Transistor 2
#define pinT3 17 //Transistor 3
//****************************************************************
// Prototipos de funciones
//****************************************************************
void configurarPWM(void);
void semaforo(void);
void mediaMovil(void);
void display(int pinDigito, int num);
//****************************************************************
// Variables Globales
//****************************************************************
//Funcionamiento Boton
int estadoBoton;
int estadoAnteriorBoton = 0;
bool banderaLectura = false;
//Valor de Temperatura
int tempAnalogico;
float tempCelsius = 0.0;
//Filtro Media Movil
long mAvgSuma = 0; // Valor de adc media movil
float bufferLecturas[10]; // Buffer de lecturas
int indexLecturas = 0; // índice de muestras
int numLecturas = 10; // Número de muestras
long tempAnalogicoFiltrado = 0; // Valor de adc filtrado
//Display
int decena;
int unidad;
int decimal;
//****************************************************************
// Configuración Adafruit
//****************************************************************
//set up the 'Canal de Temperatura' feed
AdafruitIO_Feed *tempCanal = io.feed("tempCelsius");
//****************************************************************
// Configuración
//****************************************************************
void setup(){
  pinMode(pinBoton, INPUT_PULLDOWN);
  pinMode(pinADC, INPUT);
  configurarPWM();

  ledcWrite(servoChannel, 6);

  // Configurar los pines de Display y Transistores como salidas 
  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  pinMode(pinC, OUTPUT);
  pinMode(pinD, OUTPUT);
  pinMode(pinE, OUTPUT);
  pinMode(pinF, OUTPUT);
  pinMode(pinG, OUTPUT);
  pinMode(pinT1, OUTPUT);
  pinMode(pinT2, OUTPUT);
  pinMode(pinT3, OUTPUT);
  
  // Inicializar los pines de segmentos apagados
  digitalWrite(pinA, HIGH);
  digitalWrite(pinB, HIGH);
  digitalWrite(pinC, HIGH);
  digitalWrite(pinD, HIGH);
  digitalWrite(pinE, HIGH);
  digitalWrite(pinF, HIGH);
  digitalWrite(pinG, HIGH);

  // Inicializa los transistores apagados
  digitalWrite(pinT1, HIGH);
  digitalWrite(pinT2, HIGH);
  digitalWrite(pinT3, HIGH); 

  Serial.begin(115200);
  // wait for serial monitor to open
  
  while(! Serial);

  Serial.print("Connecting to Adafruit IO");

  // connect to io.adafruit.com
  io.connect();

  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  } 

  // we are connected
  Serial.println();
  Serial.println(io.statusText());

}
//****************************************************************
// Loop Principal
//****************************************************************
void loop() {
  estadoBoton = digitalRead(pinBoton);
  delay(100);
  io.run(); 

  if (estadoBoton == 1 && estadoAnteriorBoton == 0) {
    banderaLectura = true;
  }
  estadoAnteriorBoton = estadoBoton;

  if(banderaLectura){ //Cuando se active la Bandera
    semaforo(); //Ejecutar medicion y actualizar semaforo
    banderaLectura = false; //Desactivar Bandera
  } 
  /**/
  //Mostrar la temperatura en los displays
  display(pinT1, decena); //Mostrar el primer digito
  delay(5); //Pequeña pausa 
  display(pinT2, unidad); //Mostrar el segundo digito
  delay(5); //Pequeña pausa
  display(pinT3, decimal); //Mostrar el tercer digito
  delay(5); //Pequeña pausa
  /*
  tempAnalogico = analogRead(pinADC);
  tempCelsius = map(tempAnalogico, 0, 4095, 6, 33);
  ledcWrite(servoChannel, tempCelsius); */

}
//****************************************************************
// Función para configurar módulo PWM
//****************************************************************
void configurarPWM(void){
  // Paso 1: Configurar el módulo PWM
  ledcSetup(ledRChannel, freqLED, resolution);
  ledcSetup(ledGChannel, freqLED, resolution);
  ledcSetup(ledBChannel, freqLED, resolution);
  ledcSetup(servoChannel, freqServo, resolution);

  // Paso 2: seleccionar en que GPIO tendremos nuestra señal PWM
  ledcAttachPin(pinLedR, ledRChannel);
  ledcAttachPin(pinLedG, ledGChannel);
  ledcAttachPin(pinLedB, ledBChannel);
  ledcAttachPin(pinServo, servoChannel);
}
// ****************************************************************************
// Funcion Media Movil
// ****************************************************************************
void semaforo(void){
    Serial.println("LECTURA DE TEMPERATURA");
    mediaMovil();
    tempCelsius = map(tempAnalogicoFiltrado, 0, 2047, 20, 1500)/10.0; //tempCelsius = map(tempAnalogicoFiltrado, 0, 4095, 20, 1500)/10.0;
    Serial.println(tempAnalogico);
    Serial.println(tempCelsius);
    //Limites de Temperatura y acciones a realizar en LED y SERVO
    if(tempCelsius <= 37.0){
      Serial.println("TEMPERATURA NORMAL");
      ledcWrite(servoChannel, 6); //map(tempCelsius*10, 350, 370, 6, 14)
      ledcWrite(ledRChannel, 0);
      ledcWrite(ledGChannel, map(tempCelsius*10, 20, 370, 10, 60));
      ledcWrite(ledBChannel, 0);
    } if(37.0 < tempCelsius && tempCelsius < 37.5){
      Serial.println("TEMPERATURA MODERADA");
      ledcWrite(servoChannel, 20); //map(tempCelsius*10, 371, 374, 15, 23)
      ledcWrite(ledRChannel, map(tempCelsius*10, 371, 374, 10, 60));
      ledcWrite(ledGChannel, map(tempCelsius*10, 371, 374, 10, 60));
      ledcWrite(ledBChannel, 0);
    } if(tempCelsius >= 37.5){
      Serial.println("TEMPERATURA ALTA");
      ledcWrite(servoChannel, 33); //map(tempCelsius*10, 375, 400, 24, 33)
      ledcWrite(ledRChannel, map(tempCelsius*10, 375, 1500, 10, 60));
      ledcWrite(ledGChannel, 0);
      ledcWrite(ledBChannel, 0);
    }
    //Obtener la decena, unidad y decimal del valor de temperatura
    decena = int(tempCelsius)/10;
    unidad = int(tempCelsius)%10;
    decimal = ((tempCelsius*10)-(decena*100)) - (unidad*10);

    //Enviar datos con Adafruit
    Serial.print("sending -> ");
    Serial.println(tempCelsius);
    tempCanal->save(tempCelsius);
    delay(3000);
} 
// ****************************************************************************
// Funcion Media Movil
// ****************************************************************************
void mediaMovil(void){
  mAvgSuma = 0;  // Reinicia la suma de temperaturas
    for (int i = 0; i < numLecturas; i++) { //Adquirir y sumar 10 datos
      tempAnalogico = analogRead(pinADC);
      mAvgSuma += tempAnalogico;
    }
    tempAnalogicoFiltrado = mAvgSuma / numLecturas; //Promediar los 10 datos
} 
// ****************************************************************************
// Funcion para Display
// ****************************************************************************
void display(int pinDigito, int num) {
  //Establecer que segmento se debe encender para cada numero
  digitalWrite(pinA, num != 0 && num != 2 && num != 3 && num != 5 && num != 6 && num != 7 && num != 8 && num != 9);
  digitalWrite(pinB, num != 0 && num != 1 && num != 2 && num != 3 && num != 4 && num != 7 && num != 8 && num != 9);
  digitalWrite(pinC, num != 0 && num != 1 && num != 3 && num != 4 && num != 5 && num != 6 && num != 7 && num != 8 && num != 9);
  digitalWrite(pinD, num != 2 && num != 3 && num != 4 && num != 5 && num != 6 && num != 8 && num != 9);
  digitalWrite(pinE, num != 0 && num != 2 && num != 6 && num != 8);
  digitalWrite(pinF, num != 0 && num != 4 && num != 5 && num != 6 && num != 8 && num != 9);
  digitalWrite(pinG, num != 0 && num != 2 && num != 3 && num != 5 && num != 6 && num != 8 && num != 9);
  //Activar el digito correspondiente
  digitalWrite(pinDigito, LOW);
  //Desactivar el resto de digitos
  if (pinDigito != pinT1) {
    digitalWrite(pinT1, HIGH);
  } if (pinDigito != pinT2) {
    digitalWrite(pinT2, HIGH);
  } if (pinDigito != pinT3) {
    digitalWrite(pinT3, HIGH);
  }
}