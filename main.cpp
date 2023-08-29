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
#define pinA 27 //A
#define pinB 32 //B
#define pinC 21 //C
#define pinD 13 //D
#define pinE 4 //E
#define pinF 26 //F
#define pinG 16 //G
#define pinDP 17 //Punto
//Definicion Transistores
#define pinT1 33 //Transistor 1
#define pinT2 25 //Transistor 2
#define pinT3 14 //Transistor 3
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
AdafruitIO_Feed *tempCanal = io.feed("tempCelsius");
//****************************************************************
// Configuración
//****************************************************************
void setup(){
  //Definicion de Entradas
  pinMode(pinBoton, INPUT_PULLDOWN); //Entrada Boton
  pinMode(pinADC, INPUT); //Entrada ADC
  //Configurar PWM
  configurarPWM();
  //Posicionar Servo en posicion Inicial
  ledcWrite(servoChannel, 6);
  // Configurar los pines de Display y Transistores como salidas 
  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  pinMode(pinC, OUTPUT);
  pinMode(pinD, OUTPUT);
  pinMode(pinE, OUTPUT);
  pinMode(pinF, OUTPUT);
  pinMode(pinG, OUTPUT);
  pinMode(pinDP, OUTPUT);
  pinMode(pinT1, OUTPUT);
  pinMode(pinT2, OUTPUT);
  pinMode(pinT3, OUTPUT);
  //Inicializar los pines de segmentos apagados
  digitalWrite(pinA, HIGH);
  digitalWrite(pinB, HIGH);
  digitalWrite(pinC, HIGH);
  digitalWrite(pinD, HIGH);
  digitalWrite(pinE, HIGH);
  digitalWrite(pinF, HIGH);
  digitalWrite(pinG, HIGH);
  //Inicializar los transistores
  digitalWrite(pinT1, LOW);
  digitalWrite(pinT2, LOW);
  digitalWrite(pinT3, LOW); 
  //Monitor Serial
  Serial.begin(115200);
  //Configuracion Adafruit 
  while(! Serial);
  Serial.print("Connecting to Adafruit IO");
  io.connect();
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  } 
  Serial.println();
  Serial.println(io.statusText()); //Completar conexion 
}
//****************************************************************
// Loop Principal
//****************************************************************
void loop() {
  //io.run(); //Iniciar Adafruit
  estadoBoton = digitalRead(pinBoton); //Lectura de Boton
  //Antirebote de Boton
  if (estadoBoton == 1 && estadoAnteriorBoton == 0) {
    banderaLectura = true; //Activar Bandera
  }
  estadoAnteriorBoton = estadoBoton;

  if(banderaLectura){ //Cuando se active la Bandera
    semaforo(); //Ejecutar medicion y actualizar semaforo
    banderaLectura = false; //Desactivar Bandera
  } 
  //Mostrar la temperatura en los displays
  display(pinT1, decena); //Mostrar el primer digito
  delay(5); //Pequeña pausa 
  display(pinT2, unidad); //Mostrar el segundo digito
  delay(5); //Pequeña pausa
  display(pinT3, decimal); //Mostrar el tercer digito
  delay(5); //Pequeña pausa
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
// Funcion Semaforo
// ****************************************************************************
void semaforo(void){
  io.run(); //Iniciar Adafruit
  Serial.println("LECTURA DE TEMPERATURA");
  mediaMovil();
  tempCelsius = ((tempAnalogicoFiltrado*5000.0)/4095)/10.0;
  Serial.println(tempAnalogicoFiltrado);
  Serial.println(tempCelsius);
  //Limites de Temperatura y acciones a realizar en LED y SERVO
  if(tempCelsius <= 37.0){
    Serial.println("TEMPERATURA NORMAL");
    ledcWrite(servoChannel, map(tempCelsius*10, 20, 370, 6, 14)); //6
    ledcWrite(ledRChannel, 0);
    ledcWrite(ledGChannel, map(tempCelsius*10, 20, 370, 10, 60));
    ledcWrite(ledBChannel, 0);
  } if(37.0 < tempCelsius && tempCelsius < 37.5){
    Serial.println("TEMPERATURA MODERADA");
    ledcWrite(servoChannel, map(tempCelsius*10, 371, 374, 15, 23)); //20
    ledcWrite(ledRChannel, map(tempCelsius*10, 371, 374, 10, 60));
    ledcWrite(ledGChannel, map(tempCelsius*10, 371, 374, 10, 60));
    ledcWrite(ledBChannel, 0);
  } if(tempCelsius >= 37.5){
    Serial.println("TEMPERATURA ALTA");
    ledcWrite(servoChannel, map(tempCelsius*10, 375, 1500, 24, 33));  //33
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
  // Manejar el punto decimal en el segundo display
  if (pinDigito == pinT2) { //Cuando este encendido el Digito 2
    digitalWrite(pinDP, LOW); // Encender el punto decimal
  } else {                    // En cualquier otro caso
    digitalWrite(pinDP, HIGH); // Apagar el punto decimal
  }
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