#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "STONE.h"


// Pin para el sensor DS18B20
#define ONE_WIRE_BUS 4


// Configuración de pines para relés y los sensores de caudal
#define RELAY1_PIN 18
#define RELAY2_PIN 19
#define SENSOR1_PIN 2
#define SENSOR2_PIN 3


// Variables para el conteo de pulsos y el flujo de ambos caudalímetros
volatile long pulseCount1 = 0;
volatile long pulseCount2 = 0;
unsigned long lastTime = 0;
float flowDivisor = 1.0;
float maxFlowrate = 5.5;
float flowrate1 = 0.0;
float flowrate2 = 0.0;
float totalFlowrate = 0.0;


// Variables para temporización de los relés
unsigned long relay1OnTime = 0;
unsigned long relay2OnTime = 0;
unsigned long relayDuration = 10000; // Duración en milisegundos (10 segundos)
const unsigned long extraTime = 5000; // Tiempo adicional en milisegundos (5 segundos)


// Configuración del bus OneWire y sensor DS18B20
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);


// Configuración de la pantalla STONE
STONE HMI;


void setup() {
  Serial.begin(115200);
  sensors.begin();
 
  // Configuración de pines
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(SENSOR1_PIN, INPUT);
  pinMode(SENSOR2_PIN, INPUT);


  // Configura interrupciones para los sensores de caudal
  attachInterrupt(digitalPinToInterrupt(SENSOR1_PIN), pulseCounter1, RISING);
  attachInterrupt(digitalPinToInterrupt(SENSOR2_PIN), pulseCounter2, RISING);


  // Llamar a la pantalla STONE
  HMI.begin(115200); 
}


void loop() {
  unsigned long currentMillis = millis();


  // Detectar toques en la pantalla
  HMI.listen();
 
  // Control de relés basado en los botones de la pantalla táctil
  if (HMI.isButtonTouched(1)) { // Botón 1 para RELAY1 (agua fría)
    digitalWrite(RELAY1_PIN, HIGH);
    relay1OnTime = currentMillis + relayDuration; 
  }
 
  if (HMI.isButtonTouched(2)) { // Botón 2 para RELAY2 (agua caliente)
    digitalWrite(RELAY2_PIN, HIGH);
    relay2OnTime = currentMillis + relayDuration; 
  }
 
  if (HMI.isButtonTouched(3)) { // Botón 3 para ambos relés (agua tibia)
    digitalWrite(RELAY1_PIN, HIGH);
    digitalWrite(RELAY2_PIN, HIGH);
    relay1OnTime = currentMillis + relayDuration;
    relay2OnTime = currentMillis + relayDuration;
  }


  // Botón 4: Apaga todas las electroválvulas inmediatamente
  if (HMI.isButtonTouched(4)) { 
    digitalWrite(RELAY1_PIN, LOW); // Apaga RELAY1
    digitalWrite(RELAY2_PIN, LOW); // Apaga RELAY2
    relay1OnTime = 0;
    relay2OnTime = 0; 
  }


  // Botón 5: Añadir 5 segundos de tiempo extra a los relés activos
  if (HMI.isButtonTouched(5)) { // Botón 5 para tiempo extra
    if (digitalRead(RELAY1_PIN) == HIGH) {
      relay1OnTime += extraTime; // Añade 5 segundos a RELAY1
    }
    if (digitalRead(RELAY2_PIN) == HIGH) {
      relay2OnTime += extraTime; // Añade 5 segundos a RELAY2
    }
  }


  // Apagado de relés después del tiempo configurado
  if (currentMillis >= relay1OnTime && relay1OnTime != 0) {
    digitalWrite(RELAY1_PIN, LOW);
  }


  if (currentMillis >= relay2OnTime && relay2OnTime != 0) {
    digitalWrite(RELAY2_PIN, LOW);
  }


  // Tiempo restante para los relés
  long relay1TimeLeft = (relay1OnTime > currentMillis) ? (relay1OnTime - currentMillis) / 1000 : 0;
  long relay2TimeLeft = (relay2OnTime > currentMillis) ? (relay2OnTime - currentMillis) / 1000 : 0;


  // Cálculo caudal
  if (millis() - lastTime > 1000) {
    noInterrupts();
    flowrate1 = (pulseCount1 / flowDivisor);
    flowrate2 = (pulseCount2 / flowDivisor);
    totalFlowrate = flowrate1 + flowrate2;
    pulseCount1 = 0;
    pulseCount2 = 0;
    lastTime = millis();
    interrupts();
    Serial.print("Pulse Count 1: ");
    Serial.println(pulseCount1);
    Serial.print("Pulse Count 2: ");
    Serial.println(pulseCount2);
    Serial.print("Flow Rate 1: ");
    Serial.println(flowrate1);
    Serial.print("Flow Rate 2: ");
    Serial.println(flowrate2);
    Serial.print("Total Flow Rate: ");
    Serial.println(totalFlowrate);
  }


  // Temperatura
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0);


  // Mostrar datos en la pantalla
  HMI.setText("t0", String(temperature) + " C"); // Muestra la temperatura en un componente de texto con id t0
  HMI.setText("t1", String(totalFlowrate) + " l/m");  // Muestra el caudal total en un componente de texto con id t1


  // Muestra el tiempo restante en la pantalla STONE
  HMI.setText("t2", "Tiempo Agua Fría: " + String(relay1TimeLeft) + " s"); // agua fría
  HMI.setText("t3", "Tiempo Agua Caliente " + String(relay2TimeLeft) + " s"); // agua caliente


  //Slider para el tiempo de activación de los relés
  relayDuration = HMI.getSliderValue("s0") * 1000;

  if (!isnan(temperature) && !isnan(totalFlowrate)) {
    Serial.print("Temperature (°C): ");
    Serial.println(temperature);
    Serial.print("Caudal Total (l/m): ");
    Serial.println(totalFlowrate);
    Serial.print("Relay Duration (ms): ");
    Serial.println(relayDuration);
    Serial.print("Relay 1 Time Left (s): ");
    Serial.println(relay1TimeLeft);
    Serial.print("Relay 2 Time Left (s): ");
    Serial.println(relay2TimeLeft);
  } else {
    Serial.println("Error reading from sensor");
  }
  // Delay lecturas
  delay(200);
}
// Caudalímetros
void pulseCounter1() {
  pulseCount1++;
}
void pulseCounter2() {
  pulseCount2++;
}
