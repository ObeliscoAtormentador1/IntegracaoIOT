#include <OneWire.h>
#include <DallasTemperature.h>

const int pinoTemperatura = 21;
OneWire sensorOneWireBus(pinoTemperatura);
DallasTemperature sensors(&sensorOneWireBus);

const int trigPin = 5;
const int echoPin = 18;
#define SOUND_SPEED 0.034

long duration;
float distanceCm;

const bool ON = true;
const bool OFF = false;
const int releBomba = 4;

void setup() {
   pinMode(releBomba, OUTPUT);

  Serial.begin(9600);
  Serial.println("Inicializando sensor DS18B20...");
  sensors.begin();
  delay(500);
  Serial.print("Número de dispositivos encontrados: ");
  Serial.println(sensors.getDeviceCount());
  Serial.println("Sensor DS18B20 inicializado.");

  Serial.println("Inicializando sensor HC-SR04...");
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);
  Serial.println("Sensor HC-SR04 inicializado...");

}

void loop() {
bombaFluxo(ON);
delay(500);
bombaFluxo(OFF);
delay(500);

float temperaturaCelsius = obterTemperaturaCelsius();
Serial.print("Temperatura: ");
Serial.print(temperaturaCelsius);
Serial.println(" °C");

delay(1000);
float distanciaCM = medirDistanciaCM();
Serial.print("Distância: ");
Serial.print(distanciaCM);
Serial.println(" cm");

}

void bombaFluxo(bool estado)
{
  if(estado){
    digitalWrite(releBomba,HIGH);
  }
  else{
    digitalWrite(releBomba,LOW);
  }
}


float obterTemperaturaCelsius() {
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}

float medirDistanciaCM(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distanceCm = duration * SOUND_SPEED/2;
  return distanceCm;
}
