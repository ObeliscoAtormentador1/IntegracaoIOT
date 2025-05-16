#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ThingSpeak.h>

// ==================== Configurações ====================
const char* ssid = "MFIoT";
const char* password = "12345678";

const char* mqtt_broker = "192.168.137.1";
const int mqtt_port = 1883;

const char* topicPubTemp = "GRP5/TEMPERATURA/";
const char* topicPubNivel = "GRP5/NIVEL/";

const char* topicSubGrp1Temp = "GRP1/TEMP/";
const char* topicSubGrp1Nivel = "GRP1/NIVEL/";
const char* topicSubGrp2Temp = "GRP2/TEMP/";
const char* topicSubGrp2Nivel = "GRP2/NIVEL/";
const char* topicSubGrp3Temp = "GRP3/TEMP/";
const char* topicSubGrp3Nivel = "GRP3/NIVEL/";

WiFiClient espClient;
PubSubClient client(espClient);

// ==================== Variáveis ====================
float Temp = 0.0, Nivel = 0.0;
String MsgTemp, MsgNivel;

float TempPlace1 = 0.0, NivPlace1 = 0.0;
float TempPlace2 = 0.0, NivPlace2 = 0.0;
float TempMedia = 0.0, NivMedio = 0.0;

// ==================== Sensor DS18B20 ====================
const int pinTemperatura = 21;
OneWire oneWire(pinTemperatura);
DallasTemperature sensors(&oneWire);

// ==================== Sensor HC-SR04 ====================
const int trigPin = 5;
const int echoPin = 18;
#define SOUND_SPEED 0.034

// ==================== Bomba ====================
const int releBomba = 4;
const bool ON = true;
const bool OFF = false;

// ==================== Setup ====================
void setup() {
  Serial.begin(9600);
  
  pinMode(releBomba, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.println("Iniciando sensor DS18B20...");
  sensors.begin();
  delay(500);

  if (sensors.getDeviceCount() == 0) {
    Serial.println("Nenhum sensor DS18B20 encontrado! Verifique o cabeamento e o resistor de pull-up.");
  } else {
    Serial.print("Número de sensores encontrados: ");
    Serial.println(sensors.getDeviceCount());
  }

  setupWiFi();
  client.setCallback(callback);
  connectMQTT();
}

// ==================== Loop ====================
void loop() {
  static unsigned long lastMillis = 0;

  // Controle da bomba
  bombaFluxo(ON);
  delay(500);
  bombaFluxo(OFF);
  delay(500);

  // Leitura dos sensores
  float temperaturaCelsius = obterTemperaturaCelsius();
  Serial.print("Temperatura: ");
  Serial.print(temperaturaCelsius);
  Serial.println(" °C");

  float distanciaCM = medirDistanciaCM();
  Serial.print("Distância: ");
  Serial.print(distanciaCM);
  Serial.println(" cm");

  // MQTT
  if (!client.connected()) {
    connectMQTT();
  }
  client.loop();

  if (millis() - lastMillis > 10000) {
    lastMillis = millis();

    Temp = temperaturaCelsius;
    Nivel = distanciaCM;

    MsgTemp = String(Temp, 2);  // com 2 casas decimais
    MsgNivel = String(Nivel, 2);

    client.publish(topicPubTemp, MsgTemp.c_str());
    client.publish(topicPubNivel, MsgNivel.c_str());

    Serial.println("📡 Temperatura e Nível publicados no MQTT.");
  }
}

// ==================== Funções Auxiliares ====================
void setupWiFi() {
  Serial.print("Conectando ao Wi-Fi...");
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (++attempts > 20) {
      Serial.println("\nFalha ao conectar. Reiniciando...");
      ESP.restart();
    }
  }
  Serial.println("\nWi-Fi conectado!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void bombaFluxo(bool estado) {
  digitalWrite(releBomba, estado ? HIGH : LOW);
}

float obterTemperaturaCelsius() {
  sensors.requestTemperatures();
  float temp = sensors.getTempCByIndex(0);
  if (temp == DEVICE_DISCONNECTED_C || temp == -127.0) {
    Serial.println("Erro ao ler temperatura do DS18B20.");
    return 0.0;  // valor de fallback
  }
  return temp;
}

float medirDistanciaCM() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);  // timeout de 30ms
  if (duration == 0) {
    Serial.println("Erro na leitura do HC-SR04.");
    return 0.0;
  }

  float distance = duration * SOUND_SPEED / 2;
  return distance;
}

bool connectMQTT() {
  byte tentativas = 0;
  client.setServer(mqtt_broker, mqtt_port);

  while (!client.connected() && tentativas < 5) {
    String client_id = "ESP32-" + String(WiFi.macAddress());
    if (client.connect(client_id.c_str())) {
      Serial.println(" onectado ao broker MQTT: " + client_id);
      client.subscribe(topicSubGrp1Temp);
      client.subscribe(topicSubGrp1Nivel);
      client.subscribe(topicSubGrp2Temp);
      client.subscribe(topicSubGrp2Nivel);
      client.subscribe(topicSubGrp3Temp);
      client.subscribe(topicSubGrp3Nivel);
      return true;
    } else {
      Serial.print("Tentando conexão MQTT. Estado: ");
      Serial.println(client.state());
      delay(2000);
      tentativas++;
    }
  }

  Serial.println("❌ Falha ao conectar ao MQTT.");
  return false;
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("📩 Mensagem recebida no tópico: ");
  Serial.println(topic);

  String mensagemString;
  for (unsigned int i = 0; i < length; i++) {
    mensagemString += (char)payload[i];
  }

  float valor = mensagemString.toFloat();

  if (strcmp(topic, topicSubGrp1Temp) == 0) {
    TempPlace1 = valor;
    Serial.println("Temp Placa 1: " + String(TempPlace1));
  } else if (strcmp(topic, topicSubGrp1Nivel) == 0) {
    NivPlace1 = valor;
    Serial.println("Nivel Placa 1: " + String(NivPlace1));
  } else if (strcmp(topic, topicSubGrp2Temp) == 0) {
    TempPlace2 = valor;
    Serial.println("Temp Placa 2: " + String(TempPlace2));
  } else if (strcmp(topic, topicSubGrp2Nivel) == 0) {
    NivPlace2 = valor;
    Serial.println("Nivel Placa 2: " + String(NivPlace2));
  }

  Serial.println("↳ Valor recebido: " + String(valor));
  Serial.println("-----------------------------------");
}
