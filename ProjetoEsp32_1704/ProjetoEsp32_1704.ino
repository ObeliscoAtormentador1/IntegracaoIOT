#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "MFIoT";
const char* password = "12345678";

const char* mqtt_broker = "192.168.137.1";
const char* topicPubTemp = "GRP7/TEMPERATURA";
const char* topicPubNivel = "GRP7/NIVEL";

const char* topicSubGrp1Temp = "GRP1/TEMP";
const char* topicSubGrp1Nivel = "GRP1/NIVEL";  
const char* topicSubGrp3Temp = "GRP3/TEMP";
const char* topicSubGrp3Nivel = "GRP3/NIVEL";

const char* mqtt_username = "";
const char* mqtt_password = "";
const int mqtt_port = 1883;  

bool mqttStatus = 0;
float Temp = 0.0;
float Nivel = 0.0;
String MsgTemp;
String MsgNivel;

float TempPlac = 0.0;
float NivelPlacl = 0.0;
float TempPlac2 = 0.0;
float NivelPlac2 = 0.0;

float TempMedia = 0.0;
float NivelMedio = 0.0;

WiFiClient espClient;
PubSubClient client(espClient); 

void setupWiFi();
bool connectMQTT();
void bombaFluxo(bool estado);
float obterTemperaturaCelsius();
float medirDistanciaCM();
void callback(char* topic, byte* payload, unsigned int length);

bool connectMQTT() {
  byte tentativa = 0;
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);

  do {
    String client_id = "ESP-";
    client_id += String(WiFi.macAddress());  

    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Exito na conexão:");
      Serial.print("Cliente conectado ao broker: ");
      Serial.println(client_id);
    } else {
      Serial.print("Falha ao conectar: ");
      Serial.print(client.state());
      Serial.println();
      Serial.print("Tentativas: ");
      Serial.println(tentativa);
      delay(2000);
    }
    tentativa++;
  } while (!client.connected() && tentativa < 5);

  if (tentativa < 5) {
    client.subscribe(topicPubTemp);
    client.subscribe(topicPubNivel);
    client.subscribe(topicSubGrp3Temp);
    client.subscribe(topicSubGrp3Nivel);
    client.subscribe(topicSubGrp1Temp);  
    client.subscribe(topicSubGrp1Nivel); 
    return true;
  } else {
    Serial.println("MQTT Não conectado");
    return false;
  }
}

void setup() {
  Serial.begin(9600);
  setupWiFi();
  mqttStatus = connectMQTT();
}

void loop() {
  static long long pooling = 0;
  if (mqttStatus) {
    client.loop();
    if (millis() > pooling + 5000) {
      pooling = millis();
      Temp = random(0, 1000) / 10.0;
      Nivel = random(0, 3000) / 10.0;
      MsgTemp = String(Temp);
      MsgNivel = String(Nivel);
      client.publish(topicPubTemp, MsgTemp.c_str());
      client.publish(topicPubNivel, MsgNivel.c_str());
      Serial.println("Mensagens publicadas.");
    }
  }
}

void setupWiFi() {
  delay(10);
  Serial.print("Conectando ao wi-fi...");
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    attempts++;
    if (attempts > 20) {
      Serial.println("\nFalha ao conectar no Wi-Fi! Reiniciando...");
      ESP.restart();
    }
  }
  Serial.println("\nWi-fi conectado!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  Serial.print("Mensagem recebida em ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(msg);
}
