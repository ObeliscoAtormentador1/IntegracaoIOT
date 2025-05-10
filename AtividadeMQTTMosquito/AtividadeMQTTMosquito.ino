#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Prototipos de metodos que serao definidos mais adiante
// estabelece conexao com o broker e retorna situacao
bool connectMQTT(); 
// Recebe todas as publicacoes do topico informado no metodo e retorna
// via UART, aquilo que foi lido via subscricao.
void callback(char *topic, byte *payload, unsigned int length);

// Parametros de conexao WiFi
const char* ssid = "linksys"; // REDE
const char* password = "";    // SENHA

// Parametros de conexao ao MQTT Broker
const char* mqtt_broker = "test.mosquitto.org"; // host do broker
const char* topic = "MeuTopico/teste_topico";   // topico para subscricao e publicacao
const char* mqtt_username = "";  // Usuario nao necessario no host test.mosquitto.org
const char* mqtt_password = "";  // Senha Usuario nao necessario no host test.mosquitto.org
const int mqtt_port = 1883;      // Porta

// Variavel de status de conexao MQTT
bool mqttStatus = 0;

//Objetos
WiFiClient espClient; // objeto responsavel pela conexao WiFi
PubSubClient client(espClient); // Objeto responsavel pela conexao com broker mosquitto

// Definicao do Setup
void setup(void)
{
  Serial.begin(9600); // inicia conexao com monitor serial
  Serial.print("\n\n>>> Iniciando Conexao...\n"); // Avisa inicio de conexao
  WiFi.begin(ssid, password); // Inicia conexao com WiFi
  // Inicia looping de tentativa de conexao ao servico de WiFi
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n\nWiFi conectada!!!\n");
  Serial.print(WiFi.localIP()); // Envia IP atraves da UART
  mqttStatus = connectMQTT(); // Chama conexao MQTT com broker e retorna status
}

// Definicao do looping do ESP
void loop()
{
  static long pooling = 0; // define intervalo de pooling
  if (mqttStatus) { // Verifica se houve conexao
    client.loop();

    if (millis() - pooling > 5000) { // a cada periodo de 5 segundos publica info
      pooling = millis();
      client.publish(topic, "teste123,113007042022");
    }
  }
}

// Implementacao do metodo de conexao com o broker
bool connectMQTT() {
  byte tentativa = 0; // variavel byte que contara o numero de tentativas de conexao
  client.setServer(mqtt_broker, mqtt_port); // Chama metodo setServer passando url e porta
  client.setCallback(callback); // Informa ao objeto client qual metodo deve ser chamado quando houver
                                // alguma mensagem no topico subscrito.

  do {
    // Define o ID do cliente (a propria placa ESP)
    String client_id = "ESP-"; // Que usa o prefixo ESP-
    client_id += String(WiFi.macAddress()); // Concatenado com seu respectivo MAC address

    // Tenta estabelecer a conexao com o broker
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      // Com sucesso da conexao, informa os dados do cliente (a placa)
      Serial.println("Exito na conexao");
      Serial.print("Cliente se conectou ao broker\n");
      Serial.println(client_id.c_str());

      // Publica mensagem de sucesso
      client.publish(topic, "Teste123,113007042022"); // uma mensagem e publicada
      client.subscribe(topic); // Se inscreve no broker para receber mensagens
      return 1; // retorna 1 confirmando sucesso na conexao
    } else {
      // Informa falha na conexao e aguarda 2 segundos para nova tentativa
      Serial.println("Falha ao conectar:");
      Serial.print(client.state());
      Serial.print("Tentativa: ");
      Serial.println(tentativa);
      delay(2000);
    }
    tentativa++; // Incrementa numero de tentativas
  } while (!client.connected() && tentativa < 5); // Limita numero de tentativas

  if (tentativa < 5) {
    Serial.println("Conectado com sucesso");
    client.publish(topic, "Teste123,113007042022"); // uma mensagem é publicada
    client.subscribe(topic); // Se inscreve no broker para receber mensagens
    return 1; // retorna 1 confirmando sucesso na conexao
  } else {
    // caso contrario avisa falha e retorna 0
    Serial.println("Não conectado");
    return 0; // informa falha na conexao
  }
}

// Este metodo quando o client identifica nova mensagem no broker
void callback(char *topic, byte *payload, unsigned int length) {
  // char *topic identifica o topico registrado
  // byte *payload conjunto de bytes que foram publicados
  // int length é o tamanho do vetor de bytes do payload
  Serial.print("Mensagem chegou no topico: ");
  Serial.println(topic);
  Serial.print("Mensagem: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char) payload[i]);
  }
  Serial.println();
  Serial.println("--------------------------");
}
