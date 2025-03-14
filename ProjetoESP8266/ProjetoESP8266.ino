// Importa as bibliotecas especificas para o ESP-01S (baseado no ESP8266)
#include <ESP8266WiFi.h> // Biblioteca para gerenciar a conexão Wi-Fi no ESP8266
#include <ESP8266HTTPClient.h> // Biblioteca para realizar requisições HTTP no ESP8266

// Credenciais da rede Wi-Fi
const char* ssid = "linksys"; // Nome (SSID) da rede Wi-Fi
const char* password = ""; // Senha da rede Wi-Fi (em redes abertas, deixe vazio)
// Sua chave de escrita do ThingSpeak (deve ser gerada em sua conta ThingSpeak)
const char* apikey = "E4381YB3TM3DY9QF";

void setup() {
  // Inicializa digital pin LED_BUILTIN como output
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Inicializa a comunicação serial com velocidade de 115200 bps (bits por segundo)
  Serial.begin(9600);
  
  // Inicia a conexão com o Wi-Fi usando as credenciais fornecidas
  WiFi.begin(ssid, password);
  
  Serial.println("Conectando ao Wi-Fi...");
  // Aguarda até que a conexão com o Wi-Fi seja estabelecida
  while (WiFi.status() != WL_CONNECTED) {
    delay(10000); // Espera 10 segundos antes de verificar novamente
    Serial.print("."); // Imprime um ponto na saída serial a cada tentativa
  }
  
  // Quando conectado, imprime a mensagem de sucesso
  Serial.println("Wi-Fi conectado!");
}

void loop() {
  // Verifica se o dispositivo ainda está conectado ao Wi-Fi
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client; // Cria um objeto WiFiClient necessário para conexões HTTP no ESP8266
    HTTPClient http; // Cria um objeto HTTPClient para enviar requisições HTTP

    // Simula a leitura de temperatura (gera um número aleatório entre 20.0°C e 30.0°C)
    float temperatura = random(200, 300) / 10.0;

    // Monta a URL para enviar dados ao ThingSpeak
    String url = "http://api.thingspeak.com/update?api_key=";
    url += apikey; // Adiciona a chave de API à URL
    url += "&field1=" + String(temperatura); // Adiciona o valor de temperatura ao campo "field1"
    
    // Imprime a URL gerada no monitor serial (útil para depuração)
    Serial.println("Enviando: " + url);
    
    // Inicia a requisição HTTP usando o objeto "client" e a URL gerada
    http.begin(client, url);
    
    // Executa a requisição HTTP GET e armazena o código de resposta
    int httpResponseCode = http.GET();

    // Verifica se a requisição foi bem-sucedida (código 200 indica sucesso)
    if (httpResponseCode > 0) {
      Serial.println("Resposta do servidor: " + String(httpResponseCode));
    } else {
      // Em caso de erro, imprime a mensagem correspondente
      Serial.println("Erro ao enviar: " + http.errorToString(httpResponseCode));
    }

    // Finaliza a conexão HTTP (libera recursos)
    http.end();
    
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    
    // Aguarda 15 segundos antes de enviar novamente (limitação do plano gratuito do ThingSpeak)
    delay(15000);
  }
}