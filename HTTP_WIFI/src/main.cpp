#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <HTTPClient.h>

//Defina as informações da sua rede Wi-Fi
const char* ssid = "Supernova Rocketry";
const char* password = "foguetaos2";

// Defina as informações do seu servidor HTTP
const char* serverAddress = "https://obsat.org.br";
const char* endpoint = "/teste_post/envio.php";

void setup() {
  Serial.begin(9600);
  
  Serial.println();
  Serial.print("Conectando-se a ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Conectado!");

}

void loop() {
  
  String jsonString = "{";
  jsonString += "\"equipe\": 41,";
  jsonString += "\"bateria\": 86,";
  //jsonString += "\"temperatura\":" + String(temperatura) + ",";
  //jsonString += "\"pressao\":" + String(pressao) + ",";
  //jsonString += "\"giroscopio\": [" + String(g.gyro.x) + "," + String(g.gyro.y) + "," + String(g.gyro.z) + "],";
  //jsonString += "\"acelerometro\":[" + String(a.acceleration.x) + "," + String(a.acceleration.y) + "," + String(a.acceleration.z) + "],";
  //jsonString += "\"payload\": [3.1415, 12, 123, 543, \"string\"]";
  jsonString += "}";

  // Imprimindo a string JSON no monitor serial
  Serial.println(jsonString);
  // Criando um objeto HTTPClient
  HTTPClient httpClient;

  // Definindo o endpoint do servidor
  String url = String(serverAddress) + String(endpoint);

  // Configurando o objeto HTTPClient para enviar a solicitação POST
  httpClient.begin(url);
  httpClient.addHeader("Content-Type", "application/json");

  // Enviando a string JSON como corpo da solicitação POST
  int httpResponseCode = httpClient.POST(jsonString);

  // Obtendo a resposta do servidor
  if (httpResponseCode > 0) {
    Serial.print("Código de resposta: ");
    Serial.println(httpResponseCode);

    String response = httpClient.getString();
    Serial.print("Resposta do servidor: ");
    Serial.println(response);
  } else {
    Serial.print("Erro na solicitação. Código de erro: ");
    Serial.println(httpResponseCode);
  }

  // Liberando os recursos HTTP
  httpClient.end();

  //Atraso de 4 segundos
  delay(5000);
}