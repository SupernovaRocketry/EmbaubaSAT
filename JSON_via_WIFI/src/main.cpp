#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <Wire.h>
#include <HTTPClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>

//Inicializando o objeto do sensor BMP280 e MPU6050
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;

//Definindo as informações da rede Wi-Fi
const char* ssid = "nome";
const char* password = "senha";

// Definindo as informações da servidor HTTP
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

  //Inicializando o sensor BMP280
  if (!bmp.begin(0x76)) {
    Serial.println("Erro ao iniciar o sensor BMP280");
    while (1);
  }
}

void loop() {
  //Leitura dos valores dos sensores
  float temperatura = bmp.readTemperature();
  float pressao = bmp.readPressure() / 100.0; // Conversão para hPa
  float altitude = bmp.readAltitude(1013.25); // Ajuste de pressão ao nível do mar

  //Leitura do MPU
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //Sensores que não fazem parte deste teste
  int experimento0 = 250;
  int experimento1 = 2;
  int bateria = 86;

  //String jsonString = "{";
  //jsonString += "\"equipe\": 5242,";
  //jsonString += "\"bateria\":" + String(bateria)+ ",";
  //jsonString += "\"temperatura\":" + String(temperatura) + ",";
  //jsonString += "\"pressao\":" + String(pressao) + ",";
  //jsonString += "\"giroscopio\": [" + String(g.gyro.x) + "," + String(g.gyro.y) + "," + String(g.gyro.z) + "],";
  //jsonString += "\"acelerometro\":[" + String(a.acceleration.x) + "," + String(a.acceleration.y) + "," + String(a.acceleration.z) + "],";
  //jsonString += "\"payload\": [" + String(experimento0) + "," + String(experimento1) + "]";
  //jsonString += "}";

  StaticJsonDocument<240> jsonBuffer; //Cada par de valores utiliza aproximadamente 16 bytes
                                      //Cada par nome-vetor utiliza aproximadamente 16*(1+N) bytes, em que N é o comprimento do vetor 
  //Criando um objeto JsonObject para armazenar os valores dos sensores
  JsonObject sensores = jsonBuffer.to<JsonObject>();

  //Adicionando os valores dos sensores ao JsonObject
  sensores["equipe"] = 5242;
  sensores["bateria"] = bateria;
  sensores["temperatura"] = temperatura;
  sensores["pressao"] = pressao;
  //sensores["altitude"] = altitude;
  sensores["giroscopio"][0] = g.gyro.x;
  sensores["giroscopio"][1] = g.gyro.y;
  sensores["giroscopio"][2] = g.gyro.z;
  sensores["acelerometro"][0] = a.acceleration.x;
  sensores["acelerometro"][1] = a.acceleration.y;
  sensores["acelerometro"][2] = a.acceleration.z;
  sensores["payload"][0] = experimento0;
  sensores["payload"][1] = experimento1;

  //Convertendo o JsonDocument em uma string JSON
  String jsonString;
  serializeJson(jsonBuffer, jsonString);

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