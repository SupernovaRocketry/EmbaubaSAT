#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

//Inicializando o objeto do sensor BMP280
Adafruit_BMP280 bmp;

void setup() {
  Serial.begin(9600);
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

  //Criando um objeto JsonBuffer
  StaticJsonDocument<48> jsonBuffer; //Cada par de valores utiliza aproximadamente 16 bytes

  //Criando um objeto JsonObject para armazenar os valores dos sensores
  JsonObject sensores = jsonBuffer.to<JsonObject>();

  //Adicionando os valores dos sensores ao JsonObject
  sensores["temperatura"] = temperatura;
  sensores["pressao"] = pressao;
  sensores["altitude"] = altitude;

  //Convertendo o JsonDocument em uma string JSON
  String jsonString;
  serializeJson(jsonBuffer, jsonString);

  //Imprimindo a string JSON no monitor serial
  Serial.println(jsonString);

  //Atraso de 4 segundos
  delay(4000);
}