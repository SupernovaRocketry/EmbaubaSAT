#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>

//Inicializando o objeto do sensor BMP280 e MPU6050
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;

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

  //Leitura do MPU
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  //Criando um objeto JsonBuffer
  StaticJsonDocument<176> jsonBuffer; //Cada par de valores utiliza aproximadamente 16 bytes
                                      //Cada par nome-vetor utiliza aproximadamente 16*(1+N) bytes, em que N é o comprimento do vetor 
  //Criando um objeto JsonObject para armazenar os valores dos sensores
  JsonObject sensores = jsonBuffer.to<JsonObject>();

  //Adicionando os valores dos sensores ao JsonObject
  sensores["temperatura"] = temperatura;
  sensores["pressao"] = pressao;
  sensores["altitude"] = altitude;
  sensores["giroscopio"][0] = g.gyro.x;
  sensores["giroscopio"][1] = g.gyro.y;
  sensores["giroscopio"][2] = g.gyro.z;
  sensores["aceleracao"][0] = a.acceleration.x;
  sensores["aceleracao"][1] = a.acceleration.y;
  sensores["aceleracao"][2] = a.acceleration.z;

  //Convertendo o JsonDocument em uma string JSON
  String jsonStr;
  serializeJson(jsonBuffer, jsonStr);

  //Imprimindo a string JSON no monitor serial
  Serial.println(jsonStr);

  //Atraso de 4 segundos
  delay(4000);
}