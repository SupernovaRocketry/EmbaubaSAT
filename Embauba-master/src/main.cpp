#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <Adafruit_INA219.h> 
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHTX0.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <math.h>
#include <HTTPClient.h>
#include <SPI.h>
#include <SD.h>
// --------------------------------------


#define SAMPLE_TIME 2000

Adafruit_INA219 ina219_0 (0x68);
TinyGPSPlus gps;
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
Adafruit_AHTX0 aht;
Adafruit_Sensor *aht_humidity, *aht_temp;

float current = 0;
float power = 0;
float lat = 0;
float lon = 0;
float temperatura_bmp = 0;
float pressao_bmp = 0;
float altitude_bmp = 0;
double ozonio = 0;
double carbono = 0;
float accX = 0, accY = 0, accZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
sensors_event_t humidity;
sensors_event_t temp;

//Definição dos parâmetros do sensor de O3
#define RL_o3 10     // Resistência ao lado do DOUT_LED
#define APin_o3 34   // Pino analógico utilizado
float curve_o3[2] = {-0.32372, 0.648};  // Curva do gráfico em log do MQ131 para O3 (a, b)

//Definição dos parâmetros do sensor de CO2
#define RL_co2 10     // Resistência ao lado do DOUT_LED
#define APin_co2 33   // Pino analógico utilizado

float curve_co2[2] = {-0.32372, 0.648};  // Curva do gráfico em log do MQ135 para CO2 (a, b)

float R0_co2 = 0;
float R0_o3 = 0;

File myFile;
const int CS = 15;
String jsonStr;
// -------------- Configuracoes WiFi -----------------------
//Definindo as informações da rede Wi-Fi
const char* ssid = "Supernova Rocketry"; //Define o nome do ponto de acesso
const char* password = "foguetaos2"; //Define a senha
// Definindo as informações da servidor HTTP
const char* serverAddress = "https://obsat.org.br";
const char* endpoint = "/teste_post/envio.php";


// ------------------ Calibracao ----------------------------
void calibracao_co2(){
  int cont;
  float val=0;
  // Calcula o valor de RS no cenário de ar limpo 50 vezes e pega a média
  for (cont=0;cont<50;cont++) {
                val += ((float)RL_co2 * (4095-analogRead(APin_co2)) / analogRead(APin_co2));
                delay(500);
  }
  val = val/50;                                                                                        
  R0_co2 = val;
}

void calibracao_o3(){
  int cont;
  float val=0;
  // Calcula o valor de RS no cenário de ar limpo 50 vezes e pega a média
  for (cont=0;cont<50;cont++) {
                val += ((float)RL_o3 * (4095-analogRead(APin_o3)) / analogRead(APin_o3));
                delay(500);
  }
  val = val/50;                                                                                        
  R0_o3 = val;
}
// ----------------------------------------------------------

// ------------------------ Leituras ------------------------

void readINA(){
    current = ina219_0.getCurrent_mA(); /* comando para chamar a corrente */
    power = ina219_0.getPower_mW(); /*comando para chamar a potência */

    Serial.print("Corrente: ");
    Serial.print(current); 
    Serial.println(" mA"); /*printa a corrente */
    Serial.print("Potência: "); 
    Serial.print(power); 
    Serial.println(" mW"); /* printa a potência */
}

void readGPS()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid()){
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }
}

void readBMP(){
    temperatura_bmp = bmp.readTemperature();
    pressao_bmp = bmp.readPressure();
    altitude_bmp = bmp.readAltitude(1013.25);
}

void readMQ131(){
    double ADCread=0;
    double RS, RSR0, Y, X;

    //5 Leituras e tira a media
    for (int count=0;count<5;count++) {
                    ADCread += analogRead(APin_o3);
                    delay(50);
    }
    ADCread = ADCread/5;

    //Calcula RS
    RS = (float)RL_o3 * (4095-ADCread) / ADCread;

    //Calcula RS/R0
    RSR0 = RS/R0_o3;

    //Tira o Log de RSR0 para utilizar na curva log-log (Y)
    Y = log10(RSR0);

    //Calcula o X
    X = (Y - curve_o3[1])/curve_o3[0];

    ozonio =  pow10(X);
}

void readMQ135(){
    double ADCread=0;
    double RS, RSR0, Y, X;

    //5 Leituras e tira a media
    for (int count=0;count<5;count++) {
                    ADCread += analogRead(APin_co2);
                    delay(50);
    }
    ADCread = ADCread/5;

    //Calcula RS
    RS = (float)RL_co2 * (4095-ADCread) / ADCread;

    //Calcula RS/R0
    RSR0 = RS/R0_co2;

    //Tira o Log de RSR0 para utilizar na curva log-log (Y)
    Y = log10(RSR0);

    //Calcula o X
    X = (Y - curve_co2[1])/curve_co2[0];

    //Retorna 10^X = PPM
    carbono = pow10(X);
    Serial.print("Carbono: ");
    Serial.println(carbono);
}

void readMPU(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;
}

void readAHT(){
  // Serial.print("AHT temperatura: ");
  // Serial.print(aht_temp);
  // Serial.print("\nAHT Umidade: ");
  // Serial.print(aht_humidity);
  aht_humidity->getEvent(&humidity);
  aht_temp->getEvent(&temp);
}

String getCurrentTime() {
      unsigned long currentTime = millis(); // Get the current time
      unsigned long hours = (currentTime / 3600000) % 24, minutes = (currentTime / 60000) % 60, seconds = (currentTime / 1000) % 60;
      String timeString = String(hours) + ":" + String(minutes) + ":" + String(seconds); // Create the time string
      return timeString;
    }

void saveSD(String jsonString){
  Serial.println("Salvando no SD...");
  // Adicionar string de tempo
  
  String timeString = getCurrentTime();
  // Converte a jsonStr para const char*
  const char* jsonCStr = jsonString.c_str();
  const char* timeCStr = timeString.c_str();
  // adiciona a string JSON ao arquivo

  // Abre arquivo no cartão no formato de lista
  File dataFile = SD.open("/data.json", FILE_APPEND);

  // Adiciona uma string JSON ao cartão SD
  if (dataFile) {
    dataFile.println(timeCStr);
    dataFile.println(jsonCStr);
    dataFile.close();
  } else {
    Serial.println("Erro ao abrir arquivo para adicionar JSON!");
  }
}

String JSON(){
  // StaticJsonDocument<240> jsonBuffer;       
  //Criando um objeto JsonObject para armazenar os valores dos sensores
  // JsonObject sensores = jsonBuffer.to<JsonObject>();
  //DynamicJsonDocument sensores(256);
  StaticJsonDocument<256> jsonBuffer; //Cada par de valores utiliza aproximadamente 16 bytes
                                      //Cada par nome-vetor utiliza aproximadamente 16*(1+N) bytes, em que N é o comprimento do vetor 
  JsonObject sensores = jsonBuffer.to<JsonObject>();
  //Adicionando os valores dos sensores ao JsonObject
  sensores["equipe"] = "1921";
  sensores["bateria"] = current;
  sensores["temperatura"] = temperatura_bmp;
  sensores["pressao"] = pressao_bmp;
  sensores["giroscopio"][0] = gyroX;
  sensores["giroscopio"][1] = gyroY;
  sensores["giroscopio"][2] = gyroZ;
  sensores["acelerometro"][0] = accX;
  sensores["acelerometro"][1] = accY;
  sensores["acelerometro"][2] = accZ;
  sensores["payload"][0] = ozonio;
  sensores["payload"][1] = carbono;
  // sensores["alt"] = altitude_bmp;
  //Convertendo o JsonDocument em uma string JSON
  String jsonString;
  serializeJson(sensores, jsonString);
  // Imprimindo a string JSON no monitor serial
  Serial.println(jsonString);
  return jsonString;
}

void HTTPsend(String jsonString){
  
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
}


void setup (){

    Serial.begin(115200);
    Serial.println("Hello, world!");

    Serial.println();
    Serial.print("Conectando-se a ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
  
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("Conectado!");

    // Inicializa o cartão SD
    if (!SD.begin(CS)) {
      Serial.println("Inicialização do cartão SD falhou!");
      return;
      }
    
    // Inicialização do INA
    if (! ina219_0.begin()) 
	  { 
		while (1) {
            Serial.println("Falha ao encontrar o INA219"); 
            delay(10); 
	    } 
    }
    Serial.println("INA inicializado...");

    // Inicializa o GPS
    Serial2.begin(115200);
    Serial.println("GPS inicializado..."); 

    // Inicializa o BMP
    if (!bmp.begin(0x76)) { /*Definindo o endereço I2C como 0x76. Mudar, se necessário, para (0x77)*/
    //Imprime mensagem de erro no caso de endereço invalido ou não localizado. Modifique o valor 
    Serial.println(F(" Não foi possível encontrar um sensor BMP280 válido, verifique a fiação ou "
                      "tente outro endereço!"));
    while (1) delay(10);
    }
    Serial.println("BMP inicializado...");

    //  ----------------------- Calibrando sensores ------------------------------------------
    Serial.println("Calibrando os sensores...");
    calibracao_co2();
    calibracao_o3();
    Serial.println("Sensores calibrados...");

    // Inicializando o MPU
    if (!mpu.begin(0x68)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
      }
    }
    Serial.println("MPU inicializado...");

    //setupt motion detection
    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(1);
    mpu.setMotionDetectionDuration(20);
    mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);

    //  Inicializando AHT
//      if (!aht.begin()) {
//       Serial.println("Failed to find AHT10/AHT20 chip");
//       while (1) {
//       delay(10);
//       }
//       // aht_temp = aht.getTemperatureSensor();
//       // Serial.print("AHT temperatura: ");
//       // aht_temp->printSensorDetails();
//       // Serial.print("\nAHT Umidade: ");
//       // aht_humidity = aht.getHumiditySensor();
//       // aht_temp->printSensorDetails();
// } 
//     Serial.println("AHT inicializado...");
  
  
  Serial.println("Todos os sensores inicializados. Codigo rodando...");   
}

void loop() {

    readINA();
    readGPS();
    readBMP();
    readMQ131();
    readMQ135();
    readMPU();
    // readAHT();
    jsonStr = JSON();
    saveSD(jsonStr);
    HTTPsend(jsonStr);
    delay(SAMPLE_TIME);
}