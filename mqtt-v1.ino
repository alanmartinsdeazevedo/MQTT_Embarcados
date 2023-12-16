#include <Arduino.h>
#include <WiFi.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Rede Wifi
const char* ssid = "TADS";
const char* pw = "12345678";

// Valores para ativar/desativar IN1 do Driver
#define TEMP_HIGH 28.70
#define TEMP_LOW 28.30

// Pino do cooler
const int COOLER_PIN = 13;
const int COOLER_CONTROL = 14;
const int RPM_PIN = 34;

// Variáveis
int SPEED = 0;
DHT dht(33, DHT11);
WiFiClient espClient;
PubSubClient client(espClient);


volatile int rpmCount = 0;
unsigned long lastMillis;

SemaphoreHandle_t xSemaphore = NULL;

void setup_wifi();
void reconnect();
void leituraSensorTask(void *pvParameters);
void controleTemperaturaTask(void *pvParameters);
void rpmCounterTask(void *pvParameters);

void IRAM_ATTR handleRPMInterrupt() {
  rpmCount++;
}

void setup() {
  Serial.begin(115200);

  setup_wifi();

  client.setServer("ec2-18-100-42-114.eu-south-2.compute.amazonaws.com", 1883);

  dht.begin();

  pinMode(COOLER_PIN, OUTPUT);
  pinMode(COOLER_CONTROL, OUTPUT);
  pinMode(RPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), handleRPMInterrupt, FALLING);

  xSemaphore = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(
      leituraSensorTask,   // função a ser chamada
      "LeituraSensorTask", // nome da task
      10000,               // tamanho em bytes
      NULL,                // parâmetros a passar a função
      1,                   // Prioridade
      NULL,
      0
  );

  xTaskCreatePinnedToCore(
      controleTemperaturaTask,  // função a ser chamada
      "ControleTempTask",       // nome da task
      10000,                    // tamanho em bytes
      NULL,                     // parâmetros a passar a função
      2,                        // Prioridade
      NULL,                     // handle da task
      0
  );

  xTaskCreatePinnedToCore(
      rpmCounterTask,   // função a ser chamada
      "RPMCounterTask", // nome da task
      10000,            // tamanho em bytes
      NULL,             // parâmetros a passar a função
      1,                // Prioridade
      NULL,              // handle da task
      1
  );
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  vTaskDelay(100 / portTICK_PERIOD_MS); // Coloca task em blocked
                                        // Permite tarefas de menor prioridade serem executadas algum momento
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a rede ");
  Serial.println(ssid);

  WiFi.begin(ssid, pw);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi Conectado");
  Serial.println("IP: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop reconectar a rede
  while (!client.connected()) {
    Serial.print("Conectando com o Servidor MQTT ...");

    if (client.connect("ESP8266Client")) {
      Serial.println("Conectado");
      // Subscribe
      client.subscribe("botao");
    } else {
      Serial.print("Erro ao conectar, rc=");
      Serial.print(client.state());
      Serial.println(" tentando novamente em 5 segundos");
      // Espera 5 segundos
      delay(5000);
    }
  }
}

void leituraSensorTask(void *pvParameters) {
  while (1) {
    if (xSemaphore != NULL) {
      if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE) {
        // Leitura do sensor
        float temperaturaCelsius = dht.readTemperature();
        float umidade = dht.readHumidity();

        Serial.print("Temperatura: ");
        Serial.println(temperaturaCelsius);
        Serial.print("Umidade: ");
        Serial.println(umidade);

        // Envia para o MQTT
        char tempString[8];
        char umidadeString[8];
        int tempInt = (int)temperaturaCelsius;
        int umidadeInt = (int)umidade;
        itoa(tempInt, tempString, 10);
        itoa(umidadeInt, umidadeString, 10);

        if (client.connected()) {
          client.publish("temperatura", tempString);  // Tópico "temperatura"
          client.publish("umidade", umidadeString);    // Tópico "umidade"
        } else {
          Serial.println("Erro ao conectar com o servidor MQTT");
        }

        xSemaphoreGive(xSemaphore);
      }
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Atraso de 1 segundo
  }
}

void controleTemperaturaTask(void *pvParameters) {
  while (1) {
    float temperaturaCelsius = dht.readTemperature();

    // Controle temperatura
    if (temperaturaCelsius > TEMP_HIGH) {
      Serial.println("Cooler Ligado");
      digitalWrite(COOLER_PIN, HIGH); // Liga o cooler
      for (int sp=0; sp < 255; sp++){
        vTaskDelay(500 / portTICK_PERIOD_MS);
        SPEED = SPEED+5;
        analogWrite(COOLER_CONTROL, SPEED);
        Serial.println(SPEED);
      }
    } else if (temperaturaCelsius < TEMP_LOW) {
      Serial.println("Cooler Desligado");
      digitalWrite(COOLER_PIN, LOW); // Desliga o cooler
      SPEED = 0;
    }

    vTaskDelay(5000 / portTICK_PERIOD_MS); // Atraso de 5 segundos
  }
}

void rpmCounterTask(void *pvParameters) {
  lastMillis = millis();

  while (1) {
    if (millis() - lastMillis >= 1000) {
      // calcula o RPM
      noInterrupts();
      int rpm = (rpmCount / 2) * 60; // Divide por 2
      rpmCount = 0; // Reinicia o contador
      interrupts(); // Habilita interrupções novamente
      if (rpm <= 500 ){
        Serial.print("Atenção obstrução no Cooler");
      }

      lastMillis = millis();
    }

    vTaskDelay(100 / portTICK_PERIOD_MS); 
  }
}
