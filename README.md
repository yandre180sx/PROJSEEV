/*
Nome ALUNO A- André Ribeiro
Nome ALUNO B- Pedro Borja
IPLEIRIA - Instituto Politécnico de Leiria
ESTG - Escola Superior de Tecnologia e Gestão
LEAU - Licenciatura em Engenharia Automóvel
SEEV - Sistemas Elétricos e Eletrónicos de Veículos
/*

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

const int trigPin = 12; //Pin TRIG sensor ultra-som
const int echoPin = 14; //Pin Echo sensor ultra-som
const int buzzerPin = 5; //Buzzer
const int motorPin = 13; //Motor
const int buttonPin = 0; //Botão esquerdo ESP32
const int ledRedPin = 15; //Led vermelho
const int ledGreenPin = 2; //Led verde
const int ledBluePin = 4; //Led azul

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

QueueHandle_t xQueueDistance;
SemaphoreHandle_t xSemaphoreUpdate;
SemaphoreHandle_t xSemaphoreLEDs;
SemaphoreHandle_t xSemaphoreMotor;
SemaphoreHandle_t xSemaphoreDistMin;
SemaphoreHandle_t xSemaphoreBuzzer;
SemaphoreHandle_t xSemaphoreButton;


int distancia_min = 10; //Distância mínima (Perigo)
int dist = distancia_min + 0.5 * distancia_min; //Distância advertência 

volatile int buttonPressCount = 0; //Contador cliques botão 

volatile bool buttonPressed = false;

portMUX_TYPE xButtonMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR handleButton() { 	//Interrupção do botão
  portENTER_CRITICAL_ISR(&xButtonMux);
  buttonPressed = true;
  portEXIT_CRITICAL_ISR(&xButtonMux);
}

void vTask_Button(void *pvParameters) { //Tarefa que gere o botão 
  (void)pvParameters;
  int buttonPressCount = 0;

  for (;;) {
    portENTER_CRITICAL_ISR(&xButtonMux);
    if (buttonPressed) {
      buttonPressed = false;
      portEXIT_CRITICAL_ISR(&xButtonMux);

      delay(50);
      if (digitalRead(buttonPin) == LOW) {
        buttonPressCount++;

        portENTER_CRITICAL(&xButtonMux);
        if (buttonPressCount == 1) { 	//Clique nº1
          if (xSemaphoreTake(xSemaphoreDistMin, portMAX_DELAY) == pdTRUE) {
            distancia_min += 5;
            dist += 5;
            xSemaphoreGive(xSemaphoreDistMin);
          }
        } else if (buttonPressCount == 2) { 	//Clique nº2
          if (xSemaphoreTake(xSemaphoreDistMin, portMAX_DELAY) == pdTRUE) {
            distancia_min += 10;
            dist += 10;
            xSemaphoreGive(xSemaphoreDistMin);
          }
        } else if (buttonPressCount == 3) { 	//clique nº3
          if (xSemaphoreTake(xSemaphoreDistMin, portMAX_DELAY) == pdTRUE) {
            distancia_min = 10;
            dist = distancia_min + 0.5 * distancia_min;
            buttonPressCount = 0;
            xSemaphoreGive(xSemaphoreDistMin);
          }
        }
        portEXIT_CRITICAL(&xButtonMux);
      }
    } else {
      portEXIT_CRITICAL_ISR(&xButtonMux);
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void IRAM_ATTR handleEcho() {	//Interrupção Echo; calcula a distância
  static unsigned long startTime;
  static bool firstTime = true;

  if (firstTime) {
    startTime = micros();
    firstTime = false;
  } else {
    long duration = micros() - startTime;
    long distanceValue = (duration * 0.0343) / 2;

    xQueueSendFromISR(xQueueDistance, &distanceValue, NULL);

    firstTime = true;
    portYIELD_FROM_ISR();
  }
}

void vTask_MeasureDistance(void *pvParameters) { //Tarefa que mede a distância 
  (void)pvParameters;

  for (;;) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void vTask_HandleDistance(void *pvParameters) {		//Gestão dos atuadores em função da distância medida
  (void)pvParameters;
  bool motorState = false;

  for (;;) {
    long distanceValue;

    if (xQueueReceive(xQueueDistance, &distanceValue, portMAX_DELAY) == pdTRUE) {
      if (xSemaphoreTake(xSemaphoreUpdate, portMAX_DELAY) == pdTRUE) {
        Serial.print("Distancia cm: \n");
        Serial.print(distanceValue);
        Serial.println("Valor dist: \n");
        Serial.print(dist);
        Serial.print("Distancia min: \n");
        Serial.print(distancia_min);

        if (distanceValue < distancia_min) {	//Distância Perigo 
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_ncenB08_tr);
          u8g2.setCursor(0, 15);
          u8g2.print("Perigo!");
          u8g2.setCursor(0, 30);
          u8g2.print("Distancia: ");
          u8g2.setCursor(0, 45);
          u8g2.print(distanceValue);
          u8g2.print(" cm");
          u8g2.setCursor(0, 60);
          u8g2.print("Estado Motor: OFF");
          u8g2.sendBuffer();

          if (xSemaphoreTake(xSemaphoreBuzzer, portMAX_DELAY) == pdTRUE) {
            if (distanceValue > distancia_min || distanceValue < dist) {	//Distância Advertência
              tone(buzzerPin, 400, 250);
              delay(250);
            } else if (distanceValue >= dist) {
              noTone(buzzerPin);
            } else if (distanceValue >= (distancia_min / 2)) {
              tone(buzzerPin, 800, 125);
              delay(125);
            } else {
              tone(buzzerPin, 400, 250);
              delay(250);
            }
            xSemaphoreGive(xSemaphoreBuzzer);
          }

          if (xSemaphoreTake(xSemaphoreMotor, portMAX_DELAY) == pdTRUE) {
            digitalWrite(motorPin, LOW);
            motorState = false;
            xSemaphoreGive(xSemaphoreMotor);
          }
        } else {
          u8g2.clearBuffer();
          u8g2.sendBuffer();

          if (xSemaphoreTake(xSemaphoreMotor, portMAX_DELAY) == pdTRUE) { //Controlar motor
            digitalWrite(motorPin, HIGH);
            motorState = true;
            u8g2.clearBuffer();
            u8g2.setFont(u8g2_font_ncenB08_tr);
            u8g2.setCursor(0, 15);
            u8g2.print("Estado Motor: ON");
            u8g2.sendBuffer();
            xSemaphoreGive(xSemaphoreMotor);
          }
        }

        xSemaphoreGive(xSemaphoreUpdate);

        if (xSemaphoreTake(xSemaphoreLEDs, portMAX_DELAY) == pdTRUE) { 	//Atuação LEDS
          if (distanceValue >= distancia_min && distanceValue < dist) {
            analogWrite(ledRedPin, 0);
            analogWrite(ledGreenPin, 0);
            analogWrite(ledBluePin, 255);
            tone(buzzerPin, 200, 500);
            delay(500);
          } else if (distanceValue >= dist) {
            analogWrite(ledRedPin, 0);
            analogWrite(ledGreenPin, 255);
            analogWrite(ledBluePin, 0);
          } else {
            analogWrite(ledRedPin, 255);
            analogWrite(ledGreenPin, 0);
            analogWrite(ledBluePin, 0);
          }

          xSemaphoreGive(xSemaphoreLEDs);
        }
      }
    }
  }
}



void vTask_UpdateDistMin(void *pvParameters) {
  (void)pvParameters;

  for (;;) {
    if (xSemaphoreTake(xSemaphoreDistMin, portMAX_DELAY) == pdTRUE) {
      /*u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.setCursor(0, 60);
      u8g2.print("Dist Min: ");
      u8g2.print(distancia_min);
      u8g2.sendBuffer();*/
      xSemaphoreGive(xSemaphoreDistMin);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup() {
  Serial.begin(115200);
  u8g2.begin();

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledRedPin, OUTPUT);
  pinMode(ledGreenPin, OUTPUT);
  pinMode(ledBluePin, OUTPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(echoPin), handleEcho, CHANGE);
  attachInterrupt(digitalPinToInterrupt(buttonPin), handleButton, FALLING);

  xQueueDistance = xQueueCreate(1, sizeof(long));
  xSemaphoreUpdate = xSemaphoreCreateMutex();
  xSemaphoreLEDs = xSemaphoreCreateCounting(1, 1);
  xSemaphoreMotor = xSemaphoreCreateMutex();
  xSemaphoreDistMin = xSemaphoreCreateMutex();
  xSemaphoreBuzzer = xSemaphoreCreateMutex();
  xSemaphoreButton = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(vTask_MeasureDistance, "Task_MeasureDistance", 4096, NULL, 4, NULL, 1); //prioridade 4
  xTaskCreatePinnedToCore(vTask_HandleDistance, "Task_HandleDistance", 4096, NULL, 3, NULL, 1); //prioridade 3
  xTaskCreatePinnedToCore(vTask_UpdateDistMin, "Task_UpdateDistMin", 4096, NULL, 2, NULL, 1); //prioridade 2
  xTaskCreatePinnedToCore(vTask_Button, "Task_Button", 4096, NULL, 1, NULL, 1); //prioridade 1
}

void loop() {
  // O loop principal não precisa de código
}
