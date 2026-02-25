#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <LiquidCrystal_I2C.h>  

// Pines
#define LED_RED 2
#define LED_YELLOW 4
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Estructura de datos del sensor
typedef struct {
    float valor;
    uint32_t timestamp;
    char unidad[10];
} SensorData_t;

// Handles
TaskHandle_t SensorTask1Handle = NULL;
TaskHandle_t SensorTask2Handle = NULL;
TaskHandle_t ProcessTaskHandle = NULL;
TaskHandle_t DisplayTaskHandle = NULL;

QueueHandle_t queueTemp;
QueueHandle_t queueHum;
SemaphoreHandle_t semDisplayReady;
SemaphoreHandle_t semSystemReady;

// Variable globales
volatile float globalTempAvg = 0;
volatile float globalHumAvg = 0;
volatile float globalTempCurrent = 0;
volatile float globalHumCurrent = 0;
volatile bool dataReady = false;

// Configuración
#define QUEUE_SIZE 10
#define SAMPLE_INTERVAL_MS 2000
#define TEMP_THRESHOLD 30.0
#define HUM_THRESHOLD 70.0

// ========================================
// TAREA 1: Sensor de Temperatura
// ========================================
void SensorTask1(void *pvParameters) {
  SensorData_t dato;

  xSemaphoreTake(semSystemReady, portMAX_DELAY);
  xSemaphoreGive(semSystemReady);
  
  for(;;) {
    // Simular lectura de sensor (temperatura entre 15-35°C)
    dato.valor = random(150, 350) / 10.0;  // Genera 15.0 a 35.0
    dato.timestamp = millis();
    strcpy(dato.unidad, "°C");
    
    // Enviar a la queue
    if(xQueueSend(queueTemp, &dato, pdMS_TO_TICKS(100)) == pdTRUE) {
      Serial.print("[TEMP] Enviado: ");
      Serial.print(dato.valor);
      Serial.print(dato.unidad);
      Serial.print(" @ ");
      Serial.print(dato.timestamp);
      Serial.println(" ms");
      
      // Alerta si temperatura alta
      if(dato.valor > TEMP_THRESHOLD) {
        digitalWrite(LED_RED, HIGH);
        Serial.println("  ALERTA: Temperatura alta!");
      } else {
        digitalWrite(LED_RED, LOW);
      }
    } else {
      Serial.println("[TEMP]  Queue llena, dato descartado");
    }
    
    vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
  }
}

// ========================================
// TAREA 2: Sensor de Humedad
// ========================================
void SensorTask2(void *pvParameters) {
  SensorData_t dato;

  xSemaphoreTake(semSystemReady, portMAX_DELAY);
  xSemaphoreGive(semSystemReady);
  
  for(;;) {
    // Simular lectura de sensor (humedad entre 40-80%)
    dato.valor = random(400, 800) / 10.0;  // Genera 40.0 a 80.0
    dato.timestamp = millis();
    strcpy(dato.unidad, "%");
    
    // Enviar a la queue
    if(xQueueSend(queueHum, &dato, pdMS_TO_TICKS(100)) == pdTRUE) {
      Serial.print("[HUM]  Enviado: ");
      Serial.print(dato.valor);
      Serial.print(dato.unidad);
      Serial.print(" @ ");
      Serial.print(dato.timestamp);
      Serial.println(" ms");
      
      // Alerta si humedad alta
      if(dato.valor > HUM_THRESHOLD) {
        digitalWrite(LED_YELLOW, HIGH);
        Serial.println("  ALERTA: Humedad alta!");
      } else {
        digitalWrite(LED_YELLOW, LOW);
      }
    } else {
      Serial.println("[HUM]   Queue llena, dato descartado");
    }
    
    vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
  }
}

// ========================================
// TAREA 3: Procesamiento de Datos
// ========================================
void ProcessTask(void *pvParameters) {
  SensorData_t datoTemp;
  SensorData_t datoHum;
  
  float tempHistory[5] = {0};
  float humHistory[5] = {0};
  int tempIndex = 0;
  int humIndex = 0;
  int tempCount = 0;
  int humCount = 0;
  
  xSemaphoreTake(semSystemReady, portMAX_DELAY);
  xSemaphoreGive(semSystemReady);
  
  for(;;) {
    bool newDataAvailable = false;
    
    if(xQueueReceive(queueTemp, &datoTemp, pdMS_TO_TICKS(10)) == pdTRUE) {
      tempHistory[tempIndex] = datoTemp.valor;
      tempIndex = (tempIndex + 1) % 5;
      if(tempCount < 5) tempCount++;
      globalTempCurrent = datoTemp.valor;
      newDataAvailable = true;
    }
    
    if(xQueueReceive(queueHum, &datoHum, pdMS_TO_TICKS(10)) == pdTRUE) {
      humHistory[humIndex] = datoHum.valor;
      humIndex = (humIndex + 1) % 5;
      if(humCount < 5) humCount++;
      globalHumCurrent = datoHum.valor;
      newDataAvailable = true;
    }
    
    if(newDataAvailable) {
      // Calcular promedios
      float tempSum = 0;
      for(int i = 0; i < tempCount; i++) {
        tempSum += tempHistory[i];
      }
      globalTempAvg = tempSum / tempCount;
      
      float humSum = 0;
      for(int i = 0; i < humCount; i++) {
        humSum += humHistory[i];
      }
      globalHumAvg = humSum / humCount;
      dataReady = true;

      xSemaphoreGive(semDisplayReady);
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ========================================
// TAREA 4: Display/Presentación
// ========================================
void DisplayTask(void *pvParameters) {
  uint32_t displayCount = 0;
  
  xSemaphoreTake(semSystemReady, portMAX_DELAY);
  xSemaphoreGive(semSystemReady);

  // Mensaje inicial en LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sistema listo");
  lcd.setCursor(0, 1);
  lcd.print("Esperando datos...");
  
  for(;;) {
    if(xSemaphoreTake(semDisplayReady, portMAX_DELAY) == pdTRUE) {
       displayCount++;
       if(dataReady) {
        lcd.clear();
        
        // Línea 1: Temperatura actual y promedio
        lcd.setCursor(0, 0);
        lcd.print("T:");
        lcd.print(globalTempCurrent, 1);
        lcd.print("C ");
        lcd.print("Pm:");
        lcd.print(globalTempAvg, 1);
        lcd.print("C");
        
        // Línea 2: Humedad actual y promedio
        lcd.setCursor(0, 1);
        lcd.print("H:");
        lcd.print(globalHumCurrent, 1);
        lcd.print("% ");
        lcd.print("Pm:");
        lcd.print(globalHumAvg, 1);
        lcd.print("%");
        
        // Línea 3: Estado / Alertas
        lcd.setCursor(0, 2);
        bool tempAlert = (globalTempAvg > TEMP_THRESHOLD);
        bool humAlert = (globalHumAvg > HUM_THRESHOLD);
        
        if(tempAlert && humAlert) {
          lcd.print("ALERTA: T y H alta!");
        } else if(tempAlert) {
          lcd.print("ALERTA: Temp alta!  ");
        } else if(humAlert) {
          lcd.print("ALERTA: Hum alta!   ");
        } else {
          // Verificar rango de confort
          if(globalTempAvg >= 20 && globalTempAvg <= 25 && 
             globalHumAvg >= 40 && globalHumAvg <= 60) {
            lcd.print("Estado: CONFORT     ");
          } else {
            lcd.print("Estado: NORMAL      ");
          }
        }
        
        // Línea 4: Tiempo transcurrido
        lcd.setCursor(0, 3);
        lcd.print("Tiempo: ");
        lcd.print(millis() / 1000);
        lcd.print(" seg    ");
        
        // También mostrar en Serial para debug
        Serial.println("====================");
        Serial.print("Reporte #");
        Serial.println(displayCount);
        Serial.println("====================");
        Serial.print("Temp: ");
        Serial.print(globalTempCurrent, 1);
        Serial.print("C (Prom: ");
        Serial.print(globalTempAvg, 1);
        Serial.println("C)");
        Serial.print("Hum:  ");
        Serial.print(globalHumCurrent, 1);
        Serial.print("% (Prom: ");
        Serial.print(globalHumAvg, 1);
        Serial.println("%)");
        
        if(tempAlert) {
          Serial.println("  ALERTA: Temperatura promedio alta!");
        }
        if(humAlert) {
          Serial.println("  ALERTA: Humedad promedio alta!");
        }
        
        Serial.println();
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);  

  Serial.println("\n\n=================================");
  Serial.println("  Sistema de Monitoreo Iniciado");
  Serial.println("=================================\n");
  
  // Configurar pines
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_YELLOW, LOW);

  // Pantalla LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Iniciando sistema");
  lcd.setCursor(0, 1);
  lcd.print("Por favor espere...");
  
  // Crear queues
  queueTemp = xQueueCreate(QUEUE_SIZE, sizeof(SensorData_t));
  queueHum = xQueueCreate(QUEUE_SIZE, sizeof(SensorData_t));
  
  if(queueTemp == NULL || queueHum == NULL) {
    Serial.println(" Error: No se pudieron crear las queues");
    while(1);  // Detener el programa
  }
  
  // Crear semáforo
  semDisplayReady = xSemaphoreCreateBinary();
  semSystemReady = xSemaphoreCreateBinary();
  
  if(semDisplayReady == NULL || semSystemReady == NULL) {
    Serial.println("Error: No se pudo crear el semaforo");
    while(1);
  }
  
  // Crear Tarea 1: Sensor de Temperatura
  xTaskCreatePinnedToCore(
    SensorTask1,           // Función
    "SensorTemp",          // Nombre
    4096,                  // Stack size
    NULL,                  // Parámetros
    2,                     // Prioridad 
    &SensorTask1Handle,    // Handle
    0                      // Core 0
  );
  
  // Crear Tarea 2: Sensor de Humedad
  xTaskCreatePinnedToCore(
    SensorTask2,           // Función
    "SensorHum",           // Nombre
    4096,                  // Stack size
    NULL,                  // Parámetros
    2,                     // Prioridad 
    &SensorTask2Handle,    // Handle
    0                      // Core 0
  );

  // Crear Tarea 3: Procesamiento
  xTaskCreatePinnedToCore(
    ProcessTask,           // Función
    "Process",             // Nombre
    8192,                  // Stack size 
    NULL,                  // Parámetros
    3,                     // Prioridad ALTA 
    &ProcessTaskHandle,    // Handle
    1                      // Core 1
  );
  
  // Crear Tarea 4: Display
  xTaskCreatePinnedToCore(
    DisplayTask,           // Función
    "Display",             // Nombre
    4096,                  // Stack size
    NULL,                  // Parámetros
    1,                     // Prioridad BAJA 
    &DisplayTaskHandle,    // Handle
    1                      // Core 1
  );

  vTaskDelay(pdMS_TO_TICKS(200));
  xSemaphoreGive(semSystemReady);
}

void loop() {
}