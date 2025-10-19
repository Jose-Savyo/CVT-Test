/*
 * =====================================================================================
 *
 * Filename:  esp32_datalogger_v1_6.ino (Versão com Ensaio de 5s / 90KB)
 *
 * Descrição:  Firmware V1.6 para Datalogger ESP32.
 * - Testes anteriores (180KB, 106KB) falharam na alocação.
 * - Reduz o tempo de ensaio para 5 segundos (3kHz * 5s = 15.000 pontos).
 * - O novo buffer de 90KB (15.000 * 6) deve ser alocado com segurança.
 *
 * =====================================================================================
 */

// --- 1. BIBLIOTECAS ---
#include <WiFi.h>
#include "esp_heap_caps.h" // Para alocação de memória avançada

// --- 2. ESTRUTURA DE DADOS ---
struct PontoDeColeta {
  byte estadoSensor1;
  byte estadoSensor2;
  unsigned long timestamp_us;
}; // Total: 6 bytes

// --- 3. DEFINIÇÕES E CONFIGURAÇÕES ---
// MUDANÇA PRINCIPAL AQUI - ENSAIO DE 5 SEGUNDOS
const int FREQ_COLETA_HZ = 3000;
const int TEMPO_ENSAIO_S = 5; // Reduzido para 5 segundos
const long INTERVALO_COLETA_US = 1000000 / FREQ_COLETA_HZ; // 333µs

const int TAMANHO_BUFFER_PONTOS = FREQ_COLETA_HZ * TEMPO_ENSAIO_S; // Agora 15.000
// 15.000 * 6 = 90.000 bytes (90 KB)
const int TAMANHO_BUFFER_BYTES = TAMANHO_BUFFER_PONTOS * sizeof(PontoDeColeta); 

const int SENSOR_PIN_1 = 15;
const int SENSOR_PIN_2 = 16;
const int LED_STATUS_PIN = 2;

const char* WIFI_SSID = "ESP32-DATALOGGER";
const char* WIFI_SENHA = "senha1234";
const int TCP_PORTA = 80;

// --- 4. VARIÁVEIS GLOBAIS ---
PontoDeColeta* g_dataBuffer;
enum EstadoSistema { ESTADO_OCIOSO, ESTADO_COLETANDO, ESTADO_ENVIANDO };
volatile EstadoSistema g_estadoAtual = ESTADO_OCIOSO;
WiFiServer servidorTCP(TCP_PORTA);
WiFiClient clienteTCP;
TaskHandle_t h_taskColeta;
SemaphoreHandle_t sem_inicioColeta;
volatile bool g_coletaConcluida = false;

// =====================================================================================
// --- 5. TAREFA DO NÚCLEO 1 (COLETA DE DADOS) ---
// =====================================================================================
void taskColetaCore1(void* pvParameters) {
  Serial.println("Core 1: Tarefa de coleta iniciada e pronta.");
  for (;;) {
    xSemaphoreTake(sem_inicioColeta, portMAX_DELAY);
    Serial.println("Core 1: Sinal recebido! Iniciando coleta de 5s..."); // Mensagem atualizada
    unsigned long tempoProximaColeta = micros();
    for (int i = 0; i < TAMANHO_BUFFER_PONTOS; i++) {
      g_dataBuffer[i].estadoSensor1 = digitalRead(SENSOR_PIN_1);
      g_dataBuffer[i].estadoSensor2 = digitalRead(SENSOR_PIN_2);
      g_dataBuffer[i].timestamp_us = micros();
      tempoProximaColeta += INTERVALO_COLETA_US;
      while (micros() < tempoProximaColeta) {
        // Espera ativamente
      }
    }
    Serial.println("Core 1: Coleta concluída! Avisando Core 0.");
    g_coletaConcluida = true;
  }
}

// =====================================================================================
// --- 6. FUNÇÕES DE GERENCIAMENTO (NÚCLEO 0) ---
// =====================================================================================
void gerenciarEstadoOcioso() {
  digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
  delay(500);
  if (!clienteTCP.connected()) {
    clienteTCP = servidorTCP.available();
    if (clienteTCP) {
      Serial.println("Core 0: Cliente TCP conectado!");
    }
    return;
  }
  if (clienteTCP.available() > 0) {
    char cmd = clienteTCP.read();
    if (cmd == 'S') {
      Serial.println("Core 0: Comando 'S' recebido. Iniciando coleta...");
      g_coletaConcluida = false;
      digitalWrite(LED_STATUS_PIN, HIGH);
      xSemaphoreGive(sem_inicioColeta);
      g_estadoAtual = ESTADO_COLETANDO;
    } else {
      Serial.print("Core 0: Comando '");
      Serial.print(cmd);
      Serial.println("' desconhecido.");
    }
  }
}
void gerenciarEstadoColeta() {
  if (!clienteTCP.connected()) {
    Serial.println("Core 0: Cliente desconectou durante a coleta. Abortando.");
    g_estadoAtual = ESTADO_OCIOSO;
    return;
  }
  if (g_coletaConcluida) {
    Serial.println("Core 0: Flag de coleta concluída recebida.");
    g_estadoAtual = ESTADO_ENVIANDO;
  }
}
void gerenciarEstadoEnvio() {
  Serial.print("Core 0: Iniciando envio de "); // Mensagem atualizada
  Serial.print(TAMANHO_BUFFER_BYTES / 1024.0, 2); // Imprime 90.0KB
  Serial.println("KB de dados...");
  
  digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN
