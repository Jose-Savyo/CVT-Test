/*
 * =====================================================================================
 *
 * Filename:  esp32_datalogger_v2_7.ino (Cleaned Up V2.6)
 *
 * Descrição:  Firmware V2.7 para Datalogger ESP32.
 * - Versão funcional da V2.6 (Timer Hardware @ 5kHz).
 * - Remove as definições duplicadas de funções que causavam erro de compilação.
 *
 * =====================================================================================
 */

// --- 1. BIBLIOTECAS ---
#include <WiFi.h>
#include "esp_heap_caps.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/timer.h"
#include "soc/rtc.h" // Header correto para rtc_clk_apb_freq_get()

// --- 2. ESTRUTURA DE DADOS ---
struct PontoDeColeta {
  byte estadoSensor1;
  byte estadoSensor2;
  unsigned long timestamp_us;
} __attribute__((packed)); // 6 bytes exatos

// --- 3. DEFINIÇÕES E CONFIGURAÇÕES ---
const int FREQ_COLETA_HZ = 5000;
const long INTERVALO_COLETA_US = 1000000 / FREQ_COLETA_HZ; // 200µs

const int PONTOS_POR_BUFFER = 2560;
const int TAMANHO_UM_BUFFER_BYTES = PONTOS_POR_BUFFER * sizeof(PontoDeColeta);

const int SENSOR_PIN_1 = 15;
const int SENSOR_PIN_2 = 16;
const int LED_STATUS_PIN = 2;
const char* WIFI_SSID = "ESP32-DATALOGGER";
const char* WIFI_SENHA = "senha1234";
const int TCP_PORTA = 80;

// -- Configurações do Timer de Hardware --
#define TIMER_DIVIDER         80
#define TIMER_SCALE           (rtc_clk_apb_freq_get() / TIMER_DIVIDER)
#define TIMER_INTERVALO_US    INTERVALO_COLETA_US
#define TIMER_ALARME_VALOR    (TIMER_INTERVALO_US * (TIMER_SCALE / 1000000ULL))
#define TIMER_GROUP           TIMER_GROUP_0
#define TIMER_INDEX           TIMER_0

// --- 4. VARIÁVEIS GLOBAIS ---
PontoDeColeta* g_bufferA;
PontoDeColeta* g_bufferB;
enum EstadoSistema { ESTADO_AGUARDANDO_CONEXAO, ESTADO_STREAMING };
volatile EstadoSistema g_estadoAtual = ESTADO_AGUARDANDO_CONEXAO;
WiFiServer servidorTCP(TCP_PORTA);
WiFiClient clienteTCP;
TaskHandle_t h_taskColeta;
QueueHandle_t filaBuffersProntos;
SemaphoreHandle_t sem_timer_sync;

// =====================================================================================
// --- 5. ISR DO TIMER (Executada a cada 200µs) ---
// =====================================================================================
void IRAM_ATTR timer_isr(void *para) {
    timer_group_clr_intr_status_in_isr(TIMER_GROUP, TIMER_INDEX);
    timer_group_enable_alarm_in_isr(TIMER_GROUP, TIMER_INDEX);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(sem_timer_sync, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) { portYIELD_FROM_ISR(); }
}

// =====================================================================================
// --- 6. FUNÇÃO DE CONFIGURAÇÃO DO TIMER (Chamada no Setup) ---
// =====================================================================================
void timer_init_setup() {
  Serial.print("Core 1: Frequência APB detectada: ");
  Serial.print(rtc_clk_apb_freq_get() / 1000000);
  Serial.println(" MHz");
  timer_config_t config = {
      .alarm_en = TIMER_ALARM_EN, .counter_en = TIMER_PAUSE, .intr_type = TIMER_INTR_LEVEL,
      .counter_dir = TIMER_COUNT_UP, .auto_reload = TIMER_AUTORELOAD_EN, .divider = TIMER_DIVIDER };
  timer_init(TIMER_GROUP, TIMER_INDEX, &config);
  timer_set_counter_value(TIMER_GROUP, TIMER_INDEX, 0x00000000ULL);
  timer_set_alarm_value(TIMER_GROUP, TIMER_INDEX, TIMER_ALARME_VALOR);
  timer_enable_intr(TIMER_GROUP, TIMER_INDEX);
  timer_isr_register(TIMER_GROUP, TIMER_INDEX, timer_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);
  Serial.println("Core 1: Timer de Hardware configurado.");
}

// =====================================================================================
// --- 7. TAREFA DO NÚCLEO 0 (COLETA DE DADOS - Acordada pelo Timer) ---
// =====================================================================================
void taskColetaCore0(void* pvParameters) {
  Serial.println("Core 0: Tarefa de coleta (Timer ISR) iniciada.");
  PontoDeColeta* bufferAtual = g_bufferA; int indiceBuffer = 0;
  while(g_estadoAtual != ESTADO_STREAMING){ vTaskDelay(pdMS_TO_TICKS(100)); }
  Serial.println("Core 0: Cliente conectado! Iniciando coleta contínua a 5kHz (Timer).");
  timer_start(TIMER_GROUP, TIMER_INDEX); Serial.println("Core 0: Timer iniciado!");
  for (;;) {
      if(xSemaphoreTake(sem_timer_sync, portMAX_DELAY) == pdTRUE) {
          bufferAtual[indiceBuffer].estadoSensor1 = digitalRead(SENSOR_PIN_1);
          bufferAtual[indiceBuffer].estadoSensor2 = digitalRead(SENSOR_PIN_2);
          bufferAtual[indiceBuffer].timestamp_us = micros();
          indiceBuffer++;
          if (indiceBuffer >= PONTOS_POR_BUFFER) {
              Serial.print("Core 0: Buffer "); Serial.print((bufferAtual == g_bufferA) ? 'A' : 'B'); Serial.println(" cheio. Enviando para Core 1.");
              xQueueSend(filaBuffersProntos, &bufferAtual, 0); bufferAtual = (bufferAtual == g_bufferA) ? g_bufferB : g_bufferA; indiceBuffer = 0;
          }
      }
      if(g_estadoAtual != ESTADO_STREAMING){
           Serial.println("Core 0: Cliente desconectou. Pausando coleta e timer."); timer_pause(TIMER_GROUP, TIMER_INDEX);
           while(g_estadoAtual != ESTADO_STREAMING){ vTaskDelay(pdMS_TO_TICKS(100)); }
           Serial.println("Core 0: Cliente reconectou! Retomando coleta..."); timer_set_counter_value(TIMER_GROUP, TIMER_INDEX, 0x00000000ULL); timer_start(TIMER_GROUP, TIMER_INDEX); indiceBuffer = 0;
      }
  }
}

// =====================================================================================
// --- 8. FUNÇÕES DE GERENCIAMENTO (NÚCLEO 1 - loop) ---
// =====================================================================================
void gerenciarAguardandoConexao() {
  digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN)); delay(500);
  if (clienteTCP && !clienteTCP.connected()) { Serial.println("Core 1 (loop): Cliente anterior desconectado."); clienteTCP.stop(); }
  if (!clienteTCP || !clienteTCP.connected()) {
    clienteTCP = servidorTCP.available();
    if (clienteTCP && clienteTCP.connected()) {
      Serial.println("Core 1 (loop): Novo cliente TCP conectado!"); Serial.println("Core 1 (loop): Mudando para ESTADO_STREAMING.");
      digitalWrite(LED_STATUS_PIN, HIGH); g_estadoAtual = ESTADO_STREAMING;
    }
  }
}

void gerenciarStreaming() {
  if (!clienteTCP.connected()) { Serial.println("Core 1 (loop): Cliente desconectou durante streaming."); clienteTCP.stop(); g_estadoAtual = ESTADO_AGUARDANDO_CONEXAO; return; }
  PontoDeColeta* bufferParaEnviar = NULL;
  if (xQueueReceive(filaBuffersProntos, &bufferParaEnviar, pdMS_TO_TICKS(10)) == pdPASS) {
      Serial.print("Core 1 (loop): Recebido Buffer "); Serial.print((bufferParaEnviar == g_bufferA) ? 'A' : 'B'); Serial.print(". Iniciando envio de "); Serial.print(TAMANHO_UM_BUFFER_BYTES / 1024.0, 1); Serial.println("KB...");
      size_t bytesEnviados = clienteTCP.write((uint8_t*) bufferParaEnviar, TAMANHO_UM_BUFFER_BYTES);
      if (bytesEnviados == TAMANHO_UM_BUFFER_BYTES) { Serial.println("Core 1 (loop): Envio do buffer concluído com sucesso!"); }
      else { Serial.print("Core 1 (loop): Erro no envio do buffer! Enviados "); Serial.print(bytesEnviados); Serial.print(" de "); Serial.println(TAMANHO_UM_BUFFER_BYTES); }
  }
}

// =====================================================================================
// --- 9. SETUP (Executado uma vez no Núcleo 1) ---
// =====================================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("--- Datalogger ESP32 V2.7 (Timer Hardware OK) ---"); // <-- Versão Atualizada
  Serial.println("Core 0: Dedicado à Coleta de Dados (ISR + Task).");
  Serial.println("Core 1: Rodando Setup, Loop (Rede/Envio).");

  // Aloca Buffers
  Serial.print("Core 1: Alocando 2x "); Serial.print(TAMANHO_UM_BUFFER_BYTES / 1024.0, 1); Serial.println("KB de SRAM...");
  g_bufferA = (PontoDeColeta*) heap_caps_malloc(TAMANHO_UM_BUFFER_BYTES, MALLOC_CAP_DMA);
  g_bufferB = (PontoDeColeta*) heap_caps_malloc(TAMANHO_UM_BUFFER_BYTES, MALLOC_CAP_DMA);
  if (g_bufferA == NULL || g_bufferB == NULL) {
      Serial.print("\nFALHA CRITICA! Impossivel alocar os dois buffers.");
      pinMode(LED_STATUS_PIN, OUTPUT);
      while (true) { digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN)); delay(50); }
  }
  Serial.println("Sucesso! Buffers alocados.");

  // Config Pinos, Fila, Semáforo
  pinMode(LED_STATUS_PIN, OUTPUT); pinMode(SENSOR_PIN_1, INPUT_PULLUP); pinMode(SENSOR_PIN_2, INPUT_PULLUP);
  filaBuffersProntos = xQueueCreate(2, sizeof(PontoDeColeta*));
  sem_timer_sync = xSemaphoreCreateBinary();

  // Configura Timer
  timer_init_setup();

  // Inicia Task Coleta Core 0
  xTaskCreatePinnedToCore( taskColetaCore0, "TaskColeta", 10000, NULL, configMAX_PRIORITIES - 1, &h_taskColeta, 0);

  // Configura Wi-Fi e Rede Core 1
  Serial.print("Core 1: Configurando Access Point Wi-Fi..."); WiFi.softAP(WIFI_SSID, WIFI_SENHA);
  Serial.print("OK! IP do AP: "); Serial.println(WiFi.softAPIP());
  Serial.print("Core 1: Iniciando Servidor TCP na porta "); Serial.println(TCP_PORTA); servidorTCP.begin();
  Serial.println("\n--- Sistema pronto. Aguardando cliente ---");
}

// =====================================================================================
// --- 10. LOOP PRINCIPAL (Executado continuamente no Núcleo 1) ---
// =====================================================================================
void loop() {
  switch (g_estadoAtual) {
    case ESTADO_AGUARDANDO_CONEXAO:
      gerenciarAguardandoConexao();
      break;
    case ESTADO_STREAMING:
      gerenciarStreaming();
      break;
  }
}
