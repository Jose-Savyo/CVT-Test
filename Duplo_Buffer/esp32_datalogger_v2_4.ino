/*
 * =====================================================================================
 *
 * Filename:  esp32_datalogger_v2_4.ino (Correção Compilação Timer)
 *
 * Descrição:  Firmware V2.4 para Datalogger ESP32.
 * - Corrige erros de compilação da V2.3 ('TIMERG0' e 'TIMER_BASE_CLK'
 * não declarados).
 *
 * - *** CORREÇÃO CRÍTICA (V2.4) ***
 * - Substitui 'TIMER_BASE_CLK' pela constante 'APB_CLK_FREQ' (definida em soc/clk_freq_defs.h),
 * que é a frequência correta do clock APB usado pelos timers.
 * - Modifica a ISR para usar a função 'timer_group_clr_intr_status_in_isr'
 * e 'timer_group_enable_alarm_in_isr' em vez de acesso direto a registradores,
 * tornando o código compatível com mais versões do ESP-IDF.
 * - Adiciona o include <soc/clk_freq_defs.h>.
 *
 * =====================================================================================
 */

// --- 1. BIBLIOTECAS ---
#include <WiFi.h>
#include "esp_heap_caps.h" // Para alocação
#include "freertos/queue.h"  // Para Fila Core0 -> Core1
#include "freertos/semphr.h" // Para Semáforo ISR -> Core0
#include "driver/timer.h"    // Para Timer de Hardware
#include "soc/clk_freq_defs.h" // <<<--- ADICIONADO: Define APB_CLK_FREQ

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
#define TIMER_DIVIDER         80  // Clock Base APB (~80MHz). 80MHz / 80 = 1MHz (ticks de 1µs)
// <<<--- CORRIGIDO: Usa APB_CLK_FREQ ---<<<
#define TIMER_SCALE           (APB_CLK_FREQ / TIMER_DIVIDER) // Frequência do nosso timer = 1MHz
#define TIMER_INTERVALO_US    INTERVALO_COLETA_US // Nosso alvo = 200µs
#define TIMER_ALARME_VALOR    (TIMER_INTERVALO_US * (TIMER_SCALE / 1000000ULL)) // Valor do alarme = 200 ticks (ULL para unsigned long long)
// Usamos TIMER_GROUP_0 e TIMER_0
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
    // <<<--- CORRIGIDO: Usa funções da API em vez de registradores diretos ---<<<
    // Limpa a flag de interrupção
    timer_group_clr_intr_status_in_isr(TIMER_GROUP, TIMER_INDEX);
    // Reabilita o alarme para a próxima interrupção (necessário com auto_reload=true?)
    // A documentação sugere que com auto_reload=true, isso não é estritamente necessário,
    // mas não custa garantir. Se causar problemas (ex: interrupções duplas), comente.
    timer_group_enable_alarm_in_isr(TIMER_GROUP, TIMER_INDEX);
    // >>>--- FIM DA CORREÇÃO ---<<<

    // Acorda a tarefa de coleta (taskColetaCore0)
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(sem_timer_sync, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// =====================================================================================
// --- 6. FUNÇÃO DE CONFIGURAÇÃO DO TIMER (Chamada no Setup) ---
// =====================================================================================
void timer_init_setup() {
  timer_config_t config = {
      .alarm_en = TIMER_ALARM_EN,
      .counter_en = TIMER_PAUSE,
      .intr_type = TIMER_INTR_LEVEL,
      .counter_dir = TIMER_COUNT_UP,
      .auto_reload = TIMER_AUTORELOAD_EN, // <- Confirmado: Auto-reload está ativo
      .divider = TIMER_DIVIDER
  };

  timer_init(TIMER_GROUP, TIMER_INDEX, &config);
  timer_set_counter_value(TIMER_GROUP, TIMER_INDEX, 0x00000000ULL);
  timer_set_alarm_value(TIMER_GROUP, TIMER_INDEX, TIMER_ALARME_VALOR);
  timer_enable_intr(TIMER_GROUP, TIMER_INDEX);
  timer_isr_register(TIMER_GROUP, TIMER_INDEX, timer_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);

  Serial.println("Core 1: Timer de Hardware configurado.");
}


// =====================================================================================
// --- 7. TAREFA DO NÚCLEO 0 (COLETA DE DADOS - Acordada pelo Timer) ---
// (Sem mudanças na lógica principal)
// =====================================================================================
void taskColetaCore0(void* pvParameters) {
  Serial.println("Core 0: Tarefa de coleta (Timer ISR) iniciada.");

  PontoDeColeta* bufferAtual = g_bufferA;
  int indiceBuffer = 0;

  while(g_estadoAtual != ESTADO_STREAMING){
      vTaskDelay(pdMS_TO_TICKS(100));
  }

  Serial.println("Core 0: Cliente conectado! Iniciando coleta contínua a 5kHz (Timer).");
  timer_start(TIMER_GROUP, TIMER_INDEX); // Inicia o timer
  Serial.println("Core 0: Timer iniciado!");

  for (;;) {
      if(xSemaphoreTake(sem_timer_sync, portMAX_DELAY) == pdTRUE) {
          bufferAtual[indiceBuffer].estadoSensor1 = digitalRead(SENSOR_PIN_1);
          bufferAtual[indiceBuffer].estadoSensor2 = digitalRead(SENSOR_PIN_2);
          bufferAtual[indiceBuffer].timestamp_us = micros();

          indiceBuffer++;
          if (indiceBuffer >= PONTOS_POR_BUFFER) {
              Serial.print("Core 0: Buffer ");
              Serial.print((bufferAtual == g_bufferA) ? 'A' : 'B');
              Serial.println(" cheio. Enviando para Core 1.");
              xQueueSend(filaBuffersProntos, &bufferAtual, 0);
              bufferAtual = (bufferAtual == g_bufferA) ? g_bufferB : g_bufferA;
              indiceBuffer = 0;
          }
      }
      if(g_estadoAtual != ESTADO_STREAMING){
           Serial.println("Core 0: Cliente desconectou. Pausando coleta e timer.");
           timer_pause(TIMER_GROUP, TIMER_INDEX);
           while(g_estadoAtual != ESTADO_STREAMING){ vTaskDelay(pdMS_TO_TICKS(100)); }
           Serial.println("Core 0: Cliente reconectou! Retomando coleta...");
           timer_set_counter_value(TIMER_GROUP, TIMER_INDEX, 0x00000000ULL);
           timer_start(TIMER_GROUP, TIMER_INDEX);
           indiceBuffer = 0;
      }
  }
}

// =====================================================================================
// --- 8. FUNÇÕES DE GERENCIAMENTO (NÚCLEO 1 - loop) ---
// (Sem mudanças)
// =====================================================================================
void gerenciarAguardandoConexao() { /* ... código idêntico ao V2.3 ... */ }
void gerenciarStreaming() { /* ... código idêntico ao V2.3 ... */ }
// Cole as funções completas da V2.3 aqui se precisar do código inteiro


// =====================================================================================
// --- 9. SETUP (Executado uma vez no Núcleo 1) ---
// =====================================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("--- Datalogger ESP32 V2.4 (Timer Hardware Fix) ---");
  Serial.println("Core 0: Dedicado à Coleta de Dados (ISR + Task).");
  Serial.println("Core 1: Rodando Setup, Loop (Rede/Envio).");

  // Aloca Buffers
  Serial.print("Core 1: Alocando 2x "); Serial.print(TAMANHO_UM_BUFFER_BYTES / 1024.0, 1); Serial.println("KB de SRAM...");
  g_bufferA = (PontoDeColeta*) heap_caps_malloc(TAMANHO_UM_BUFFER_BYTES, MALLOC_CAP_DMA);
  g_bufferB = (PontoDeColeta*) heap_caps_malloc(TAMANHO_UM_BUFFER_BYTES, MALLOC_CAP_DMA);
  if (g_bufferA == NULL || g_bufferB == NULL) { /* ... código de erro idêntico V2.3 ... */ }
  Serial.println("Sucesso! Buffers alocados.");

  // Config Pinos
  pinMode(LED_STATUS_PIN, OUTPUT); pinMode(SENSOR_PIN_1, INPUT_PULLUP); pinMode(SENSOR_PIN_2, INPUT_PULLUP);

  // Cria Fila e Semáforo
  filaBuffersProntos = xQueueCreate(2, sizeof(PontoDeColeta*));
  sem_timer_sync = xSemaphoreCreateBinary();

  // Configura Timer
  timer_init_setup(); // <<<--- Chamada da função corrigida

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

// --- Funções gerenciarEstadoOcioso e gerenciarStreaming (completas) ---
// Adicione o código completo dessas duas funções aqui, copiando da V2.3,
// pois elas não foram alteradas.
