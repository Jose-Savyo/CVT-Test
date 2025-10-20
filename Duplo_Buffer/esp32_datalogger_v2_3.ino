/*
 * =====================================================================================
 *
 * Filename:  esp32_datalogger_v2_3.ino (Timer de Hardware @ 5kHz)
 *
 * Descrição:  Firmware V2.3 para Datalogger ESP32.
 * - Implementa a temporização de 5kHz usando um Timer de Hardware dedicado.
 * - Isso garante a máxima precisão temporal (jitter mínimo) e libera a CPU
 * do Core 0 entre as coletas, tornando o sistema mais eficiente e robusto.
 *
 * - ARQUITETURA:
 * - Timer Group 0, Timer 0 configurado para gerar interrupção a cada 200µs.
 * - Uma ISR (Interrupt Service Routine) rápida sinaliza um semáforo a cada 200µs.
 * - A 'taskColetaCore0' (Core 0, Prio Máxima) fica bloqueada esperando esse semáforo.
 * - Ao ser acordada, realiza UMA coleta e volta a esperar.
 * - Core 1 (Loop, Prio 1) gerencia Wi-Fi, TCP e envio de buffers via Fila.
 * - Buffers: 2 x 15KB (2560 pontos cada).
 *
 * =====================================================================================
 */

// --- 1. BIBLIOTECAS ---
#include <WiFi.h>
#include "esp_heap_caps.h" // Para alocação
#include "freertos/queue.h"  // Para Fila Core0 -> Core1
#include "freertos/semphr.h" // Para Semáforo ISR -> Core0
#include "driver/timer.h"    // Para Timer de Hardware

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
#define TIMER_DIVIDER         80  // Clock Base = 80MHz. 80MHz / 80 = 1MHz (ticks de 1µs)
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER) // Frequência do nosso timer = 1MHz
#define TIMER_INTERVALO_US    INTERVALO_COLETA_US // Nosso alvo = 200µs
#define TIMER_ALARME_VALOR    (TIMER_INTERVALO_US * (TIMER_SCALE / 1000000)) // Valor do alarme = 200 ticks

// --- 4. VARIÁVEIS GLOBAIS ---
PontoDeColeta* g_bufferA;
PontoDeColeta* g_bufferB;
enum EstadoSistema { ESTADO_AGUARDANDO_CONEXAO, ESTADO_STREAMING };
volatile EstadoSistema g_estadoAtual = ESTADO_AGUARDANDO_CONEXAO;
WiFiServer servidorTCP(TCP_PORTA);
WiFiClient clienteTCP; 
TaskHandle_t h_taskColeta; 
QueueHandle_t filaBuffersProntos; 
// Semáforo Binário: ISR sinaliza para a taskColeta acordar
SemaphoreHandle_t sem_timer_sync; 


// =====================================================================================
// --- 5. ISR DO TIMER (Executada a cada 200µs) ---
// =====================================================================================
// IRAM_ATTR: Coloca esta função na RAM para execução ultra-rápida
void IRAM_ATTR timer_isr(void *para) {
    // Limpa a flag de interrupção do Timer Group 0, Timer 0
    TIMERG0.int_clr_timers.t0 = 1;
    // Reabilita o alarme para a próxima interrupção
    TIMERG0.hw_timer[0].config.alarm_en = TIMER_ALARM_EN;

    // Acorda a tarefa de coleta (taskColetaCore0)
    // Usamos xSemaphoreGiveFromISR pois estamos dentro de uma interrupção
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(sem_timer_sync, &xHigherPriorityTaskWoken);

    // Se a tarefa acordada tiver maior prioridade, força a troca de contexto
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// =====================================================================================
// --- 6. FUNÇÃO DE CONFIGURAÇÃO DO TIMER (Chamada no Setup) ---
// =====================================================================================
void timer_init_setup() {
  timer_config_t config = {
      .alarm_en = TIMER_ALARM_EN,      // Habilita alarme
      .counter_en = TIMER_PAUSE,     // Timer começa pausado
      .intr_type = TIMER_INTR_LEVEL, // Tipo de interrupção
      .counter_dir = TIMER_COUNT_UP, // Contagem crescente
      .auto_reload = TIMER_AUTORELOAD_EN, // Recarrega o contador automaticamente
      .divider = TIMER_DIVIDER       // Divisor do clock (80 -> 1MHz)
  };

  timer_init(TIMER_GROUP_0, TIMER_0, &config);
  // Define o valor do contador inicial como 0
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
  // Define o valor do alarme (200 ticks = 200µs)
  timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_ALARME_VALOR);
  // Habilita a interrupção do timer
  timer_enable_intr(TIMER_GROUP_0, TIMER_0);
  // Registra a nossa função ISR para ser chamada na interrupção
  timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);

  // O timer só será iniciado ('timer_start') quando a conexão for estabelecida
  Serial.println("Core 1: Timer de Hardware configurado.");
}


// =====================================================================================
// --- 7. TAREFA DO NÚCLEO 0 (COLETA DE DADOS - Acordada pelo Timer) ---
// =====================================================================================
void taskColetaCore0(void* pvParameters) {
  Serial.println("Core 0: Tarefa de coleta (Timer ISR) iniciada.");
  
  PontoDeColeta* bufferAtual = g_bufferA;
  int indiceBuffer = 0;

  // Espera o sistema principal sinalizar conexão
  while(g_estadoAtual != ESTADO_STREAMING){
      vTaskDelay(pdMS_TO_TICKS(100)); 
  }
  
  Serial.println("Core 0: Cliente conectado! Iniciando coleta contínua a 5kHz (Timer).");
  
  // --- INICIA O TIMER DE HARDWARE ---
  timer_start(TIMER_GROUP_0, TIMER_0);
  Serial.println("Core 0: Timer iniciado!");

  // Loop principal de coleta contínua
  for (;;) {
      // 1. Dorme esperando o sinal da ISR (vem a cada 200µs)
      if(xSemaphoreTake(sem_timer_sync, portMAX_DELAY) == pdTRUE) {
          
          // 2. Acordou! Coleta os dados.
          bufferAtual[indiceBuffer].estadoSensor1 = digitalRead(SENSOR_PIN_1);
          bufferAtual[indiceBuffer].estadoSensor2 = digitalRead(SENSOR_PIN_2);
          bufferAtual[indiceBuffer].timestamp_us = micros(); 
          
          indiceBuffer++;

          // 3. Verifica se o buffer atual encheu
          if (indiceBuffer >= PONTOS_POR_BUFFER) {
              Serial.print("Core 0: Buffer ");
              Serial.print((bufferAtual == g_bufferA) ? 'A' : 'B');
              Serial.println(" cheio. Enviando para Core 1.");
              
              xQueueSend(filaBuffersProntos, &bufferAtual, 0); 
              bufferAtual = (bufferAtual == g_bufferA) ? g_bufferB : g_bufferA;
              indiceBuffer = 0;
          }
      }

      // Se o cliente desconectar, PAUSA o timer e espera
      if(g_estadoAtual != ESTADO_STREAMING){
           Serial.println("Core 0: Cliente desconectou. Pausando coleta e timer.");
           timer_pause(TIMER_GROUP_0, TIMER_0); // Pausa o timer
           
           while(g_estadoAtual != ESTADO_STREAMING){
               vTaskDelay(pdMS_TO_TICKS(100)); 
           }
           
           Serial.println("Core 0: Cliente reconectou! Retomando coleta...");
           timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL); // Reseta contador
           timer_start(TIMER_GROUP_0, TIMER_0); // Reinicia o timer
           indiceBuffer = 0; 
           // Poderia haver um ponto perdido na transição, mas simplifica
      }
  }
}

// =====================================================================================
// --- 8. FUNÇÕES DE GERENCIAMENTO (NÚCLEO 1 - loop) ---
// (Sem mudanças na lógica)
// =====================================================================================
void gerenciarAguardandoConexao() {
  digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN)); 
  delay(500);
  if (clienteTCP && !clienteTCP.connected()) {
      Serial.println("Core 1 (loop): Cliente anterior desconectado.");
      clienteTCP.stop(); 
  }
  if (!clienteTCP || !clienteTCP.connected()) {
    clienteTCP = servidorTCP.available();
    if (clienteTCP && clienteTCP.connected()) {
      Serial.println("Core 1 (loop): Novo cliente TCP conectado!");
      Serial.println("Core 1 (loop): Mudando para ESTADO_STREAMING.");
      digitalWrite(LED_STATUS_PIN, HIGH); 
      g_estadoAtual = ESTADO_STREAMING; // Sinaliza Core 0 para iniciar timer
    }
  }
}

void gerenciarStreaming() {
  if (!clienteTCP.connected()) {
      Serial.println("Core 1 (loop): Cliente desconectou durante streaming.");
      clienteTCP.stop();
      g_estadoAtual = ESTADO_AGUARDANDO_CONEXAO; // Sinaliza Core 0 para pausar timer
      return;
  }

  PontoDeColeta* bufferParaEnviar = NULL;
  if (xQueueReceive(filaBuffersProntos, &bufferParaEnviar, pdMS_TO_TICKS(10)) == pdPASS) {
      Serial.print("Core 1 (loop): Recebido Buffer ");
      Serial.print((bufferParaEnviar == g_bufferA) ? 'A' : 'B');
      Serial.print(". Iniciando envio de ");
      Serial.print(TAMANHO_UM_BUFFER_BYTES / 1024.0, 1);
      Serial.println("KB...");

      size_t bytesEnviados = clienteTCP.write((uint8_t*) bufferParaEnviar, TAMANHO_UM_BUFFER_BYTES);

      if (bytesEnviados == TAMANHO_UM_BUFFER_BYTES) {
          Serial.println("Core 1 (loop): Envio do buffer concluído com sucesso!");
      } else {
          Serial.print("Core 1 (loop): Erro no envio do buffer! Enviados ");
          Serial.print(bytesEnviados); Serial.print(" de "); Serial.println(TAMANHO_UM_BUFFER_BYTES);
      }
  }
}

// =====================================================================================
// --- 9. SETUP (Executado uma vez no Núcleo 1) ---
// =====================================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("--- Datalogger ESP32 V2.3 (Timer Hardware @ 5kHz) ---");
  Serial.println("Core 0: Dedicado à Coleta de Dados (ISR + Task).");
  Serial.println("Core 1: Rodando Setup, Loop (Rede/Envio).");

  // --- 1. Alocar os DOIS Buffers ---
  Serial.print("Core 1: Alocando 2x ");
  Serial.print(TAMANHO_UM_BUFFER_BYTES / 1024.0, 1);
  Serial.println("KB de SRAM (ANTES do Wi-Fi)..."); 
  g_bufferA = (PontoDeColeta*) heap_caps_malloc(TAMANHO_UM_BUFFER_BYTES, MALLOC_CAP_DMA);
  g_bufferB = (PontoDeColeta*) heap_caps_malloc(TAMANHO_UM_BUFFER_BYTES, MALLOC_CAP_DMA);
  if (g_bufferA == NULL || g_bufferB == NULL) {
    Serial.print("\nFALHA CRITICA! Impossivel alocar os dois buffers.");
    pinMode(LED_STATUS_PIN, OUTPUT);
    while (true) { digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN)); delay(50); }
  }
  Serial.println("Sucesso! Buffers alocados.");

  // --- 2. Configurar Pinos ---
  pinMode(LED_STATUS_PIN, OUTPUT);
  pinMode(SENSOR_PIN_1, INPUT_PULLUP);
  pinMode(SENSOR_PIN_2, INPUT_PULLUP);

  // --- 3. Criar a Fila e o Semáforo ---
  filaBuffersProntos = xQueueCreate(2, sizeof(PontoDeColeta*)); 
  sem_timer_sync = xSemaphoreCreateBinary(); // Semáforo para ISR -> Task

  // --- 4. Configurar o Timer de Hardware ---
  timer_init_setup(); // Chama a nossa nova função de configuração

  // --- 5. Iniciar a Tarefa de Coleta no Core 0 ---
  xTaskCreatePinnedToCore(
      taskColetaCore0, "TaskColeta", 10000, NULL, configMAX_PRIORITIES - 1, &h_taskColeta, 0);
  
  // --- 6. Configurar Wi-Fi e Rede (Core 1) ---
  Serial.print("Core 1: Configurando Access Point Wi-Fi...");
  WiFi.softAP(WIFI_SSID, WIFI_SENHA);
  Serial.print("OK! IP do AP: "); Serial.println(WiFi.softAPIP());
  Serial.print("Core 1: Iniciando Servidor TCP na porta "); Serial.println(TCP_PORTA);
  servidorTCP.begin();
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
