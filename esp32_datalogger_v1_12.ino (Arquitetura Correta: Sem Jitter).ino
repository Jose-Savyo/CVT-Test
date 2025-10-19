/*
 * =====================================================================================
 *
 * Filename:  esp32_datalogger_v1_12.ino (Arquitetura Correta: Sem Jitter)
 *
 * Descrição:  Firmware V1.12 para Datalogger ESP32.
 * - A V1.11 falhou (deadlock) ao mover a coleta para o Core 0 e competir com o Wi-Fi.
 *
 * - *** CORREÇÃO CRÍTICA (V1.12) ***
 * 1. Arquitetura: Retornamos ao plano de rodar TUDO no Núcleo 1 (Core 1).
 * O Core 0 é deixado livre para o Wi-Fi.
 *
 * 2. Correção de Jitter: A V1.10 tinha jitter. A causa era o loop de "espera ativa"
 * ('while (micros < ...)') que sufocava a 'loop()' (TCP) de prioridade baixa.
 *
 * 3. A Solução: Usamos 'vTaskDelayUntil()'. Esta é a função correta do FreeRTOS
 * para tarefas periódicas. Ela coloca a tarefa de coleta para "dormir"
 * e a acorda precisamente no próximo intervalo, sem usar 100% da CPU.
 *
 * 4. Ajuste de Frequência: 'vTaskDelayUntil' funciona com "ticks" de 1ms.
 * Para ter zero jitter, ajustamos a frequência de coleta para 1kHz (1000µs = 1ms).
 *
 * PARÂMETROS FINAIS: 1kHz * 6 segundos = 6.000 pontos = 36KB de buffer.
 *
 * =====================================================================================
 */

// --- 1. BIBLIOTECAS ---
#include <WiFi.h>
#include "esp_heap_caps.h"

// --- 2. ESTRUTURA DE DADOS ---
struct PontoDeColeta {
  byte estadoSensor1;
  byte estadoSensor2;
  unsigned long timestamp_us;
} __attribute__((packed)); // 6 bytes exatos

// --- 3. DEFINIÇÕES E CONFIGURAÇÕES ---
// MUDANÇA PRINCIPAL AQUI - 1kHz PARA USAR vTaskDelayUntil
const int FREQ_COLETA_HZ = 1000; // Ajustado para 1kHz
const int TEMPO_ENSAIO_S = 6; 
const long INTERVALO_COLETA_MS = 1000 / FREQ_COLETA_HZ; // 1ms

const int TAMANHO_BUFFER_PONTOS = FREQ_COLETA_HZ * TEMPO_ENSAIO_S; // Agora 6.000
const int TAMANHO_BUFFER_BYTES = TAMANHO_BUFFER_PONTOS * sizeof(PontoDeColeta); // Agora 36.000

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
  Serial.println("Core 1: Tarefa de coleta iniciada e pronta (Prioridade 5).");
  
  // Converte nosso intervalo de 1ms para "Ticks" do FreeRTOS
  const TickType_t xFrequency = pdMS_TO_TICKS(INTERVALO_COLETA_MS);
  TickType_t xLastWakeTime;

  for (;;) {
    // 1. Espera pelo "sinal verde" (semáforo) vindo do Core 0
    xSemaphoreTake(sem_inicioColeta, portMAX_DELAY);
    Serial.println("Core 1: Sinal recebido! Iniciando coleta de 6s a 1kHz...");

    // 2. Pega o tempo atual como ponto de partida
    xLastWakeTime = xTaskGetTickCount();

    for (int i = 0; i < TAMANHO_BUFFER_PONTOS; i++) {
      // 3. Coloca a tarefa para "dormir" até o próximo intervalo exato de 1ms
      // Esta função é o coração da estabilidade (zero jitter)
      vTaskDelayUntil(&xLastWakeTime, xFrequency);

      // 4. Acorda e coleta os dados
      g_dataBuffer[i].estadoSensor1 = digitalRead(SENSOR_PIN_1);
      g_dataBuffer[i].estadoSensor2 = digitalRead(SENSOR_PIN_2);
      g_dataBuffer[i].timestamp_us = micros();
    }

    // --- COLETA CONCLUÍDA ---
    Serial.println("Core 1: Coleta concluída! Avisando Core 1 (loop).");
    g_coletaConcluida = true;
  }
}

// =====================================================================================
// --- 6. FUNÇÕES DE GERENCIAMENTO (NÚCLEO 1 - loop) ---
// =====================================================================================
void gerenciarEstadoOcioso() {
  digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
  delay(500);
  if (!clienteTCP.connected()) {
    clienteTCP = servidorTCP.available();
    if (clienteTCP) { Serial.println("Core 1 (loop): Cliente TCP conectado!"); }
    return;
  }
  if (clienteTCP.available() > 0) {
    char cmd = clienteTCP.read();
    if (cmd == 'S') {
      Serial.println("Core 1 (loop): Comando 'S' recebido. Iniciando coleta...");
      g_coletaConcluida = false;
      digitalWrite(LED_STATUS_PIN, HIGH);
      xSemaphoreGive(sem_inicioColeta); // "Dá o sinal verde" para a taskColeta
      g_estadoAtual = ESTADO_COLETANDO;
    } else {
      Serial.print("Core 1 (loop): Comando '"); Serial.print(cmd); Serial.println("' desconhecido.");
    }
  }
}
void gerenciarEstadoColeta() {
  if (!clienteTCP.connected()) {
    Serial.println("Core 1 (loop): Cliente desconectou durante a coleta. Abortando.");
    g_estadoAtual = ESTADO_OCIOSO;
    return;
  }
  if (g_coletaConcluida) {
    Serial.println("Core 1 (loop): Flag de coleta concluída recebida.");
    g_estadoAtual = ESTADO_ENVIANDO;
  }
}
void gerenciarEstadoEnvio() {
  Serial.print("Core 1 (loop): Iniciando envio de ");
  Serial.print(TAMANHO_BUFFER_BYTES / 1024.0, 2); // Imprime 36.0KB
  Serial.println("KB de dados...");
  digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
  delay(100);
  if (clienteTCP.connected()) {
    size_t bytesEnviados = clienteTCP.write((uint8_t*) g_dataBuffer, TAMANHO_BUFFER_BYTES);
    if (bytesEnviados == TAMANHO_BUFFER_BYTES) {
      Serial.println("Core 1 (loop): Envio concluído com sucesso!");
    } else {
      Serial.print("Core 1 (loop): Erro no envio! Enviados ");
      Serial.print(bytesEnviados); Serial.print(" de "); Serial.println(TAMANHO_BUFFER_BYTES);
    }
  } else {
    Serial.println("Core 1 (loop): Erro! Cliente desconectou antes do envio.");
  }
  clienteTCP.stop();
  Serial.println("Core 1 (loop): Cliente desconectado. Voltando ao estado ocioso.");
  g_estadoAtual = ESTADO_OCIOSO;
}

// =====================================================================================
// --- 7. SETUP (Executado uma vez no Núcleo 1) ---
// =====================================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("--- Datalogger ESP32 Dual-Core V1.12 (1kHz No-Jitter) ---");
  Serial.println("Core 0: Dedicado ao Wi-Fi (Padrão).");
  Serial.println("Core 1: Rodando Setup, Loop (Prio 1) e Coleta (Prio 5).");

  // --- 1. Alocar Memória PRIMEIRO! ---
  Serial.print("Core 1: Alocando 36KB de SRAM (ANTES do Wi-Fi)...");
  
  g_dataBuffer = (PontoDeColeta*) heap_caps_malloc(TAMANHO_BUFFER_BYTES, MALLOC_CAP_DMA);
  if (g_dataBuffer == NULL) {
    g_dataBuffer = (PontoDeColeta*) heap_caps_malloc(TAMANHO_BUFFER_BYTES, MALLOC_CAP_8BIT);
  }
  if (g_dataBuffer == NULL) {
    Serial.print("\nFALHA CRITICA! Impossivel alocar 36KB."); 
    pinMode(LED_STATUS_PIN, OUTPUT);
    while (true) { digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN)); delay(50); }
  }
  Serial.println("Sucesso! Memória alocada.");

  // --- 2. Configurar Pinos ---
  pinMode(LED_STATUS_PIN, OUTPUT);
  pinMode(SENSOR_PIN_1, INPUT_PULLUP);
  pinMode(SENSOR_PIN_2, INPUT_PULLUP);

  // --- 3. Criar Semáforo de Sincronização ---
  sem_inicioColeta = xSemaphoreCreateBinary();

  // --- 4. Iniciar a Tarefa de Coleta (no Core 1) ---
  xTaskCreatePinnedToCore(
      taskColetaCore1,    // Função da tarefa
      "TaskColeta",       // Nome
      10000,              // Stack
      NULL,               // Parâmetros
      5,                  // Prioridade (MÉDIA, >1 que a loop() )
      &h_taskColeta,      // Handle
      1);                 // Núcleo: CORE 1 (junto com o loop)
  
  // --- 5. Configurar Wi-Fi (Core 0) e Rede (Core 1) ---
  Serial.print("Core 1: Configurando Access Point Wi-Fi...");
  WiFi.softAP(WIFI_SSID, WIFI_SENHA);
  Serial.print("OK! IP do AP: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("Core 1: Iniciando Servidor TCP na porta ");
  Serial.println(TCP_PORTA);
  servidorTCP.begin();
  Serial.println("\n--- Sistema pronto. Aguardando cliente no Estado Ocioso ---");
}

// =====================================================================================
// --- 8. LOOP PRINCIPAL (Executado continuamente no Núcleo 1, Prio 1) ---
// =====================================================================================
void loop() {
  // A 'loop()' (Prio 1) roda. Quando 'vTaskDelayUntil' acorda a 'taskColeta'
  // (Prio 5), ela interrompe o loop, coleta o dado, e volta a dormir,
  // devolvendo o controle para o loop. Esta é uma arquitetura estável.
  
  switch (g_estadoAtual) {
    case ESTADO_OCIOSO:
      gerenciarEstadoOcioso();
      break;
    case ESTADO_COLETANDO:
      gerenciarEstadoColeta();
      break;
    case ESTADO_ENVIANDO:
      gerenciarEstadoEnvio();
      break;
  }
}
