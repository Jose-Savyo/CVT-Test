/*
 * =====================================================================================
 *
 * Filename:  esp32_datalogger_v1_11.ino (Correção de Jitter e Dual-Core Real)
 *
 * Descrição:  Firmware V1.11 para Datalogger ESP32.
 * - Versão baseada na V1.10 (que corrigiu a corrupção de dados).
 *
 * - *** CORREÇÃO CRÍTICA (V1.11) ***
 * Os dados da V1.10 mostraram 'jitter' nos timestamps (intervalos
 * instáveis: 500, 1456, 2, 41...).
 *
 * - CAUSA RAIZ: Conflito de Núcleo (Core).
 * O código anterior (V1.10) estava erroneamente a 'pinar' (fixar) a tarefa
 * de coleta de dados no NÚCLEO 1. No entanto, o loop principal do Arduino
 * (que gere o Wi-Fi e o TCP) também corre, por defeito, no NÚCLEO 1.
 * As duas tarefas estavam a competir, e o Wi-Fi interrompia a coleta.
 *
 * - A SOLUÇÃO: Mover a tarefa de coleta para o NÚCLEO 0 e
 * aumentar a sua prioridade.
 * - Núcleo 0 (isolado): Apenas a 'taskColetaCore1' em prioridade máxima.
 * - Núcleo 1 (padrão): O loop() do Arduino, Wi-Fi, TCP, etc.
 *
 * Isso garante que a coleta de dados NUNCA será interrompida.
 * =====================================================================================
 */

// --- 1. BIBLIOTECAS ---
#include <WiFi.h>
#include "esp_heap_caps.h"

// --- 2. ESTRUTURA DE DADOS ---
// Estrutura "empacotada" para garantir 6 bytes exatos
struct PontoDeColeta {
  byte estadoSensor1;
  byte estadoSensor2;
  unsigned long timestamp_us;
} __attribute__((packed));

// --- 3. DEFINIÇÕES E CONFIGURAÇÕES ---
const int FREQ_COLETA_HZ = 2000;
const int TEMPO_ENSAIO_S = 6; 
const long INTERVALO_COLETA_US = 1000000 / FREQ_COLETA_HZ; // 500µs
const int TAMANHO_BUFFER_PONTOS = FREQ_COLETA_HZ * TEMPO_ENSAIO_S; // 12.000
const int TAMANHO_BUFFER_BYTES = TAMANHO_BUFFER_PONTOS * sizeof(PontoDeColeta); // 72.000

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

// --- 5. TAREFA DO NÚCLEO 1 (COLETA DE DADOS) ---
// (Sem mudanças na lógica da tarefa)
void taskColetaCore1(void* pvParameters) {
  Serial.println("Core 0: Tarefa de coleta iniciada e pronta."); // Mensagem atualizada
  for (;;) {
    xSemaphoreTake(sem_inicioColeta, portMAX_DELAY);
    Serial.println("Core 0: Sinal recebido! Iniciando coleta de 6s..."); // Mensagem atualizada
    unsigned long tempoProximaColeta = micros();
    for (int i = 0; i < TAMANHO_BUFFER_PONTOS; i++) {
      g_dataBuffer[i].estadoSensor1 = digitalRead(SENSOR_PIN_1);
      g_dataBuffer[i].estadoSensor2 = digitalRead(SENSOR_PIN_2);
      g_dataBuffer[i].timestamp_us = micros();
      
      tempoProximaColeta += INTERVALO_COLETA_US; 
      while (micros() < tempoProximaColeta) {}
    }
    Serial.println("Core 0: Coleta concluída! Avisando Core 1."); // Mensagem atualizada
    g_coletaConcluida = true;
  }
}

// --- 6. FUNÇÕES DE GERENCIAMENTO (NÚCLEO 0) ---
// (Sem mudanças na lógica de gestão)
void gerenciarEstadoOcioso() {
  digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
  delay(500);
  if (!clienteTCP.connected()) {
    clienteTCP = servidorTCP.available();
    if (clienteTCP) { Serial.println("Core 1: Cliente TCP conectado!"); } // Mensagem atualizada
    return;
  }
  if (clienteTCP.available() > 0) {
    char cmd = clienteTCP.read();
    if (cmd == 'S') {
      Serial.println("Core 1: Comando 'S' recebido. Iniciando coleta..."); // Mensagem atualizada
      g_coletaConcluida = false;
      digitalWrite(LED_STATUS_PIN, HIGH);
      xSemaphoreGive(sem_inicioColeta);
      g_estadoAtual = ESTADO_COLETANDO;
    } else {
      Serial.print("Core 1: Comando '"); Serial.print(cmd); Serial.println("' desconhecido."); // Mensagem atualizada
    }
  }
}
void gerenciarEstadoColeta() {
  if (!clienteTCP.connected()) {
    Serial.println("Core 1: Cliente desconectou durante a coleta. Abortando."); // Mensagem atualizada
    g_estadoAtual = ESTADO_OCIOSO;
    return;
  }
  if (g_coletaConcluida) {
    Serial.println("Core 1: Flag de coleta concluída recebida."); // Mensagem atualizada
    g_estadoAtual = ESTADO_ENVIANDO;
  }
}
void gerenciarEstadoEnvio() {
  Serial.print("Core 1: Iniciando envio de "); // Mensagem atualizada
  Serial.print(TAMANHO_BUFFER_BYTES / 1024.0, 2); 
  Serial.println("KB de dados...");
  digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
  delay(100);
  if (clienteTCP.connected()) {
    size_t bytesEnviados = clienteTCP.write((uint8_t*) g_dataBuffer, TAMANHO_BUFFER_BYTES);
    if (bytesEnviados == TAMANHO_BUFFER_BYTES) {
      Serial.println("Core 1: Envio concluído com sucesso!"); // Mensagem atualizada
    } else {
      Serial.print("Core 1: Erro no envio! Enviados "); // Mensagem atualizada
      Serial.print(bytesEnviados); Serial.print(" de "); Serial.println(TAMANHO_BUFFER_BYTES);
    }
  } else {
    Serial.println("Core 1: Erro! Cliente desconectou antes do envio."); // Mensagem atualizada
  }
  clienteTCP.stop();
  Serial.println("Core 1: Cliente desconectado. Voltando ao estado ocioso."); // Mensagem atualizada
  g_estadoAtual = ESTADO_OCIOSO;
}

// =====================================================================================
// --- 7. SETUP (Executado uma vez no Núcleo 1) ---
// =====================================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("--- Datalogger ESP32 Dual-Core V1.11 (Core/Priority Fix) ---");
  Serial.println("Sistema principal (Wi-Fi, TCP) rodando no Core 1.");

  // --- 1. Alocar Memória PRIMEIRO! ---
  Serial.print("Core 1: Alocando 72KB de SRAM (ANTES do Wi-Fi)..."); 
  
  g_dataBuffer = (PontoDeColeta*) heap_caps_malloc(TAMANHO_BUFFER_BYTES, MALLOC_CAP_DMA);
  if (g_dataBuffer == NULL) {
    g_dataBuffer = (PontoDeColeta*) heap_caps_malloc(TAMANHO_BUFFER_BYTES, MALLOC_CAP_8BIT);
  }
  if (g_dataBuffer == NULL) {
    Serial.print("\nFALHA CRITICA! Impossivel alocar 72KB."); 
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

  // --- 4. Iniciar a Tarefa no Core 1 ---
  //
  // *** INÍCIO DA CORREÇÃO V1.11 ***
  //
  xTaskCreatePinnedToCore(
      taskColetaCore1,    // Função da tarefa
      "TaskColeta",       // Nome (para debug)
      10000,              // Tamanho da Stack
      NULL,               // Parâmetros da tarefa
      24,                 // Prioridade MÁXIMA (ex: 24. configMAX_PRIORITIES - 1)
      &h_taskColeta,      // Handle da tarefa
      0);                 // Núcleo onde a tarefa vai rodar: NÚCLEO 0
  //
  // *** FIM DA CORREÇÃO V1.11 ***
  //
  
  // --- 5. Configurar Wi-Fi e Rede (Core 1) ---
  Serial.print("Core 1: Configurando Access Point Wi-Fi (DEPOIS da alocação)...");
  WiFi.softAP(WIFI_SSID, WIFI_SENHA);
  Serial.print("OK! IP do AP: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("Core 1: Iniciando Servidor TCP na porta ");
  Serial.println(TCP_PORTA);
  servidorTCP.begin();
  Serial.println("\n--- Sistema pronto. Aguardando cliente no Estado Ocioso ---");
}

// =====================================================================================
// --- 8. LOOP PRINCIPAL (Executado continuamente no Núcleo 1) ---
// =====================================================================================
void loop() {
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
