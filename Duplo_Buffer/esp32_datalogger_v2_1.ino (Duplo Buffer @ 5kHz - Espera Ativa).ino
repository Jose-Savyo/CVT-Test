/*
 * =====================================================================================
 *
 * Filename:  esp32_datalogger_v2_1.ino (Duplo Buffer @ 5kHz - Espera Ativa)
 *
 * Descrição:  Firmware V2.1 para Datalogger ESP32.
 * - Baseado na arquitetura V2.0 (Duplo Buffer Contínuo).
 * - Aumenta a frequência de coleta para 5kHz.
 * - Usa loop de espera ativa ("busy-wait" com micros()) para temporização,
 * pois 5kHz (200µs) é muito rápido para vTaskDelayUntil (baseado em 1ms).
 * - Mantém os buffers de 15KB (2560 pontos), resultando em T_fill = 512ms.
 * - Arquitetura Dual-Core mantida (Core 0 para Coleta, Core 1 para Rede).
 *
 * =====================================================================================
 *
 * --- NOTA DE ARQUITETURA E COMPROMISSO DE PROJETO (V2.1) ---
 *
 * 1. O OBJETIVO: Aumentar a frequência de coleta para 5kHz, mantendo a
 * arquitetura de streaming contínuo (duplo buffer).
 *
 * 2. O DESAFIO DE TEMPORIZAÇÃO:
 * A V2.0 usava 'vTaskDelayUntil()' para garantir 1kHz sem jitter. No entanto,
 * essa função opera com "ticks" de 1ms (1000µs) no FreeRTOS.
 * A nova frequência de 5kHz exige um intervalo de 200µs, que é muito
 * mais rápido que o tick do sistema.
 *
 * 3. A SOLUÇÃO ATUAL (V2.1 - ESPERA ATIVA):
 * Para esta versão, implementamos a temporização usando um loop de
 * "espera ativa" (busy-wait) baseado na função 'micros()'.
 * CÓDIGO: 'while (micros() < tempoProximaColeta) {}'
 *
 * VANTAGEM: Simples de implementar com funções Arduino padrão.
 * DESVANTAGEM (POTENCIAL):
 * - Consumo de CPU: O Core 0 ficará 100% ocupado na espera.
 * - Jitter: A precisão da temporização depende da velocidade do loop
 * e da resolução do micros(). Espera-se um jitter pequeno, mas
 * maior que o da V2.0. É necessário analisar os dados do CSV
 * para verificar se a estabilidade é aceitável.
 *
 * 4. A FUTURA SOLUÇÃO MAIS ROBUSTA (V2.2+):
 * Se a análise dos dados da V2.1 mostrar um jitter inaceitável, ou se
 * a máxima precisão temporal for desejada, a próxima etapa será
 * reimplementar a temporização usando os **Timers de Hardware** da ESP32.
 * Isso garante precisão na casa dos nanossegundos e é a solução
 * tecnicamente superior para tempo real, embora mais complexa.
 *
 * =====================================================================================
 */

// --- 1. BIBLIOTECAS ---
#include <WiFi.h>
#include "esp_heap_caps.h"
#include "freertos/queue.h"

// --- 2. ESTRUTURA DE DADOS ---
struct PontoDeColeta {
  byte estadoSensor1;
  byte estadoSensor2;
  unsigned long timestamp_us;
} __attribute__((packed)); // 6 bytes exatos

// --- 3. DEFINIÇÕES E CONFIGURAÇÕES ---
// MUDANÇA PRINCIPAL AQUI - FREQUÊNCIA 5kHz
const int FREQ_COLETA_HZ = 5000;
const long INTERVALO_COLETA_US = 1000000 / FREQ_COLETA_HZ; // Agora 200µs

// Mantemos o tamanho do buffer (15KB)
const int PONTOS_POR_BUFFER = 2560; 
const int TAMANHO_UM_BUFFER_BYTES = PONTOS_POR_BUFFER * sizeof(PontoDeColeta); 

const int SENSOR_PIN_1 = 15;
const int SENSOR_PIN_2 = 16;
const int LED_STATUS_PIN = 2;
const char* WIFI_SSID = "ESP32-DATALOGGER";
const char* WIFI_SENHA = "senha1234";
const int TCP_PORTA = 80;

// --- 4. VARIÁVEIS GLOBAIS ---
PontoDeColeta* g_bufferA;
PontoDeColeta* g_bufferB;
enum EstadoSistema { ESTADO_AGUARDANDO_CONEXAO, ESTADO_STREAMING };
volatile EstadoSistema g_estadoAtual = ESTADO_AGUARDANDO_CONEXAO;
WiFiServer servidorTCP(TCP_PORTA);
WiFiClient clienteTCP; 
TaskHandle_t h_taskColeta; 
QueueHandle_t filaBuffersProntos; 

// =====================================================================================
// --- 5. TAREFA DO NÚCLEO 0 (COLETA DE DADOS @ 5kHz - Espera Ativa) ---
// =====================================================================================
void taskColetaCore0(void* pvParameters) {
  Serial.println("Core 0: Tarefa de coleta (ping-pong @ 5kHz) iniciada.");
  
  PontoDeColeta* bufferAtual = g_bufferA;
  int indiceBuffer = 0;
  unsigned long tempoProximaColeta; // Timer para espera ativa

  // Espera o sistema principal sinalizar conexão
  while(g_estadoAtual != ESTADO_STREAMING){
      vTaskDelay(pdMS_TO_TICKS(100)); 
  }
  
  Serial.println("Core 0: Cliente conectado! Iniciando coleta contínua a 5kHz...");
  tempoProximaColeta = micros(); // Inicia o timer da coleta

  // Loop principal de coleta contínua
  for (;;) {
      // 1. Coleta os dados no buffer atual
      bufferAtual[indiceBuffer].estadoSensor1 = digitalRead(SENSOR_PIN_1);
      bufferAtual[indiceBuffer].estadoSensor2 = digitalRead(SENSOR_PIN_2);
      bufferAtual[indiceBuffer].timestamp_us = micros(); // Grava tempo ANTES da espera
      
      indiceBuffer++;

      // 2. Verifica se o buffer atual encheu
      if (indiceBuffer >= PONTOS_POR_BUFFER) {
          Serial.print("Core 0: Buffer ");
          Serial.print((bufferAtual == g_bufferA) ? 'A' : 'B');
          Serial.println(" cheio. Enviando para Core 1.");
          
          xQueueSend(filaBuffersProntos, &bufferAtual, 0); 
          bufferAtual = (bufferAtual == g_bufferA) ? g_bufferB : g_bufferA;
          indiceBuffer = 0;
      }

      // 3. Loop de espera ativa ("busy-wait") para garantir a frequência de 5kHz
      // Atualiza o alvo para o próximo intervalo de 200µs
      tempoProximaColeta += INTERVALO_COLETA_US; 
      // Espera ativamente até o tempo alvo chegar
      while (micros() < tempoProximaColeta) {
        // Core 0 fica 100% ocupado aqui
      }

      // Se o cliente desconectar, pausa a coleta
       if(g_estadoAtual != ESTADO_STREAMING){
           Serial.println("Core 0: Cliente desconectou. Pausando coleta.");
           while(g_estadoAtual != ESTADO_STREAMING){
               vTaskDelay(pdMS_TO_TICKS(100)); 
           }
           Serial.println("Core 0: Cliente reconectou! Retomando coleta...");
           tempoProximaColeta = micros(); // Reinicia timer
           indiceBuffer = 0; 
       }
  }
}

// =====================================================================================
// --- 6. FUNÇÕES DE GERENCIAMENTO (NÚCLEO 1 - loop) ---
// (Sem mudanças na lógica, apenas mensagens atualizadas se necessário)
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
      g_estadoAtual = ESTADO_STREAMING; 
    }
  }
}

void gerenciarStreaming() {
  if (!clienteTCP.connected()) {
      Serial.println("Core 1 (loop): Cliente desconectou durante streaming.");
      clienteTCP.stop();
      g_estadoAtual = ESTADO_AGUARDANDO_CONEXAO; 
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
// --- 7. SETUP (Executado uma vez no Núcleo 1) ---
// =====================================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("--- Datalogger ESP32 V2.1 (Duplo Buffer @ 5kHz Espera Ativa) ---");
  Serial.println("Core 0: Dedicado à Coleta de Dados.");
  Serial.println("Core 1: Rodando Setup, Loop (Rede/Envio).");

  // --- 1. Alocar os DOIS Buffers ---
  Serial.print("Core 1: Alocando 2x ");
  Serial.print(TAMANHO_UM_BUFFER_BYTES / 1024.0, 1);
  Serial.println("KB de SRAM (ANTES do Wi-Fi)..."); 
  
  g_bufferA = (PontoDeColeta*) heap_caps_malloc(TAMANHO_UM_BUFFER_BYTES, MALLOC_CAP_DMA);
  g_bufferB = (PontoDeColeta*) heap_caps_malloc(TAMANHO_UM_BUFFER_BYTES, MALLOC_CAP_DMA);

  if (g_bufferA == NULL || g_bufferB == NULL) {
    Serial.print("\nFALHA CRITICA! Impossivel alocar os dois buffers (");
    Serial.print(TAMANHO_UM_BUFFER_BYTES * 2 / 1024.0, 1);
    Serial.println("KB total)."); 
    pinMode(LED_STATUS_PIN, OUTPUT);
    while (true) { digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN)); delay(50); }
  }
  Serial.println("Sucesso! Buffers alocados.");

  // --- 2. Configurar Pinos ---
  pinMode(LED_STATUS_PIN, OUTPUT);
  pinMode(SENSOR_PIN_1, INPUT_PULLUP);
  pinMode(SENSOR_PIN_2, INPUT_PULLUP);

  // --- 3. Criar a Fila de Comunicação ---
  filaBuffersProntos = xQueueCreate(2, sizeof(PontoDeColeta*)); 

  // --- 4. Iniciar a Tarefa de Coleta no Core 0 ---
  xTaskCreatePinnedToCore(
      taskColetaCore0,    // Função
      "TaskColeta",       // Nome
      10000,              // Stack
      NULL,               // Parâmetros
      configMAX_PRIORITIES - 1, // Prioridade MÁXIMA
      &h_taskColeta,      // Handle
      0);                 // Núcleo: CORE 0 (Isolado)
  
  // --- 5. Configurar Wi-Fi e Rede (Core 1) ---
  Serial.print("Core 1: Configurando Access Point Wi-Fi...");
  WiFi.softAP(WIFI_SSID, WIFI_SENHA);
  Serial.print("OK! IP do AP: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("Core 1: Iniciando Servidor TCP na porta ");
  Serial.println(TCP_PORTA);
  servidorTCP.begin();
  Serial.println("\n--- Sistema pronto. Aguardando cliente no Estado Aguardando Conexão ---");
}

// =====================================================================================
// --- 8. LOOP PRINCIPAL (Executado continuamente no Núcleo 1) ---
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
