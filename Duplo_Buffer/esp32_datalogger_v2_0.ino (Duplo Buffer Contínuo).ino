/*
 * =====================================================================================
 *
 * Filename:  esp32_datalogger_v2_0.ino (Duplo Buffer Contínuo)
 *
 * Descrição:  Firmware V2.0 para Datalogger ESP32.
 * - Implementa a arquitetura de "streaming contínuo" com duplo buffer (ping-pong).
 * - Usa DOIS buffers menores (ex: 15KB cada) alocados na SRAM.
 * - Core 0 (Coleta): Enche um buffer a 1kHz e imediatamente troca para o outro.
 * Avisa o Core 1 qual buffer está pronto via Fila (Queue).
 * - Core 1 (Rede): Espera por notificações da Fila. Quando um buffer está
 * pronto, envia-o pela conexão TCP.
 * - Permite coleta de dados por tempo ILIMITADO.
 *
 * =====================================================================================
 */

// --- 1. BIBLIOTECAS ---
#include <WiFi.h>
#include "esp_heap_caps.h"
#include "freertos/queue.h" // Para comunicação entre Cores (Fila)

// --- 2. ESTRUTURA DE DADOS ---
struct PontoDeColeta {
  byte estadoSensor1;
  byte estadoSensor2;
  unsigned long timestamp_us;
} __attribute__((packed)); // 6 bytes exatos

// --- 3. DEFINIÇÕES E CONFIGURAÇÕES ---
const int FREQ_COLETA_HZ = 1000;
const long INTERVALO_COLETA_MS = 1000 / FREQ_COLETA_HZ; // 1ms

// -- Configurações do Duplo Buffer --
const int PONTOS_POR_BUFFER = 2560; // 2560 pontos * 6 bytes = 15360 bytes (15KB)
const int TAMANHO_UM_BUFFER_BYTES = PONTOS_POR_BUFFER * sizeof(PontoDeColeta); 

const int SENSOR_PIN_1 = 15;
const int SENSOR_PIN_2 = 16;
const int LED_STATUS_PIN = 2;
const char* WIFI_SSID = "ESP32-DATALOGGER";
const char* WIFI_SENHA = "senha1234";
const int TCP_PORTA = 80;

// --- 4. VARIÁVEIS GLOBAIS ---

// -- Buffers (Ponteiros) --
PontoDeColeta* g_bufferA;
PontoDeColeta* g_bufferB;

// -- Máquina de Estados (Simplificada para streaming) --
enum EstadoSistema { ESTADO_AGUARDANDO_CONEXAO, ESTADO_STREAMING };
volatile EstadoSistema g_estadoAtual = ESTADO_AGUARDANDO_CONEXAO;

// -- Rede --
WiFiServer servidorTCP(TCP_PORTA);
WiFiClient clienteTCP; 

// -- Sincronização Dual-Core --
TaskHandle_t h_taskColeta; // Handle da Tarefa no Core 0
// Fila para enviar ponteiros de buffer do Core 0 para o Core 1
QueueHandle_t filaBuffersProntos; 

// =====================================================================================
// --- 5. TAREFA DO NÚCLEO 0 (COLETA DE DADOS - Ping Pong) ---
// =====================================================================================
void taskColetaCore0(void* pvParameters) {
  Serial.println("Core 0: Tarefa de coleta (ping-pong) iniciada e pronta.");
  
  PontoDeColeta* bufferAtual = g_bufferA; // Começa enchendo o Buffer A
  int indiceBuffer = 0;

  const TickType_t xFrequency = pdMS_TO_TICKS(INTERVALO_COLETA_MS);
  TickType_t xLastWakeTime;

  // Espera o sistema principal sinalizar que uma conexão foi estabelecida
  while(g_estadoAtual != ESTADO_STREAMING){
      vTaskDelay(pdMS_TO_TICKS(100)); // Dorme por 100ms
  }
  
  Serial.println("Core 0: Cliente conectado! Iniciando coleta contínua a 1kHz...");
  xLastWakeTime = xTaskGetTickCount(); // Inicia o timer da coleta

  // Loop principal de coleta contínua
  for (;;) {
      // 1. Dorme até o próximo intervalo de 1ms
      vTaskDelayUntil(&xLastWakeTime, xFrequency);

      // 2. Acorda e coleta os dados no buffer atual
      bufferAtual[indiceBuffer].estadoSensor1 = digitalRead(SENSOR_PIN_1);
      bufferAtual[indiceBuffer].estadoSensor2 = digitalRead(SENSOR_PIN_2);
      bufferAtual[indiceBuffer].timestamp_us = micros();
      
      indiceBuffer++;

      // 3. Verifica se o buffer atual encheu
      if (indiceBuffer >= PONTOS_POR_BUFFER) {
          // Encheu!
          Serial.print("Core 0: Buffer ");
          Serial.print((bufferAtual == g_bufferA) ? 'A' : 'B');
          Serial.println(" cheio. Enviando para Core 1.");
          
          // 4. Envia o PONTEIRO do buffer cheio para a fila (sem bloquear)
          xQueueSend(filaBuffersProntos, &bufferAtual, 0); 
          
          // 5. Troca o buffer ativo (Ping-Pong)
          bufferAtual = (bufferAtual == g_bufferA) ? g_bufferB : g_bufferA;
          
          // 6. Reseta o índice para o novo buffer
          indiceBuffer = 0;
      }

      // Se o cliente desconectar, pausa a coleta
       if(g_estadoAtual != ESTADO_STREAMING){
           Serial.println("Core 0: Cliente desconectou. Pausando coleta.");
           while(g_estadoAtual != ESTADO_STREAMING){
               vTaskDelay(pdMS_TO_TICKS(100)); 
           }
           Serial.println("Core 0: Cliente reconectou! Retomando coleta...");
           xLastWakeTime = xTaskGetTickCount(); // Reinicia timer
           indiceBuffer = 0; // Reinicia buffer atual
       }
  }
}


// =====================================================================================
// --- 6. FUNÇÕES DE GERENCIAMENTO (NÚCLEO 1 - loop) ---
// =====================================================================================

void gerenciarAguardandoConexao() {
  digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN)); // Pisca LED azul (lento)
  delay(500);

  // Se já existe um cliente, mas ele desconectou
  if (clienteTCP && !clienteTCP.connected()) {
      Serial.println("Core 1 (loop): Cliente anterior desconectado.");
      clienteTCP.stop(); // Garante que foi fechado
  }
  
  // Verifica se um novo cliente está tentando conectar
  if (!clienteTCP || !clienteTCP.connected()) {
    clienteTCP = servidorTCP.available();
    if (clienteTCP && clienteTCP.connected()) {
      Serial.println("Core 1 (loop): Novo cliente TCP conectado!");
      Serial.println("Core 1 (loop): Mudando para ESTADO_STREAMING.");
      digitalWrite(LED_STATUS_PIN, HIGH); // LED Vermelho (sólido = Coletando/Streamando)
      g_estadoAtual = ESTADO_STREAMING; // Sinaliza para o Core 0 começar
    }
  }
}

void gerenciarStreaming() {
  // LED já está vermelho (HIGH)

  // 1. Verifica se o cliente ainda está conectado
  if (!clienteTCP.connected()) {
      Serial.println("Core 1 (loop): Cliente desconectou durante streaming.");
      clienteTCP.stop();
      g_estadoAtual = ESTADO_AGUARDANDO_CONEXAO; // Sinaliza para Core 0 pausar
      return;
  }

  // 2. Verifica se há um buffer pronto na fila vindo do Core 0
  PontoDeColeta* bufferParaEnviar = NULL;
  // Tenta ler da fila, esperando no máximo 10ms (não bloquear indefinidamente)
  if (xQueueReceive(filaBuffersProntos, &bufferParaEnviar, pdMS_TO_TICKS(10)) == pdPASS) {
      
      // Sim! Recebemos um ponteiro de buffer para enviar!
      Serial.print("Core 1 (loop): Recebido Buffer ");
      Serial.print((bufferParaEnviar == g_bufferA) ? 'A' : 'B');
      Serial.print(". Iniciando envio de ");
      Serial.print(TAMANHO_UM_BUFFER_BYTES / 1024.0, 1);
      Serial.println("KB...");

      // 3. Envia o buffer inteiro pela conexão TCP
      size_t bytesEnviados = clienteTCP.write((uint8_t*) bufferParaEnviar, TAMANHO_UM_BUFFER_BYTES);

      if (bytesEnviados == TAMANHO_UM_BUFFER_BYTES) {
          Serial.println("Core 1 (loop): Envio do buffer concluído com sucesso!");
      } else {
          Serial.print("Core 1 (loop): Erro no envio do buffer! Enviados ");
          Serial.print(bytesEnviados);
          Serial.print(" de ");
          Serial.println(TAMANHO_UM_BUFFER_BYTES);
          // Neste caso, provavelmente a conexão caiu, o estado mudará no próximo loop
      }
  }
  
  // 4. Se não há buffer pronto, o loop continua rapidamente,
  // permitindo que o sistema verifique a conexão TCP e outras tarefas.
}

// =====================================================================================
// --- 7. SETUP (Executado uma vez no Núcleo 1) ---
// =====================================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("--- Datalogger ESP32 V2.0 (Duplo Buffer Contínuo) ---");
  Serial.println("Core 0: Dedicado à Coleta de Dados (1kHz).");
  Serial.println("Core 1: Rodando Setup, Loop (Rede/Envio).");

  // --- 1. Alocar os DOIS Buffers ---
  Serial.print("Core 1: Alocando 2x ");
  Serial.print(TAMANHO_UM_BUFFER_BYTES / 1024.0, 1);
  Serial.println("KB de SRAM (ANTES do Wi-Fi)..."); 
  
  // Tenta alocar ambos no DMA Heap
  g_bufferA = (PontoDeColeta*) heap_caps_malloc(TAMANHO_UM_BUFFER_BYTES, MALLOC_CAP_DMA);
  g_bufferB = (PontoDeColeta*) heap_caps_malloc(TAMANHO_UM_BUFFER_BYTES, MALLOC_CAP_DMA);

  // Verificação robusta (poderia tentar alocar no 8BIT se DMA falhar)
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
  // A fila terá capacidade para 2 ponteiros (um para cada buffer)
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
// --- 8. LOOP PRINCIPAL (Executado continuamente no Núcleo 1, Prio 1) ---
// =====================================================================================
void loop() {
  // A 'loop()' (Prio 1) no Core 1 agora gerencia a rede e o envio de buffers.
  // A 'taskColeta' (Prio MAX) no Core 0 faz a coleta sem interrupção.
  
  switch (g_estadoAtual) {
    case ESTADO_AGUARDANDO_CONEXAO:
      gerenciarAguardandoConexao();
      break;
      
    case ESTADO_STREAMING:
      gerenciarStreaming();
      break;
  }
}
