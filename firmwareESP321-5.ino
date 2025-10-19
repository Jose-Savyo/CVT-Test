/*
 * =====================================================================================
 *
 * Filename:  esp32_datalogger_v1_7.ino (Versão "Alocar ANTES do Wi-Fi")
 *
 * Descrição:  Firmware V1.7 para Datalogger ESP32.
 * - As alocações anteriores falharam, mesmo com 90KB.
 * - A hipótese é que o Wi-Fi, ao ser iniciado primeiro, fragmenta a memória.
 * - Esta versão inverte a ordem: PRIMEIRO, aloca os 90KB de memória.
 * - SEGUNDO, inicia o Wi-Fi e o restante do sistema.
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
}; // 6 bytes

// --- 3. DEFINIÇÕES E CONFIGURAÇÕES ---
// Vamos manter os 90KB (5 segundos) da V1.6
const int FREQ_COLETA_HZ = 3000;
const int TEMPO_ENSAIO_S = 5; 
const long INTERVALO_COLETA_US = 1000000 / FREQ_COLETA_HZ;
const int TAMANHO_BUFFER_PONTOS = FREQ_COLETA_HZ * TEMPO_ENSAIO_S; // 15.000
const int TAMANHO_BUFFER_BYTES = TAMANHO_BUFFER_PONTOS * sizeof(PontoDeColeta); // 90.000

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
// (Sem mudanças)
void taskColetaCore1(void* pvParameters) {
  Serial.println("Core 1: Tarefa de coleta iniciada e pronta.");
  for (;;) {
    xSemaphoreTake(sem_inicioColeta, portMAX_DELAY);
    Serial.println("Core 1: Sinal recebido! Iniciando coleta de 5s...");
    unsigned long tempoProximaColeta = micros();
    for (int i = 0; i < TAMANHO_BUFFER_PONTOS; i++) {
      g_dataBuffer[i].estadoSensor1 = digitalRead(SENSOR_PIN_1);
      g_dataBuffer[i].estadoSensor2 = digitalRead(SENSOR_PIN_2);
      g_dataBuffer[i].timestamp_us = micros();
      tempoProximaColeta += INTERVALO_COLETA_US;
      while (micros() < tempoProximaColeta) {}
    }
    Serial.println("Core 1: Coleta concluída! Avisando Core 0.");
    g_coletaConcluida = true;
  }
}

// --- 6. FUNÇÕES DE GERENCIAMENTO (NÚCLEO 0) ---
// (Sem mudanças)
void gerenciarEstadoOcioso() {
  digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
  delay(500);
  if (!clienteTCP.connected()) {
    clienteTCP = servidorTCP.available();
    if (clienteTCP) { Serial.println("Core 0: Cliente TCP conectado!"); }
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
      Serial.print("Core 0: Comando '"); Serial.print(cmd); Serial.println("' desconhecido.");
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
  Serial.print("Core 0: Iniciando envio de ");
  Serial.print(TAMANHO_BUFFER_BYTES / 1024.0, 2);
  Serial.println("KB de dados...");
  digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
  delay(100);
  if (clienteTCP.connected()) {
    size_t bytesEnviados = clienteTCP.write((uint8_t*) g_dataBuffer, TAMANHO_BUFFER_BYTES);
    if (bytesEnviados == TAMANHO_BUFFER_BYTES) {
      Serial.println("Core 0: Envio concluído com sucesso!");
    } else {
      Serial.print("Core 0: Erro no envio! Enviados ");
      Serial.print(bytesEnviados); Serial.print(" de "); Serial.println(TAMANHO_BUFFER_BYTES);
    }
  } else {
    Serial.println("Core 0: Erro! Cliente desconectou antes do envio.");
  }
  clienteTCP.stop();
  Serial.println("Core 0: Cliente desconectado. Voltando ao estado ocioso.");
  g_estadoAtual = ESTADO_OCIOSO;
}

// =====================================================================================
// --- 7. SETUP (Executado uma vez no Núcleo 0) ---
// =====================================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("--- Datalogger ESP32 Dual-Core V1.7 (Aloca ANTES do Wi-Fi) ---");

  // --- 1. MUDANÇA PRINCIPAL: Alocar Memória PRIMEIRO! ---
  Serial.print("Core 0: Alocando 90KB de SRAM (ANTES do Wi-Fi)...");
  
  g_dataBuffer = (PontoDeColeta*) heap_caps_malloc(TAMANHO_BUFFER_BYTES, MALLOC_CAP_DMA);

  if (g_dataBuffer == NULL) {
    Serial.println("\nFALHA CRITICA! Nao foi possivel alocar o buffer DMA.");
    Serial.println("Tentando alocar em memoria comum (pode falhar)...");
    g_dataBuffer = (PontoDeColeta*) heap_caps_malloc(TAMANHO_BUFFER_BYTES, MALLOC_CAP_8BIT);
  }

  if (g_dataBuffer == NULL) {
    Serial.print("\nFALHA CRITICA! Impossivel alocar 90KB.");
    pinMode(LED_STATUS_PIN, OUTPUT);
    while (true) {
      digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
      delay(50);
    }
  }
  Serial.println("Sucesso! Memória alocada.");

  // --- 2. Configurar Pinos ---
  pinMode(LED_STATUS_PIN, OUTPUT);
  pinMode(SENSOR_PIN_1, INPUT_PULLUP);
  pinMode(SENSOR_PIN_2, INPUT_PULLUP);

  // --- 3. Criar Semáforo de Sincronização ---
  sem_inicioColeta = xSemaphoreCreateBinary();

  // --- 4. Iniciar a Tarefa no Core 1 ---
  xTaskCreatePinnedToCore(
      taskColetaCore1, "TaskColeta", 10000, NULL, 1, &h_taskColeta, 1);
  
  // --- 5. Configurar Wi-Fi e Rede (AGORA POR ÚLTIMO) ---
  Serial.print("Core 0: Configurando Access Point Wi-Fi (DEPOIS da alocação)...");
  WiFi.softAP(WIFI_SSID, WIFI_SENHA);
  Serial.print("OK! IP do AP: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("Core 0: Iniciando Servidor TCP na porta ");
  Serial.println(TCP_PORTA);
  servidorTCP.begin();
  Serial.println("\n--- Sistema pronto. Aguardando cliente no Estado Ocioso ---");
}

// =====================================================================================
// --- 8. LOOP PRINCIPAL (Executado continuamente no Núcleo 0) ---
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
