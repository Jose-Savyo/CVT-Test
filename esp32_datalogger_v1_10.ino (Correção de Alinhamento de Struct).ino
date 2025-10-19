/*
 * =====================================================================================
 *
 * Filename:  esp32_datalogger_v1_10.ino (Correção de Alinhamento de Struct)
 *
 * Descrição:  Firmware V1.10 para Datalogger ESP32.
 * - Versão baseada na V1.9 (2kHz / 6s / 72KB).
 *
 * - *** CORREÇÃO CRÍTICA (V1.10) ***
 * Foi identificado que os dados no arquivo .bin estavam corrompidos.
 * Os valores dos sensores (deveriam ser 0/1) e os timestamps estavam caóticos.
 *
 * - CAUSA RAIZ: Padding de Compilador (Alinhamento de Memória).
 * A ESP32 é uma CPU de 32 bits e prefere ler dados de 4 bytes (como o
 * 'unsigned long' do timestamp) em endereços de memória múltiplos de 4.
 * Para forçar esse alinhamento, o compilador estava adicionando 2 bytes
 * de "preenchimento" (padding) invisíveis na nossa struct, fazendo-a
 * ter 8 BYTES na memória (1+1+[2 de padding]+4) em vez dos 6 BYTES
 * que calculamos (1+1+4).
 *
 * - EFEITO: Isso causou um "buffer overflow". Nosso código alocou 72.000 bytes
 * (12.000 * 6), mas o loop 'for', ao usar a struct de 8 bytes, tentou
 * escrever 96.000 bytes (12.000 * 8), corrompendo a memória.
 *
 * - A SOLUÇÃO: Adicionar '__attribute__((packed))' à definição da struct.
 * Isso força o compilador a remover os bytes de padding e criar a
 * struct com exatamente 6 bytes, alinhando nosso cálculo de memória
 * com a realidade do hardware.
 *
 * =====================================================================================
 *
 * --- NOTA DE ARQUITETURA E COMPROMISSO DE PROJETO (V1.9 / V1.10) ---
 *
 * 1. O PROBLEMA (LIMITE DE MEMÓRIA SRAM):
 * ... (O resto das notas de arquitetura permanecem as mesmas) ...
 * CÁLCULO ATUAL: 2.000 Hz * 6 Segundos = 12.000 Pontos (Buffer de 72KB)
 * ALERTA DE RISCO: Coletar a 2kHz para um sensor de 1kHz está no limite
 * exato de Nyquist.
 *
 * 2. AS FUTURAS SOLUÇÕES ROBUSTAS:
 * A) SOLUÇÃO DE HARDWARE: Usar uma ESP32-WROVER (com PSRAM).
 * B) SOLUÇÃO DE SOFTWARE: Mudar para um "streaming de buffer duplo (ping-pong)".
 *
 * =====================================================================================
 */

// --- 1. BIBLIOTECAS ---
#include <WiFi.h>
#include "esp_heap_caps.h"

// --- 2. ESTRUTURA DE DADOS ---
//
// *** INÍCIO DA CORREÇÃO V1.10 ***
//
// Adicionamos '__attribute__((packed))' para forçar o compilador
// a criar esta struct com exatamente 6 bytes (1+1+4) e remover
// os 2 bytes de "padding" (preenchimento) que estavam sendo
// adicionados automaticamente para alinhamento de 32 bits.
// Isso resolve a corrupção de dados.
//
struct PontoDeColeta {
  byte estadoSensor1;      // 1 byte
  byte estadoSensor2;      // 1 byte
  unsigned long timestamp_us; // 4 bytes
} __attribute__((packed));
//
// *** FIM DA CORREÇÃO V1.10 ***
//

// --- 3. DEFINIÇÕES E CONFIGURAÇÕES ---
const int FREQ_COLETA_HZ = 2000;
const int TEMPO_ENSAIO_S = 6; 
const long INTERVALO_COLETA_US = 1000000 / FREQ_COLETA_HZ; // 500µs

const int TAMANHO_BUFFER_PONTOS = FREQ_COLETA_HZ * TEMPO_ENSAIO_S; // 12.000
// Agora o sizeof() reportará 6 bytes, e nosso cálculo estará correto.
const int TAMANHO_BUFFER_BYTES = TAMANHO_BUFFER_PONTOS * sizeof(PontoDeColeta); // 72.000 bytes

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
    Serial.println("Core 1: Sinal recebido! Iniciando coleta de 6s...");
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

// =====================================================================================
// --- 6. FUNÇÕES DE GERENCIAMENTO (NÚCLEO 0) ---
// =====================================================================================
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
  Serial.print(TAMANHO_BUFFER_BYTES / 1024.0, 2); // Imprime 72.0KB
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
  Serial.println("--- Datalogger ESP32 Dual-Core V1.10 (Packed Struct) ---");

  // --- 1. Alocar Memória PRIMEIRO! ---
  Serial.print("Core 0: Alocando 72KB de SRAM (ANTES do Wi-Fi)..."); 
  
  g_dataBuffer = (PontoDeColeta*) heap_caps_malloc(TAMANHO_BUFFER_BYTES, MALLOC_CAP_DMA);

  if (g_dataBuffer == NULL) {
    Serial.println("\nFALHA CRITICA! Nao foi possivel alocar o buffer DMA.");
    Serial.println("Tentando alocar em memoria comum (pode falhar)...");
    g_dataBuffer = (PontoDeColeta*) heap_caps_malloc(TAMANHO_BUFFER_BYTES, MALLOC_CAP_8BIT);
  }

  if (g_dataBuffer == NULL) {
    Serial.print("\nFALHA CRITICA! Impossivel alocar 72KB."); 
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
