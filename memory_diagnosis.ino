/*
 * =====================================================================================
 *
 * Filename:  esp32_datalogger_v1_4.ino (Versão de Diagnóstico de Memória)
 *
 * Descrição:  Este firmware não tenta alocar o buffer. Em vez disso, ele
 * inicializa o Wi-Fi (que consome memória) e depois imprime um relatório
 * detalhado sobre a memória heap disponível.
 *
 * OBJETIVO: Descobrir qual o maior bloco de memória contíguo que
 * podemos alocar com segurança.
 *
 * =====================================================================================
 */

// --- 1. BIBLIOTECAS ---
#include <WiFi.h>
#include "esp_heap_caps.h" // Para diagnóstico e alocação

// --- 2. ESTRUTURA DE DADOS ---
struct PontoDeColeta {
  byte estadoSensor1;
  byte estadoSensor2;
  unsigned long timestamp_us;
}; // Total: 6 bytes

// --- 3. DEFINIÇÕES E CONFIGURAÇÕES ---
// Configurações do ensaio (temporariamente desabilitadas para teste)
// const int FREQ_COLETA_HZ = 3000;
// const int TEMPO_ENSAIO_S = 10; 
// const int TAMANHO_BUFFER_PONTOS = FREQ_COLETA_HZ * TEMPO_ENSAIO_S; 
// const int TAMANHO_BUFFER_BYTES = TAMANHO_BUFFER_PONTOS * sizeof(PontoDeColeta); 

const int SENSOR_PIN_1 = 15;
const int SENSOR_PIN_2 = 16;
const int LED_STATUS_PIN = 2;

const char* WIFI_SSID = "ESP32-DATALOGGER";
const char* WIFI_SENHA = "senha1234";
const int TCP_PORTA = 80;

// --- 4. VARIÁVEIS GLOBAIS ---
// Não vamos alocar o buffer ainda
// PontoDeColeta* g_dataBuffer; 

enum EstadoSistema { ESTADO_OCIOSO, ESTADO_COLETANDO, ESTADO_ENVIANDO };
volatile EstadoSistema g_estadoAtual = ESTADO_OCIOSO;
WiFiServer servidorTCP(TCP_PORTA);
WiFiClient clienteTCP;
TaskHandle_t h_taskColeta;
SemaphoreHandle_t sem_inicioColeta;
volatile bool g_coletaConcluida = false;

// =====================================================================================
// --- 5. TAREFA DO NÚCLEO 1 (COLETA DE DADOS) ---
// (A tarefa é criada, mas não fará nada de útil sem o buffer)
// =====================================================================================
void taskColetaCore1(void* pvParameters) {
  Serial.println("Core 1: Tarefa de coleta iniciada (modo diagnóstico).");
  for (;;) {
    // Apenas dorme, pois não há buffer para preencher
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// =====================================================================================
// --- 6. FUNÇÕES DE GERENCIAMENTO (NÚCLEO 0) ---
// (Não serão usadas neste teste)
// =====================================================================================

// =====================================================================================
// --- 7. SETUP (Executado uma vez no Núcleo 0) ---
// =====================================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("--- Datalogger ESP32 Dual-Core V1.4 (Diagnóstico de Memória) ---");

  // --- 1. Configurar Pinos ---
  pinMode(LED_STATUS_PIN, OUTPUT);
  pinMode(SENSOR_PIN_1, INPUT_PULLUP);
  pinMode(SENSOR_PIN_2, INPUT_PULLUP);

  // --- 2. Iniciar a Tarefa no Core 1 ---
  // (Ainda precisamos iniciar para simular o consumo de memória dela)
  sem_inicioColeta = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(
      taskColetaCore1, "TaskColeta", 10000, NULL, 1, &h_taskColeta, 1);
  
  // --- 3. Configurar Wi-Fi (Consome memória) ---
  Serial.print("Core 0: Configurando Access Point Wi-Fi (para simular uso de memória)...");
  WiFi.softAP(WIFI_SSID, WIFI_SENHA);
  Serial.println("OK!");

  // --- 4. RELATÓRIO DE MEMÓRIA (A PARTE IMPORTANTE) ---
  Serial.println("\n--- RELATORIO DE MEMORIA (APOS INICIAR WI-FI E TAREFAS) ---");

  // Imprime informações sobre a memória interna (SRAM)
  Serial.println("--- Memoria SRAM Interna (8BIT) ---");
  heap_caps_print_heap_info(MALLOC_CAP_8BIT);
  size_t maiorBloco8Bit = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  Serial.print("Maior Bloco Contiguo (8BIT): ");
  Serial.print(maiorBloco8Bit);
  Serial.println(" bytes");

  // Imprime informações sobre a memória DMA (parte especial da SRAM)
  Serial.println("\n--- Memoria SRAM-DMA (DMA) ---");
  heap_caps_print_heap_info(MALLOC_CAP_DMA);
  size_t maiorBlocoDMA = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
  Serial.print("Maior Bloco Contiguo (DMA): ");
  Serial.print(maiorBlocoDMA);
  Serial.println(" bytes");

  Serial.println("\n--- Conclusao do Diagnostico ---");
  size_t maxAlocavel = (maiorBlocoDMA > maiorBloco8Bit) ? maiorBlocoDMA : maiorBloco8Bit;
  Serial.print("Tamanho maximo que podemos alocar (aprox.): ");
  Serial.print(maxAlocavel - 4096); // Deixa uma margem de segurança de 4KB
  Serial.println(" bytes");
  Serial.println("Envie esta saida para o assistente.");
  Serial.println("-------------------------------------\n");

  // Trava o código aqui, pois o teste terminou.
  while (true) {
    digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
    delay(100);
  }
}
// =====================================================================================
// --- 8. LOOP PRINCIPAL (Não será executado) ---
// =====================================================================================
void loop() {
  // Nada aqui
}
