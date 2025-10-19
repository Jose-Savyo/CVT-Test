/*
 * =====================================================================================
 *
 * Filename:  esp32_datalogger_v1_1.ino (Versão Corrigida)
 *
 * Descrição:  Firmware V1.1 para Datalogger de Alta Frequência em ESP32.
 * - Corrige o erro de compilação 'PontoDeColeta was not declared in this scope'
 * movendo a definição da struct para antes de seu uso.
 *
 * =====================================================================================
 */

// --- 1. BIBLIOTECAS ---
#include <WiFi.h>

// --- 2. ESTRUTURA DE DADOS (MOVIDA PARA CIMA) ---
// A estrutura de dados deve ser definida ANTES de ser usada por 'sizeof()'
struct PontoDeColeta {
  byte estadoSensor1;      // 1 byte
  byte estadoSensor2;      // 1 byte
  unsigned long timestamp_us; // 4 bytes
}; // Total: 6 bytes

// --- 3. DEFINIÇÕES E CONFIGURAÇÕES ---

// -- Configurações do Ensaio --
const int FREQ_COLETA_HZ = 3000;
const int TEMPO_ENSAIO_S = 20;
const long INTERVALO_COLETA_US = 1000000 / FREQ_COLETA_HZ; // 333µs
const int TAMANHO_BUFFER_PONTOS = FREQ_COLETA_HZ * TEMPO_ENSAIO_S; // 60.000 pontos

// ESTA LINHA AGORA VAI FUNCIONAR CORRETAMENTE
const int TAMANHO_BUFFER_BYTES = TAMANHO_BUFFER_PONTOS * sizeof(PontoDeColeta); // 360.000 bytes

// -- Configurações de Pinos --
const int SENSOR_PIN_1 = 15; // Pino para Sensor 1
const int SENSOR_PIN_2 = 16; // Pino para Sensor 2
const int LED_STATUS_PIN = 2; // LED integrado na maioria das placas

// -- Configurações de Rede --
const char* WIFI_SSID = "ESP32-DATALOGGER";
const char* WIFI_SENHA = "senha1234";
const int TCP_PORTA = 80;


// --- 4. VARIÁVEIS GLOBAIS ---

// -- Buffer de Dados --
PontoDeColeta* g_dataBuffer; // Ponteiro (memória será alocada no setup)

// -- Máquina de Estados --
enum EstadoSistema { ESTADO_OCIOSO, ESTADO_COLETANDO, ESTADO_ENVIANDO };
volatile EstadoSistema g_estadoAtual = ESTADO_OCIOSO;

// -- Rede --
WiFiServer servidorTCP(TCP_PORTA);
WiFiClient clienteTCP; 

// -- Sincronização Dual-Core --
TaskHandle_t h_taskColeta; // Handle da tarefa no Core 1
SemaphoreHandle_t sem_inicioColeta; // Semáforo para sinalizar "Comece!"
volatile bool g_coletaConcluida = false; // Flag de aviso Core 1 -> Core 0


// =====================================================================================
// --- 5. TAREFA DO NÚCLEO 1 (COLETA DE DADOS) ---
// =====================================================================================
void taskColetaCore1(void* pvParameters) {
  
  Serial.println("Core 1: Tarefa de coleta iniciada e pronta.");

  for (;;) {
    
    // 1. Espera pelo "sinal verde" (semáforo) vindo do Core 0
    xSemaphoreTake(sem_inicioColeta, portMAX_DELAY);

    // --- SINAL RECEBIDO! INICIANDO A COLETA ---
    Serial.println("Core 1: Sinal recebido! Iniciando coleta de 20s...");

    unsigned long tempoProximaColeta = micros();

    for (int i = 0; i < TAMANHO_BUFFER_PONTOS; i++) {
      
      // 2. Coleta os dados no buffer
      g_dataBuffer[i].estadoSensor1 = digitalRead(SENSOR_PIN_1);
      g_dataBuffer[i].estadoSensor2 = digitalRead(SENSOR_PIN_2);
      g_dataBuffer[i].timestamp_us = micros(); 

      // 3. Loop de espera ("busy-wait") para garantir a frequência de 3kHz
      tempoProximaColeta += INTERVALO_COLETA_US;
      while (micros() < tempoProximaColeta) {
        // Espera ativamente
      }
    }

    // --- COLETA CONCLUÍDA ---
    Serial.println("Core 1: Coleta concluída! Avisando Core 0.");

    // 4. Avisa ao Core 0 que os dados estão prontos
    g_coletaConcluida = true;
  }
}


// =====================================================================================
// --- 6. FUNÇÕES DE GERENCIAMENTO (NÚCLEO 0) ---
// =====================================================================================

void gerenciarEstadoOcioso() {
  digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN)); // Pisca LED azul (lento)
  delay(500);

  // Verifica se algum cliente novo se conectou
  if (!clienteTCP.connected()) {
    clienteTCP = servidorTCP.available();
    if (clienteTCP) {
      Serial.println("Core 0: Cliente TCP conectado!");
    }
    return; 
  }

  // Se o cliente está conectado, verifica se ele enviou dados
  if (clienteTCP.available() > 0) {
    char cmd = clienteTCP.read(); // Lê o comando

    if (cmd == 'S') {
      Serial.println("Core 0: Comando 'S' recebido. Iniciando coleta...");
      
      g_coletaConcluida = false;
      digitalWrite(LED_STATUS_PIN, HIGH); // LED Vermelho (sólido)
      xSemaphoreGive(sem_inicioColeta); // "Dá o sinal verde" para o Core 1
      g_estadoAtual = ESTADO_COLETANDO;
      
    } else {
      Serial.print("Core 0: Comando '");
      Serial.print(cmd);
      Serial.println("' desconhecido.");
    }
  }
}

void gerenciarEstadoColeta() {
  // O LED já está vermelho (HIGH)
  
  if (!clienteTCP.connected()) {
    Serial.println("Core 0: Cliente desconectou durante a coleta. Abortando.");
    // NOTA: Implementar V2 - Lógica para forçar parada do Core 1
    g_estadoAtual = ESTADO_OCIOSO;
    return;
  }
  
  // Verifica se o Core 1 já terminou seu trabalho
  if (g_coletaConcluida) {
    Serial.println("Core 0: Flag de coleta concluída recebida.");
    g_estadoAtual = ESTADO_ENVIANDO;
  }
}

void gerenciarEstadoEnvio() {
  Serial.println("Core 0: Iniciando envio de 360KB de dados...");
  
  // Pisca o LED verde (rápido) para indicar "enviando"
  digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN)); 
  delay(100);

  if (clienteTCP.connected()) {
    // Envia o buffer inteiro de dados binários brutos
    size_t bytesEnviados = clienteTCP.write((uint8_t*) g_dataBuffer, TAMANHO_BUFFER_BYTES);

    if (bytesEnviados == TAMANHO_BUFFER_BYTES) {
      Serial.println("Core 0: Envio concluído com sucesso!");
    } else {
      Serial.print("Core 0: Erro no envio! Enviados ");
      Serial.print(bytesEnviados);
      Serial.print(" de ");
      Serial.println(TAMANHO_BUFFER_BYTES);
    }
    
  } else {
    Serial.println("Core 0: Erro! Cliente desconectou antes do envio.");
  }

  // Volta ao estado ocioso
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
  Serial.println("--- Datalogger ESP32 Dual-Core V1.1 Iniciando ---");

  // --- 1. Alocação do Buffer (Crítico!) ---
  Serial.print("Core 0: Alocando 360KB de SRAM para o buffer de dados...");
  g_dataBuffer = (PontoDeColeta*) malloc(TAMANHO_BUFFER_BYTES);

  if (g_dataBuffer == NULL) {
    Serial.println("FALHA CRITICA! Nao foi possivel alocar o buffer.");
    pinMode(LED_STATUS_PIN, OUTPUT);
    while (true) { // Trava o sistema indicando erro fatal
      digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
      delay(50);
    }
  }
  Serial.println("Sucesso!");

  // --- 2. Configurar Pinos ---
  pinMode(LED_STATUS_PIN, OUTPUT);
  pinMode(SENSOR_PIN_1, INPUT_PULLUP);
  pinMode(SENSOR_PIN_2, INPUT_PULLUP);

  // --- 3. Criar Semáforo de Sincronização ---
  sem_inicioColeta = xSemaphoreCreateBinary();

  // --- 4. Iniciar a Tarefa no Core 1 ---
  xTaskCreatePinnedToCore(
      taskColetaCore1,    // Função da tarefa
      "TaskColeta",       // Nome (para debug)
      10000,              // Tamanho da Stack (pilha de memória)
      NULL,               // Parâmetros da tarefa
      1,                  // Prioridade da tarefa (1 = alta)
      &h_taskColeta,      // Handle da tarefa
      1);                 // Núcleo onde a tarefa vai rodar (0 ou 1)
  
  // --- 5. Configurar Wi-Fi e Rede (Core 0) ---
  Serial.print("Core 0: Configurando Access Point Wi-Fi...");
  WiFi.softAP(WIFI_SSID, WIFI_SENHA);
  Serial.print("OK! IP do AP: ");
  Serial.println(WiFi.softAPIP()); // Geralmente 192.168.4.1

  Serial.print("Core 0: Iniciando Servidor TCP na porta ");
  Serial.println(TCP_PORTA);
  servidorTCP.begin();

  Serial.println("\n--- Sistema pronto. Aguardando cliente no Estado Ocioso ---");
}


// =====================================================================================
// --- 8. LOOP PRINCIPAL (Executado continuamente no Núcleo 0) ---
// =====================================================================================
void loop() {
  // O loop principal no Core 0 é apenas um gerenciador de estados.
  
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
