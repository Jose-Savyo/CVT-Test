# CVT-Test

# Projeto Datalogger de Alta Frequência com ESP32

![Status: Em Desenvolvimento](https://img.shields.io/badge/status-em_desenvolvimento-yellow.svg)

Este repositório contém o firmware para um sistema de aquisição de dados (datalogger) de alta velocidade, baseado na plataforma ESP32. O objetivo é realizar ensaios de 20 segundos a uma frequência de 3 kHz, armazenando os dados em um buffer interno e transferindo-os via Wi-Fi para um PC para análise posterior.

Este projeto é projetado especificamente para operação em campo, onde a ESP32 atua como um ponto de acesso Wi-Fi (Access Point), permitindo que um notebook se conecte diretamente a ela para controle e coleta de dados sem a necessidade de uma rede externa.

## Funcionalidades Principais

* **Hardware:** ESP32 (placa de desenvolvimento convencional, sem PSRAM).
* **Aquisição de Dados:** Leitura de 2 canais (dois sensores digitais).
* **Frequência de Amostragem:** **3 kHz** (estável).
* **Duração do Ensaio:** **20 segundos**.
* **Buffer de Aquisição:** Buffer único na SRAM interna com **60.000 amostras**.
    * *Cálculo: (3.000 amostras/s) * (20 s) = 60.000 amostras*
* **Estrutura de Dados:** 6 bytes por amostra (Sensor 1, Sensor 2, Timestamp `micros()`).
    * *Uso de Memória: 60.000 amostras * 6 bytes/amostra = 360.000 bytes (360 KB)*
* **Rede:** Servidor Wi-Fi em modo **Access Point (AP)**.
* **Protocolo de Transferência:** **Servidor TCP** para transferência de dados binários brutos, garantindo máxima velocidade e integridade.
* **Controle:** Início de ensaio controlado por um comando enviado via TCP.
* **Arquitetura Robusta:** Uso da arquitetura **Dual-Core** da ESP32 para garantir coleta de dados sem *jitter* (interferência da pilha Wi-Fi).

## Arquitetura do Software

O desafio central deste projeto é gerenciar um buffer de 360 KB e, ao mesmo tempo, lidar com a pilha de rede Wi-Fi em uma placa sem PSRAM. A solução é uma arquitetura dual-core que isola as tarefas críticas.

### Núcleo 0 (Aplicação e Rede)

Este núcleo é o "gerente de comunicação". Ele é responsável por todas as tarefas não-críticas em tempo real:
* Inicializar e manter o Access Point Wi-Fi.
* Executar o Servidor TCP.
* Gerenciar conexões de clientes (o notebook).
* Controlar a máquina de estados principal (Ocioso, Coletando, Enviando).
* Controlar LEDs de status.

### Núcleo 1 (Aquisição de Dados)

Este núcleo é o "coletor de tempo real". Ele tem uma única responsabilidade:
* Permanecer em espera até ser sinalizado pelo Núcleo 0.
* Ao receber o sinal, executar um loop de alta prioridade para coletar as 60.000 amostras na frequência exata de 3 kHz (a cada 333µs).
* Avisar o Núcleo 0 quando a coleta estiver concluída.

Esta separação garante que as interrupções do Wi-Fi e do TCP/IP (executadas no Núcleo 0) **nunca** causem atrasos ou *jitter* na rotina de coleta de dados (executada no Núcleo 1).

### Estrutura de Dados (6 bytes)

Cada ponto de coleta é salvo na seguinte estrutura:

```c
struct PontoDeColeta {
  byte estadoSensor1;      // 1 byte
  byte estadoSensor2;      // 1 byte
  unsigned long timestamp_us; // 4 bytes
};
