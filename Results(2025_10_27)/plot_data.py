import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import os

# --- Configurações ---
ARQUIVO_CSV = "ensaio_decodificado.csv"
ARQUIVO_GRAFICO_SAIDA = "leituras_sensores.png" # Nome do arquivo de imagem a ser salvo

# --- Verifica se o arquivo CSV existe ---
if not os.path.exists(ARQUIVO_CSV):
    print(f"Erro: Arquivo '{ARQUIVO_CSV}' não encontrado.")
    print("Certifique-se de que o arquivo CSV está na mesma pasta que este script.")
    exit()

print(f"Lendo dados do arquivo: {ARQUIVO_CSV}")

# --- Leitura dos Dados ---
try:
    # Lê o CSV para um DataFrame do pandas
    df = pd.read_csv(ARQUIVO_CSV)

    # --- Pré-processamento dos Dados ---
    # Converte o timestamp de microssegundos para segundos (melhor para o eixo do gráfico)
    # Pega o primeiro timestamp como referência (t=0) para evitar números muito grandes no eixo X
    tempo_inicial_us = df['timestamp_us'].iloc[0]
    df['tempo_s'] = (df['timestamp_us'] - tempo_inicial_us) / 1_000_000.0

    print("Dados lidos com sucesso. Gerando gráfico...")

    # --- Criação do Gráfico ---
    # Cria uma figura e dois eixos (subplots), um em cima do outro (2 linhas, 1 coluna)
    # sharex=True faz com que ambos os gráficos compartilhem o mesmo eixo X (tempo)
    fig, axes = plt.subplots(nrows=2, ncols=1, figsize=(12, 8), sharex=True)

    # --- Plot Sensor 1 ---
    axes[0].plot(df['tempo_s'], df['sensor_1'], marker='.', linestyle='-', markersize=2, label='Sensor 1')
    axes[0].set_title('Leitura do Sensor 1')
    axes[0].set_ylabel('Estado (0 ou 1)')
    # Ajusta os limites do eixo Y para melhor visualização de dados digitais
    axes[0].set_ylim(-0.1, 1.1) 
    axes[0].grid(True, which='both', linestyle='--', linewidth=0.5)
    axes[0].legend()
    # Formata o eixo Y para mostrar apenas 0 e 1
    axes[0].yaxis.set_major_locator(ticker.MultipleLocator(1)) 

    # --- Plot Sensor 2 ---
    axes[1].plot(df['tempo_s'], df['sensor_2'], marker='.', linestyle='-', markersize=2, color='orange', label='Sensor 2')
    axes[1].set_title('Leitura do Sensor 2')
    axes[1].set_xlabel('Tempo (segundos)')
    axes[1].set_ylabel('Estado (0 ou 1)')
    axes[1].set_ylim(-0.1, 1.1)
    axes[1].grid(True, which='both', linestyle='--', linewidth=0.5)
    axes[1].legend()
    # Formata o eixo Y para mostrar apenas 0 e 1
    axes[1].yaxis.set_major_locator(ticker.MultipleLocator(1))

    # --- Ajustes Finais e Salvamento ---
    plt.suptitle('Leituras dos Sensores ao Longo do Tempo', fontsize=16) # Título geral
    plt.tight_layout(rect=[0, 0.03, 1, 0.95]) # Ajusta espaçamento para evitar sobreposição

    # Salva o gráfico como uma imagem PNG
    plt.savefig(ARQUIVO_GRAFICO_SAIDA)
    print(f"Gráfico salvo com sucesso como: {ARQUIVO_GRAFICO_SAIDA}")

    # plt.show() # Descomente esta linha se quiser que o gráfico apareça numa janela

except Exception as e:
    print(f"Ocorreu um erro ao processar o arquivo ou gerar o gráfico: {e}")
