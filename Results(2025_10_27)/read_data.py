import struct
import csv
import os

# --- Configurações ---
# O arquivo binário que o 'client.py' acabou de salvar
ARQUIVO_BINARIO = "ensaio_continuo_3.bin" 

# O nome do arquivo legível que vamos criar
ARQUIVO_CSV = "ensaio_decodificado.csv"

# O "molde" dos nossos dados, exatamente como no firmware.
# 6 bytes no total.
TAMANHO_DO_PONTO = 6 # 1 + 1 + 4

# A "fórmula de tradução" do 'struct' do Python:
# '<' = Little-endian (como a ESP32 armazena)
# 'B' = 1 byte (unsigned char), para o Sensor 1
# 'B' = 1 byte (unsigned char), para o Sensor 2
# 'L' = 4 bytes (unsigned long), para o Timestamp
FORMATO_STRUCT = "<BBL"

def decodificar_binario():
    
    print(f"--- Decodificador de Dados (BIN para CSV) ---")
    
    # Verifica se o arquivo de entrada existe
    if not os.path.exists(ARQUIVO_BINARIO):
        print(f"Erro: Arquivo '{ARQUIVO_BINARIO}' não encontrado.")
        print("Certifique-se de executar o 'client.py' primeiro.")
        return

    # 1. Abrimos o arquivo binário para leitura ('rb' = Read Binary)
    with open(ARQUIVO_BINARIO, 'rb') as f_bin:
        
        # 2. Criamos e abrimos o arquivo CSV para escrita ('w' = Write)
        with open(ARQUIVO_CSV, 'w', newline='') as f_csv:
            
            # 3. Preparamos o escritor de CSV
            csv_writer = csv.writer(f_csv)
            
            # 4. Escrevemos o cabeçalho no CSV
            csv_writer.writerow(["timestamp_us", "sensor_1", "sensor_2"])
            
            total_pontos = 0
            
            # 5. Loop para ler o arquivo "fatia" por "fatia"
            while True:
                # Lê a próxima "fatia" de 6 bytes
                chunk = f_bin.read(TAMANHO_DO_PONTO)
                
                # Se a "fatia" estiver vazia, chegamos ao fim do arquivo
                if not chunk:
                    break 
                
                # Se a "fatia" tiver o tamanho correto (6 bytes)...
                if len(chunk) == TAMANHO_DO_PONTO:
                    # 6. A "Mágica": Desempacota os bytes em números
                    # data será uma tupla, ex: (1, 0, 123456)
                    data = struct.unpack(FORMATO_STRUCT, chunk)
                    
                    # 7. Reorganiza e escreve a linha no CSV
                    # data[0] = sensor1, data[1] = sensor2, data[2] = timestamp
                    csv_writer.writerow([data[2], data[0], data[1]])
                    
                    total_pontos += 1
                else:
                    print(f"Aviso: Encontrado um pedaço de dados incompleto no fim do arquivo.")

    print(f"\n--- Decodificação Concluída! ---")
    print(f"Total de {total_pontos} pontos de dados lidos.")
    print(f"Arquivo legível salvo como: '{ARQUIVO_CSV}'")

# --- Ponto de Entrada Principal ---
if __name__ == "__main__":
    decodificar_binario()
