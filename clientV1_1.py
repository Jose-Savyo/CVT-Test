import socket
import sys

# --- Configurações do Datalogger ---
ESP32_HOST = "192.168.4.1" 
ESP32_PORT = 80             
COMANDO_INICIO = b'S'       
WIFI_SSID = "ESP32-DATALOGGER" 
WIFI_SENHA = "senha1234"

# --- Configurações do Ensaio (V1.12) ---
#
# *** CORREÇÃO AQUI ***
# Ajustado para corresponder ao firmware V1.12
#
FREQ_COLETA_HZ = 1000  # <<< MUDADO DE 2000
TEMPO_ENSAIO_S = 6
PONTOS_POR_AMOSTRA = 6000  # <<< MUDADO DE 12000 (1000 Hz * 6 s)
BYTES_POR_PONTO = 6      
TAMANHO_ESPERADO_BYTES = PONTOS_POR_AMOSTRA * BYTES_POR_PONTO # <<< AGORA 36000

ARQUIVO_SAIDA = "ensaio.bin" 

def iniciar_ensaio():
    """
    Conecta ao datalogger ESP32, envia o comando de início,
    recebe os dados e os salva em um arquivo binário.
    """
    
    print(f"--- Cliente Datalogger Python (para Firmware V1.12) ---")
    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        try:
            sock.settimeout(15.0) 
            
            print(f"Conectando a {ESP32_HOST}:{ESP32_PORT}...")
            sock.connect((ESP32_HOST, ESP32_PORT))
            print("Conectado! Enviando comando de início...")

            sock.sendall(COMANDO_INICIO)

            print(f"Comando '{COMANDO_INICIO.decode()}' enviado.")
            print(f"Aguardando a ESP32 coletar dados por {TEMPO_ENSAIO_S} segundos...")
            
            # Esta mensagem agora mostrará o valor correto (36.0 KB)
            print(f"Esperando receber {TAMANHO_ESPERADO_BYTES / 1024.0:.1f} KB de dados...")

            data_recebida = b''
            bytes_faltando = TAMANHO_ESPERADO_BYTES
            
            while bytes_faltando > 0:
                chunk = sock.recv(min(bytes_faltando, 4096))
                
                if not chunk:
                    raise ConnectionError("Conexão interrompida. Dados incompletos.")
                
                data_recebida += chunk
                bytes_faltando -= len(chunk)
                
                print(f"Recebido {len(data_recebida)} de {TAMANHO_ESPERADO_BYTES} bytes...", end='\r')

            print("\n--- Transferência Concluída! ---")

            if len(data_recebida) == TAMANHO_ESPERADO_BYTES:
                with open(ARQUIVO_SAIDA, 'wb') as f: 
                    f.write(data_recebida)
                print(f"Sucesso! Dados salvos em '{ARQUIVO_SAIDA}'")
                print(f"Tamanho do arquivo: {len(data_recebida)} bytes.")
            else:
                print(f"Erro: Recebido {len(data_recebida)} bytes, mas esperava {TAMANHO_ESPERADO_BYTES}.")

        except socket.timeout:
            print(f"\nErro: Timeout! A ESP32 em {ESP32_HOST} não respondeu.")
            print("Verifique se você está conectado ao Wi-Fi 'ESP32-DATALOGGER'.")
        except ConnectionRefusedError:
            print(f"\nErro: Conexão recusada.")
            print("Verifique se a ESP32 está ligada e se o IP {ESP32_HOST} está correto.")
        except Exception as e:
            print(f"\nOcorreu um erro inesperado: {e}")

# --- Ponto de Entrada Principal ---
if __name__ == "__main__":
    print("===================================================================")
    print("ATENÇÃO: Por favor, conecte-se manually à rede Wi-Fi")
    print(f"SSID: '{WIFI_SSID}' (Senha: '{WIFI_SENHA}')")
    print("===================================================================")
    input("Pressione ENTER quando estiver conectado...")
    
    iniciar_ensaio()
