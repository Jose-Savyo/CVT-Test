import socket
import sys
import time

# --- Configurações do Datalogger ---
ESP32_HOST = "192.168.4.1" 
ESP32_PORT = 80             # <<<--- Definido corretamente aqui
WIFI_SSID = "ESP32-DATALOGGER" 
WIFI_SENHA = "senha1234"

# --- Configurações do Buffer (V2.0 - Duplo Buffer) ---
TAMANHO_CHUNK_BYTES = 15360 # 15KB (2560 pontos * 6 bytes)

ARQUIVO_SAIDA = "ensaio_continuo.bin" # Nome do arquivo que será salvo

def receber_stream():
    """
    Conecta ao datalogger ESP32 e recebe o stream contínuo de dados,
    salvando-os em um arquivo binário até ser interrompido (Ctrl+C).
    """
    
    print(f"--- Cliente Datalogger Python (Receptor de Stream V2.1) ---") # Versão atualizada
    
    bytes_totais_recebidos = 0
    sock = None # Inicializa para o bloco finally
    
    try:
        # 1. Criar o socket e conectar
        print(f"Conectando a {ESP32_HOST}:{ESP32_PORT}...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(10.0) 
        
        # *** CORREÇÃO AQUI *** # Usar a variável correta ESP32_PORT em vez de ESP_PORT
        sock.connect((ESP32_HOST, ESP32_PORT)) 
        # *** FIM DA CORREÇÃO ***
        
        sock.settimeout(None) 
        print("Conectado! Aguardando stream de dados...")

        # 2. Abrir o arquivo binário para escrita
        with open(ARQUIVO_SAIDA, 'wb') as f_bin: 
            
            # 3. Loop infinito de recebimento
            while True:
                data_chunk = b''
                bytes_faltando_neste_chunk = TAMANHO_CHUNK_BYTES
                
                # 4. Loop para garantir recebimento completo de UM chunk de 15KB
                while bytes_faltando_neste_chunk > 0:
                    try:
                        packet = sock.recv(min(bytes_faltando_neste_chunk, 4096))
                        
                        if not packet:
                            print("\nConexão fechada pelo servidor.")
                            raise ConnectionAbortedError("Stream terminado pela ESP32.")
                        
                        data_chunk += packet
                        bytes_faltando_neste_chunk -= len(packet)
                        
                    except socket.timeout:
                        print("\nTimeout esperando dados do chunk. A ESP32 parou de enviar?")
                        raise 
                    except Exception as e:
                         print(f"\nErro durante recebimento do chunk: {e}")
                         raise

                # 5. Se recebemos o chunk completo, salva no arquivo
                if len(data_chunk) == TAMANHO_CHUNK_BYTES:
                    f_bin.write(data_chunk)
                    bytes_totais_recebidos += len(data_chunk)
                    print(f"Chunk de {len(data_chunk)/1024.0:.1f} KB recebido. Total: {bytes_totais_recebidos / 1024.0:.1f} KB", end='\r')
                else:
                    print(f"\nErro: Recebido chunk de tamanho inesperado: {len(data_chunk)} bytes.")
                    
    except KeyboardInterrupt:
        print("\nInterrupção pelo usuário (Ctrl+C). Fechando conexão.")
    except ConnectionAbortedError as e:
        print(f"\n{e}")
    except socket.timeout:
         print(f"\nErro: Timeout na conexão inicial. Verifique IP e rede.")
    except ConnectionRefusedError:
         print(f"\nErro: Conexão recusada. Verifique se a ESP32 está ligada e rodando o firmware.")
    except Exception as e:
        print(f"\nOcorreu um erro inesperado: {e}")
        
    finally:
        # Garante que o socket seja fechado
        if sock:
             sock.close()
        print(f"\n--- Fim da Recepção ---")
        print(f"Total de dados salvos em '{ARQUIVO_SAIDA}': {bytes_totais_recebidos} bytes.")

# --- Ponto de Entrada Principal ---
if __name__ == "__main__":
    print("===================================================================")
    print("ATENÇÃO: Por favor, conecte-se manualmente à rede Wi-Fi")
    print(f"SSID: '{WIFI_SSID}' (Senha: '{WIFI_SENHA}')")
    print("===================================================================")
    input("Pressione ENTER quando estiver conectado...")
    
    receber_stream()
