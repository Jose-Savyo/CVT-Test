% Script para ler dados CSV do datalogger ESP32, calcular RPM em intervalos
% de tempo definidos e plotar os resultados.

clear;
clc;
close all;

% --- Configurações ---
nomeArquivoCSV = 'ensaio_decodificado_2.csv';
intervalo_s = 0.5; % Intervalo de tempo para cálculo do RPM (em segundos)

% --- Verifica se o arquivo existe ---
if ~isfile(nomeArquivoCSV)
    error('Arquivo "%s" não encontrado. Verifique o nome e o caminho.', nomeArquivoCSV);
end

fprintf('Lendo dados de "%s"...\n', nomeArquivoCSV);

% --- Leitura dos Dados ---
try
    opts = detectImportOptions(nomeArquivoCSV);
    % Garante que as colunas sejam lidas como números
    opts = setvartype(opts, {'timestamp_us', 'sensor_1', 'sensor_2'}, 'double'); 
    dataTable = readtable(nomeArquivoCSV, opts);
catch ME
    error('Falha ao ler o arquivo CSV. Verifique o formato.\nErro: %s', ME.message);
end

% Verifica se a tabela foi carregada corretamente e tem as colunas esperadas
if isempty(dataTable) || ~all(ismember({'timestamp_us', 'sensor_1', 'sensor_2'}, dataTable.Properties.VariableNames))
    error('Arquivo CSV vazio ou colunas esperadas ("timestamp_us", "sensor_1", "sensor_2") não encontradas.');
end

fprintf('Dados lidos. Processando %d linhas...\n', height(dataTable));

% --- Pré-processamento ---
% Converte timestamp para segundos relativos ao início
tempo_inicial_us = dataTable.timestamp_us(1);
dataTable.tempo_s = (dataTable.timestamp_us - tempo_inicial_us) / 1e6;

tempo_total_s = dataTable.tempo_s(end);
fprintf('Duração total do ensaio: %.2f segundos.\n', tempo_total_s);

% --- Detecção de Pulsos (Transições) ---
% Encontra os ÍNDICES onde ocorrem mudanças de estado (0->1 ou 1->0)
% diff calcula a diferença entre elementos consecutivos. !=0 significa mudança.
% Adiciona 1 ao índice pois diff compara i com i-1, o resultado corresponde ao índice i.
pulse_indices_s1 = find(diff(dataTable.sensor_1) ~= 0) + 1;
pulse_indices_s2 = find(diff(dataTable.sensor_2) ~= 0) + 1;

% Obtém os TEMPOS em que ocorreram os pulsos
pulse_times_s1 = dataTable.tempo_s(pulse_indices_s1);
pulse_times_s2 = dataTable.tempo_s(pulse_indices_s2);

fprintf('Pulsos detectados - Sensor 1: %d, Sensor 2: %d\n', numel(pulse_times_s1), numel(pulse_times_s2));

% --- Agrupamento e Cálculo de RPM por Intervalo ---
% Cria as "bordas" das janelas de tempo (0s, 1s, 2s, ...)
time_bins = 0:intervalo_s:ceil(tempo_total_s); % Usa ceil para garantir que o último intervalo inclua o final

% Conta quantos pulsos caem em cada janela de tempo usando histcounts
if ~isempty(pulse_times_s1)
    pulse_counts_s1 = histcounts(pulse_times_s1, time_bins);
else
    pulse_counts_s1 = zeros(1, length(time_bins)-1); % Zero pulsos se não houver transições
end

if ~isempty(pulse_times_s2)
    pulse_counts_s2 = histcounts(pulse_times_s2, time_bins);
else
    pulse_counts_s2 = zeros(1, length(time_bins)-1);
end

% Calcula o RPM para cada intervalo (Pulsos / Intervalo_s * 60)
% Como Intervalo_s = 1.0, RPM = pulse_counts * 60
rpm_s1 = pulse_counts_s1 * 60; 
rpm_s2 = pulse_counts_s2 * 60;

% Cria o vetor de tempo correspondente ao *meio* de cada intervalo para plotagem
time_points_mid = time_bins(1:end-1) + intervalo_s / 2;

% Garante que os vetores de tempo e RPM tenham o mesmo tamanho
% (histcounts retorna N valores para N+1 bordas)
if length(time_points_mid) ~= length(rpm_s1)
   warning('Discrepância de tamanho entre tempo e RPM. Verificando bins...');
   % Ajuste se necessário (pouco provável com histcounts)
   minLength = min(length(time_points_mid), length(rpm_s1));
   time_points_mid = time_points_mid(1:minLength);
   rpm_s1 = rpm_s1(1:minLength);
   rpm_s2 = rpm_s2(1:minLength);
end


fprintf('Cálculo de RPM concluído. Gerando gráfico...\n');

% --- Plotagem ---
fig = figure('Name', 'RPM dos Motores', 'NumberTitle', 'off');
fig.Position(3:4) = [900, 600]; % Ajusta tamanho da janela [width, height]

% Subplot para Sensor 1
ax1 = subplot(2, 1, 1);
stairs(time_points_mid, rpm_s1./2, 'b-', 'LineWidth', 1.5);
title('RPM - Entrada (Motor)');
ylabel('RPM');
grid on;
legend('Entrada', 'Location', 'best');
% Define limite inferior do eixo Y como 0
ylim_s1 = get(ax1, 'YLim');
max_rpm_s1 = max([rpm_s1, 10]); % Evita limite muito pequeno se RPM for sempre 0
ylim(ax1, [0, max_rpm_s1 * 1.1/2]);


% Subplot para Sensor 2
ax2 = subplot(2, 1, 2);
stairs(time_points_mid, rpm_s2./3, 'r-', 'LineWidth', 1.5);
title('RPM - Saída (Roda)');
xlabel('Tempo (segundos)');
ylabel('RPM');
grid on;
legend('Saída', 'Location', 'best');
ylim_s2 = get(ax2, 'YLim');
max_rpm_s2 = max([rpm_s2, 10]);
ylim(ax2, [0, max_rpm_s2 * 1.1/3]);

% Vincula os eixos X para zoom e pan simultâneos
linkaxes([ax1, ax2], 'x');

% Ajusta limites do eixo X para cobrir todo o período
xlim([0, time_bins(end)]);
% xlim([22, 40]);

sgtitle(sprintf('RPM Calculado a Cada %.1f Segundo(s)', intervalo_s), 'FontSize', 14); % Título geral

fprintf('Gráfico gerado com sucesso!\n');

% --- Opcional: Salvar o gráfico ---
% saveas(fig, 'rpm_motores_matlab.png');
% fprintf('Gráfico salvo como "rpm_motores_matlab.png"\n');
