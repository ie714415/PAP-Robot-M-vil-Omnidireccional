%Inicialización
clear all
samples = 5000;
formatSpec = '%d %d';
sizeA = [2 samples];
speeds     = 0;
RefSpeed   = zeros(1,samples); 
RealSpeed  = zeros(1,samples);
%% Gráfica
figure;
xlabel('Muestra');
ylabel('Velocidad Angular (rad/s)');
title('Velocidad Angular del Motor');
hold on;
%% Se abre el archivo de texto para leer los datos del la velocidad de
%referencia y la real 
fileID = fopen('datosPID.txt','r');
%% Obtención de datos 
speeds = fscanf(fileID,formatSpec,sizeA)';
fclose(fileID);

for index = 1:1:samples
    RefSpeed(index) = speeds(index,1);
    RealSpeed(index) = speeds(index,2);
end

vectorSamples = 1:1:samples;
plot(vectorSamples,RefSpeed,'g',vectorSamples,RealSpeed,'b');
legend('Referencia','Real');