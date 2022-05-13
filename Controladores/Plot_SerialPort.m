%Inicialización
samples = 100;
formatSpect = '%d %d';
speeds     = 0;
RefSpeed   = zeros(1,samples+1); 
RealSpeed  = zeros(1,samples+1);
%% Crear objeto y abrir puerto ligado al objeto
%Borrar previos
delete(instrfind({'port'},{'COM3'}));
%Objeto serial
serialObj = serial('COM3','Baudrate',115200,'Terminator','CR/LF');
fopen(serialObj);
%% Gráfica
figure;
xlabel('Muestra');
ylabel('Velocidad Angular (rad/s)');
title('Velocidad Angular del Motor');
hold on;
%% Obtención de datos 
speeds = fscanf(serialObj,formatSpect)';
counter = 2;
for index = 1:2:samples
    RefSpeed(counter) = speeds(index);
    RealSpeed(counter) = speeds(index + 1);
    counter = counter + 1;
end

speeds = fscanf(serialObj,formatSpect)';
for index = 1:2:samples
    RefSpeed(counter) = speeds(index);
    RealSpeed(counter) = speeds(index + 1);
    counter = counter + 1;
end
vectorSamples = 1:1:samples+1;
plot(vectorSamples,RefSpeed,'g',vectorSamples,RealSpeed,'b');
legend('Referencia','Real');
%% Cerrar y borrar puerto
fclose(serialObj);
delete(serialObj);