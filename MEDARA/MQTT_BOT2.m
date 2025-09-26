%% ==== CONFIGURACIÓN (EDITA ESTAS VARIABLES) ====
endpointATS = "ssl://aqloxhiemdroo-ats.iot.us-east-1.amazonaws.com";
portTLS     = 8883;
clientId    = "GinBot";  % Puede ser el nombre de tu Thing u otro ID único

% Rutas a TUS archivos locales (ajusta según tu PC)
% Ejemplo Windows:
rootCA   = "C:\Users\Usuario\OneDrive\Documentos\MATLAB\MEDARA\AmazonRootCA1.pem"; % o AmazonRootCA3.pem
clientCr = "C:\Users\Usuario\OneDrive\Documentos\MATLAB\MEDARA\13c49290336fd2f0702fece323b6893a8028373592bf52037ba31d4d27891749-certificate.pem.crt";
clientKey= "C:\Users\Usuario\OneDrive\Documentos\MATLAB\MEDARA\13c49290336fd2f0702fece323b6893a8028373592bf52037ba31d4d27891749-private.pem.key";

% Ejemplo Linux/macOS (comenta las 3 de arriba y descomenta estas si aplica):
% rootCA   = "/home/usuario/certs/AmazonRootCA1.pem";
% clientCr = "/home/usuario/certs/13c49290...-certificate.pem.crt";
% clientKey= "/home/usuario/certs/13c49290...-private.pem.key";

% Si tu llave privada está cifrada con passphrase, añade:
% sslPass = "tu_passphrase";  % y pasa SSLPassword=sslPass en mqttclient

%topicSub = "test";   % Topic al que te suscribes
%topicPub = "test";   % Topic al que publicas (puede ser el mismo)
%qosLevel = 1;        % 0|1|2 (AWS IoT soporta 0 y 1)

%% == TOPICO ==
topicSub = "UDG\CUValles\Robot\SCARA\GinBot" ;
topicPub = "UDG\CUValles\Robot\SCARA\GinBot" ;
qosLevel = 1 ;

%% == TOPICO SIMULACION ==

%SUB A TOPIC
topicSub = "UDG\CUValles\Robot\SCARA\GinBot\simulacion" ;
topicPub = "UDG\CUValles\Robot\SCARA\GinBot\simulacion" ;
qosLevel = 1 ;

%% ==== VALIDACIONES BÁSICAS ====
assert(isfile(rootCA),   "No se encontró el CA root en: %s", rootCA);
assert(isfile(clientCr), "No se encontró el certificado del cliente en: %s", clientCr);
assert(isfile(clientKey),"No se encontró la llave privada en: %s", clientKey);

%% ==== CONEXIÓN MQTT CON TLS (mTLS) ====
try
    % Crea el cliente MQTT con certificados
    mq = mqttclient(endpointATS, ...
        Port=portTLS, ...
        ClientID=clientId, ...
        CARootCertificate=rootCA, ...
        ClientCertificate=clientCr, ...
        ClientKey=clientKey); % , SSLPassword=sslPass

    fprintf("Conectado a %s con ClientID=%s\n", endpointATS, clientId);
    disp(mq);  % Muestra info del cliente (útil para depurar)
catch ME
    warning("Fallo conectando a MQTT/TLS: %s", ME.message);
    rethrow(ME);
end

%% ==== SUSCRIPCIÓN CON CALLBACK ====
% El callback imprime cada mensaje entrante
mySub = [];
try
    mySub = subscribe(mq, topicSub, QualityOfService=qosLevel, Callback=@msgCallback);
    fprintf("Suscrito a '%s' (QoS=%d)\n", topicSub, qosLevel);
catch ME
    warning("No se pudo suscribir a '%s': %s", topicSub, ME.message);
    cleanupClient(mq);
    rethrow(ME);
end

%% ==== PUBLICACIÓN DE PRUEBA ====
payload = struct("msg","hola desde MATLAB","ts",datestr(now,"yyyy-mm-dd HH:MM:SS.FFF"));
try
    write(mq, topicPub, jsonencode(payload), QualityOfService=qosLevel);
    fprintf("Publicado en '%s': %s\n", topicPub, jsonencode(payload));
catch ME
    warning("Error al publicar: %s", ME.message);
end

%% ==== ESPERA BREVE PARA RECIBIR MENSAJES (DEMO) ====
pause(2.0);  % dale un par de segundos a la callback

%% Robot GEMELO
%CINEMATICA DIRECTA
longitudes = [L1, L2, L3];
angulos = [theta1, theta2, theta3];
yaw   = theta1 + theta2 + theta3;  % en grados

% Posición
X  = round(x,3); 
Y  = round(y,3); 
Z  = round(z,3);
% Matriz de rotación redondeada
R_rounded = round(R,3);

% ==== CREAR STRUCT FORMATEADO ====
robotData = struct( ...
    'Robot', struct( ...
        'Tipo', 'SCARA', ...
        'Modelo', 'GinBot', ...
        'Modo_de_Operacion', 'GemeloDigital' ...
    ), ...
    'Cinematica', struct( ...
        'Tipo', 'Directa', ...
        'Metodo', 'Geometrico', ...
        'ParametrosEntrada', struct( ...
            'Longitudes', longitudes, ...
            'Angulos', angulos ... % grados
        ), ...
        'Resultados', struct( ...
            'X', X, 'Y', Y, 'Z', Z, ...
            'Orientacion', struct( ...
                'Euler', struct('Roll', Roll,'Pitch', Pitch,'Yaw', Yaw), ...
                'MatrizRotacion', R_rounded ...
            ) ...
        ) ...
    ), ...
    'Metadata', struct( ...
        'Timestamp', datestr(now,'yyyy-mm-dd HH:MM:SS.FFF'), ...
        'UnidadLongitud', 'mm', ...
        'UnidadAngulos', 'grados' ...
    ) ...
);

% JSON “bonito” con indentación
payloadPretty = jsonencode(robotData, PrettyPrint=true);

try
    write(mq, topicPub, payloadPretty, QualityOfService=qosLevel);
    fprintf("Publicado en '%s': %s\n", topicPub, payloadPretty);
catch ME
    warning("Error al publicar: %s", ME.message);
end

%% DECIDUFUCADO
decodedData = jsondecode(payload);

disp(decodedData.Robot)
disp(decodedData.Cinematica.Resultados)
disp(decodedData.Metadata)

%% Cinematia Inversa y directa

%MARCO DE REFERENCIA FIJO AL CUERPO%
p1 = [10 0 0]';
p2 = [0 10 0]';
p3 = [0 0 10]';
xlim([-10.5,10.5]); ylim([-10.5,10.5]); zlim([-10.5, 10.5]);

plot3(p1(1),p1(2),p1(3), 'x', 'color', 'red', 'LineWidth',2);hold on;
plot3(p2(1),p2(2),p2(3), 'x', 'color', 'red', 'LineWidth',2);hold on;
plot3(p3(1),p3(2),p3(3), 'x', 'color', 'red', 'LineWidth',2),hold on;

line([0,p1(1)],[0,p1(2)],[0,p1(3)],'color','red','Linewidth', 2); hold on;
line([0,p2(1)],[0,p2(2)],[0,p2(3)],'color','red','Linewidth', 2); hold on;
line([0,p3(1)],[0,p3(2)],[0,p3(3)],'color','red','Linewidth', 2); hold on;
view(2)

xlabel('X');
ylabel('Y');
title('Cinematica Inversa SCARA 2D');
grid on;

L1 = 2 ; L2 = 3; xb = 5; yb = 0;

texto_q1 = text(5, 5, 'Ángulo Eslabón 1: 0°', 'FontSize', 10, 'Color', 'black');
texto_q2 = text(5, 4, 'Ángulo Eslabón 2: 0°', 'FontSize', 10, 'Color', 'magenta');
efector_final = text(5, 3, sprintf('Posición Efector Final: (%.2f, %.2f)', xb, yb), 'FontSize', 12, 'Color', 'blue', 'FontWeight', 'bold');

for t = 0:0.05:10
    xb = xb - 0.01;
    yb = yb + 0.02;
n = 10 / 0.05;
coseq2 = (xb^2 + yb^2 - L2^2 - L1^2)/(2*L1*L2);

senoq2 = real(sqrt((1- coseq2^2)));

q2 = atan2d(senoq2,coseq2);

q1 = atan2d(yb,xb) - atan2d((L2* senoq2),(L1 + (L2 * coseq2)));


%CINEMATICA DIRECTA ESLABO 1
p1A = [L1, 0, 0]';
Rz1 = [cosd(q1) -sind(q1) 0;
        sind(q1) cosd(q1) 0;
        0 0 1] ;
    
p1B = Rz1 * p1A;
A1 = plot3(p1B(1),p1B(2),p1B(3), 'o', 'color', 'black', 'LineWidth',2);
A2 =line([0,p1B(1)],[0,p1B(2)],[0,p1B(3)],'color','black','Linewidth', 2);

%CINEMATICA DIRECTA ESLABO 1
p1C = [L2, 0, 0]';
Rz2 = [cosd(q1+q2) -sind(q1+q2) 0;
        sind(q1+q2) cosd(q1+q2) 0;
        0 0 1];
    
p1D = p1B + Rz2*p1C;

B1 = plot3(p1D(1),p1D(2),p1D(3), 'o', 'color', 'magenta', 'LineWidth',2);
B2 = line([p1B(1),p1D(1)],[p1B(2),p1D(2)],[p1B(3),p1D(3)],'color','magenta','Linewidth', 3);

C  = plot3(xb,yb,0, 'o', 'color', 'blue', 'LineWidth',2);

 set(texto_q1, 'String', sprintf('Ángulo Eslabón 1: %.2f°', q1));
 set(texto_q2, 'String', sprintf('Ángulo Eslabón 2: %.2f°', q2));
 set(efector_final, 'String', sprintf('Posición Efector Final: (%.2f, %.2f)', xb, yb));


legend([A2, B2, C], {'Eslabon 1', 'Eslabon 2', 'Efector final'});

longitudes = [L1, L2];
    angulos = [q1, q2];
    yaw   = q1 + q2;  % en grados
    pEnd = p1D;
    
    % Posición
    X  = round(xb,3); 
    Y  = round(yb,3); 
    Z  = 0.00;
    
    % ==== CREAR STRUCT FORMATEADO ====
    robotData = struct( ...
        'Robot', struct( ...
            'Tipo', 'SCARA', ...
            'Modelo', 'GinBot', ...
            'Modo_de_Operacion', 'GemeloDigital' ...
        ), ...
        'CinematicaDirecta', struct( ...
            'ParametrosEntrada', struct('Longitudes',longitudes,'Angulos',angulos), ...
            'Eslabones', struct('p1B',round(p1B',3),'p1D',round(p1D',3),'pEnd',round(pEnd',3)), ...
            'Resultados', struct('X',round(pEnd(1),3),'Y',round(pEnd(2),3),'Z',round(pEnd(3),3), ...
                                 'Orientacion', struct('Euler',struct('Roll',0,'Pitch',0,'Yaw',round(yaw,2)))) ...
        ), ...
        'CinematicaInversa', struct( ...
            'AngulosCalculados', struct('q1', round(q1,3),'q2', round(q2,3)), ...
            'PosicionDeseada', struct('X',round(xb,3),'Y',round(yb,3),'Z',0)...
            ), ...
        'Metadata', struct( ...
            'Timestamp', datestr(now,'yyyy-mm-dd HH:MM:SS.FFF'), ...
            'UnidadLongitud', 'mm', ...
            'UnidadAngulos', 'grados' ...
        ) ...
    );
    
   %JSON “bonito” con indentación
   payloadPretty = jsonencode(robotData, PrettyPrint=true);
   if t == 0 || t == floor(10/2) || t == 10 
        try
            write(mq, topicPub, payloadPretty, QualityOfService=qosLevel);
            fprintf("Publicado en '%s': %s\n", topicPub, payloadPretty);
        catch ME
            warning("Error al publicar: %s", ME.message);
        end
        disp('Mensaje enviado');
        disp(t);
   end
    pause(0.001);
    delete(A1); delete(A2); delete(B1); delete(B2); delete(C);

end

A1 = plot3(p1B(1),p1B(2),p1B(3), 'o', 'color', 'black', 'LineWidth',2); hold on;
A2 =line([0,p1B(1)],[0,p1B(2)],[0,p1B(3)],'color','black','Linewidth', 2); hold on;

B1 = plot3(p1D(1),p1D(2),p1D(3), 'o', 'color', 'magenta', 'LineWidth',2); hold on;
B2 = line([p1B(1),p1D(1)],[p1B(2),p1D(2)],[p1B(3),p1D(3)],'color','magenta','Linewidth', 3); hold on;
C  = plot3(xb,yb,0, 'o', 'color', 'blue', 'LineWidth',2);

legend([A2, B2, C], {'Eslabon 1', 'Eslabon 2', 'Posicion Efector final'});


    disp('===== JSON PUBLICADO =====');
    disp(payloadPretty)
%% ==== LIMPIEZA (UNSUBSCRIBE Y CIERRE) ====
try
    if ~isempty(mySub)
        unsubscribe(mq, topicSub);
        fprintf("Desuscrito de '%s'\n", topicSub);
    end
catch ME
    warning("Error al desuscribir: %s", ME.message);
end

%% Mensaje tiempo real
L1 = 2; 
L2 = 3;

% Visualización inicial
figure('Name','SCARA Simulación');
ax = axes;
xlim([-6,6]); ylim([-6,6]); zlim([-1,6]); view(2); grid on; hold on;
xlabel('X'); ylabel('Y'); title('SCARA 2D');

% Ejes fijos
plot3([0,L1],[0,0],[0,0],'color','r','LineWidth',2); hold on;
plot3([0,0],[0,L2],[0,0],'color','r','LineWidth',2);

% Textos
texto_q1 = text(-5,5,'q1: 0°','FontSize',10);
texto_q2 = text(-5,4,'q2: 0°','FontSize',10);
efector_final = text(-5,3,'Posición: (0,0)','FontSize',10,'FontWeight','bold');

% ===== BUCLE PRINCIPAL =====
topicSub = "UDG/CUValles/Robot/SCARA/GinBot/simulacion"; 
disp("Iniciando simulación...");
while true
    try
        msg = read(mq,topicSub); % Leer mensajes del tópico
        
        if istimetable(msg) && height(msg) > 0
            raw = msg{end,1}; % Último mensaje
            if isa(raw,'uint8') || isa(raw,'string')
                raw = char(raw'); 
                data = jsondecode(raw);

                % Extraer ángulos
                angulos = data.CinematicaDirecta.ParametrosEntrada.Angulos;
                q1 = angulos(1);
                q2 = angulos(2);

                % Extraer longitudes (opcional)
                L1 = data.CinematicaDirecta.ParametrosEntrada.Longitudes(1);
                L2 = data.CinematicaDirecta.ParametrosEntrada.Longitudes(2);

                % Cinemática directa
                p1B = [L1*cosd(q1); L1*sind(q1); 0];
                p1D = p1B + [L2*cosd(q1+q2); L2*sind(q1+q2); 0];

                % Mover servos simultáneamente (normalizando 0-1)
               % writePosition(servo1, q1/180);
                %writePosition(servo2, q2/180);

                % Limpiar figuras anteriores
                cla(ax);

                % Dibujar eslabón 1
                line([0,p1B(1)],[0,p1B(2)],[0,0],'color','b','LineWidth',3);
                plot3(p1B(1),p1B(2),0,'bo','MarkerSize',6,'MarkerFaceColor','b');

                % Dibujar eslabón 2
                line([p1B(1),p1D(1)],[p1B(2),p1D(2)],[0,0],'color','m','LineWidth',3);
                plot3(p1D(1),p1D(2),0,'mo','MarkerSize',6,'MarkerFaceColor','m');

                % Actualizar textos
                set(texto_q1,'String',sprintf('q1: %.2f°', q1));
                set(texto_q2,'String',sprintf('q2: %.2f°', q2));
                set(efector_final,'String',sprintf('Posición: (%.2f, %.2f)', p1D(1), p1D(2)));

                drawnow; % Actualizar figura
            end
        else
            disp("Esperando mensajes...");
            pause(0.1);
        end
    catch ME
        warning("Error procesando mensaje: %s", ME.message);
        pause(0.1);
    end
end
%%
clear mq;  % cierra la conexión MQTT
disp("Conexión cerrada.");

%% ==== FUNCIONES LOCALES ====

function msgCallback(topic, data)
% Imprime cada mensaje que llega a través de la suscripción.
    try
        strData = string(data);
        % Si es JSON, intenta decodificar para mostrar bonito
        try
            obj = jsondecode(strData);
            fprintf("[RX] Topic=%s | JSON=%s\n", topic, jsonencode(obj));
        catch
            fprintf("[RX] Topic=%s | Data=%s\n", topic, strData);
        end
    catch ME
        fprintf("[CallbackError] %s\n", ME.message);
    end
end

function cleanupClient(mq)
% Helper por si quieres asegurar cierre en errores
    try
        clear mq
    catch
    end
end
%%

% Configuración inicial del gráfico
figure('Name','Simulación SCARA con JSON');
hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Cinemática Inversa SCARA 2D');
view(2);
xlim([-10.5,10.5]); ylim([-10.5,10.5]); zlim([-1, 10]);

% Ejes fijos del robot (marco de referencia)
p1 = [10 0 0]'; p2 = [0 10 0]'; p3 = [0 0 10]';
plot3(p1(1), p1(2), p1(3), 'x', 'Color', 'r', 'LineWidth', 2); hold on;
plot3(p2(1), p2(2), p2(3), 'x', 'Color', 'r', 'LineWidth', 2); hold on;
plot3(p3(1), p3(2), p3(3), 'x', 'Color', 'r', 'LineWidth', 2); hold on;

line([0,p1(1)],[0,p1(2)],[0,p1(3)],'color','r','LineWidth',2); hold on;
line([0,p2(1)],[0,p2(2)],[0,p2(3)],'color','r','LineWidth',2); hold on;
line([0,p3(1)],[0,p3(2)],[0,p3(3)],'color','r','LineWidth',2); hold on;

% Configuración robot SCARA
L1 = 2; L2 = 3;  % Longitudes de los eslabones

% Textos dinámicos
texto_q1 = text(5,5,'Ángulo Eslabón 1: 0°','FontSize',10,'Color','black');
texto_q2 = text(5,4,'Ángulo Eslabón 2: 0°','FontSize',10,'Color','magenta');
efector_final = text(5,3,'Posición Efector Final: (0,0)','FontSize',12,'Color','blue','FontWeight','bold');

% Archivo JSON simulado
%archivoMensaje = 'C:\Users\Usuario\OneDrive\Documentos\MATLAB\MEDARA\Posicion.json';  % Ruta al archivo JSON

% Bucle principal de simulación
for k = 1:20
    try
        % Leer último mensaje recibido en el topic
        msg = read(mq, topicSub);
        if istimetable(msg) && height(msg) > 0
            raw = msg{end,1};  % Último mensaje
            if isa(raw,'uint8') || isa(raw,'string')
                raw = char(raw');
                data = jsondecode(raw);  % Decodificar JSON
                xb = data.CinematicaInversa.PosicionDeseada.X;
                yb = data.CinematicaInversa.PosicionDeseada.Y;
            else
                xb = 5; yb = 0; % Valores por defecto si no es uint8/string
            end
        else
            xb = 5; yb = 0; % Valores por defecto si no hay mensaje
        end
    catch ME
        warning("Error procesando mensaje: %s", ME.message);
        xb = 5; yb = 0;
    end
    
    % CINEMÁTICA INVERSA
    coseq2 = (xb^2 + yb^2 - L2^2 - L1^2)/(2*L1*L2);
    senoq2 = real(sqrt(1 - coseq2^2));
    q2 = atan2d(senoq2,coseq2);
    q1 = atan2d(yb,xb) - atan2d(L2*senoq2, L1 + L2*coseq2);

    % CINEMÁTICA DIRECTA
    p1A = [L1;0;0];
    Rz1 = [cosd(q1) -sind(q1) 0;
           sind(q1)  cosd(q1) 0;
           0          0       1];
    p1B = Rz1*p1A;  % Fin del primer eslabón

    p1C = [L2;0;0];
    Rz2 = [cosd(q1+q2) -sind(q1+q2) 0;
           sind(q1+q2)  cosd(q1+q2) 0;
           0            0          1];
    p1D = p1B + Rz2*p1C; % Fin del segundo eslabón (efector final)

    % Dibujar robot
    A1 = plot3(p1B(1),p1B(2),p1B(3),'o','color','k','LineWidth',2);
    A2 = line([0,p1B(1)],[0,p1B(2)],[0,p1B(3)],'color','k','LineWidth',2);

    B1 = plot3(p1D(1),p1D(2),p1D(3),'o','color','m','LineWidth',2);
    B2 = line([p1B(1),p1D(1)],[p1B(2),p1D(2)],[p1B(3),p1D(3)],'color','m','LineWidth',3);

    C = plot3(xb,yb,0,'o','color','b','LineWidth',2);

    % Actualizar textos
    set(texto_q1,'String',sprintf('Ángulo Eslabón 1: %.2f°', q1));
    set(texto_q2,'String',sprintf('Ángulo Eslabón 2: %.2f°', q2));
    set(efector_final,'String',sprintf('Posición Efector Final: (%.2f, %.2f)', xb, yb));

    legend([A2, B2, C],{'Eslabon 1','Eslabon 2','Efector final'});

    pause(10);  % Pausa para animación
    delete(A1); delete(A2); delete(B1); delete(B2); delete(C);
end