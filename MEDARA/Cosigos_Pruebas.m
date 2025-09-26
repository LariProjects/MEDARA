% Número de iteraciones
n = 315;

for i = 1:n
    if i == 1
        disp('Inicio del ciclo: ');
        disp(i);
    elseif i == floor(n/2)
        disp('Mitad del ciclo: ');
        disp(i);
    elseif i == n
        disp('Fin del ciclo: ');
        disp(i);
    end
    
    pause(1)

    % Código que se ejecuta en cada iteración
    %fprintf('Iteración %d\n', i);
end


%% DECODIFICADOR MENSAJE JSON

clc; clear; close all;

% Simulación de un mensaje JSON recibido
jsonMessage = '{"ParametrosEntrada":{"Longitudes":[1,0.8,0.6],"Angulos":[30,45,-20]}}';

% Decodificar JSON
data = jsondecode(jsonMessage);

% Extraer datos
longitudes = data.ParametrosEntrada.Longitudes; % [L1, L2, L3]
angulos = data.ParametrosEntrada.Angulos;       % [theta1, theta2, theta3]

% Mostrar datos recibidos
disp("Datos recibidos desde JSON:");
disp(data);

% Ejemplo de operación: posición X, Y (cinemática directa simple SCARA)
L1 = longitudes(1);
L2 = longitudes(2);
L3 = longitudes(3);

theta1 = deg2rad(angulos(1));
theta2 = deg2rad(angulos(2));
theta3 = deg2rad(angulos(3));

% Cálculo de posición en 2D + altura
x = L1*cos(theta1) + L2*cos(theta1+theta2);
y = L1*sin(theta1) + L2*sin(theta1+theta2);
z = L3 + theta3; % ejemplo, altura más un ángulo como offset

% Crear resultado en JSON
resultados = struct( ...
    "Resultados", struct( ...
        "X", x, ...
        "Y", y, ...
        "Z", z ...
    ) ...
);

% Convertir resultados a JSON para enviar de vuelta
jsonResult = jsonencode(resultados);

disp("Resultados procesados en JSON:");
disp(jsonResult);

%% Mesanje Json fuera

clc; clear; close all;

% Leer el mensaje desde un archivo externo
fid = fopen("C:\Users\Usuario\OneDrive\Documentos\MATLAB\MEDARA\mensaje.json",'r');
raw = fread(fid,inf);
fclose(fid);
jsonMessage = char(raw)';

% Decodificar JSON
data = jsondecode(jsonMessage);

% Extraer datos
longitudes = data.ParametrosEntrada.Longitudes;
angulos = data.ParametrosEntrada.Angulos;

disp("Datos recibidos desde JSON externo:");
disp(data);

% Operación (ejemplo cinemática directa SCARA)
L1 = longitudes(1); L2 = longitudes(2); L3 = longitudes(3);
theta1 = deg2rad(angulos(1));
theta2 = deg2rad(angulos(2));
theta3 = deg2rad(angulos(3));

x = L1*cos(theta1) + L2*cos(theta1+theta2);
y = L1*sin(theta1) + L2*sin(theta1+theta2);
z = L3 + theta3;

% Resultados
resultados = struct("Resultados",struct("X",x,"Y",y,"Z",z));
jsonResult = jsonencode(resultados);

disp("Resultados procesados:");
disp(jsonResult);

%% Mesnaje en tiempo real Json
clc; clear; close all;

% Archivo JSON que actuará como "mensaje entrante"
archivoMensaje = 'C:\Users\Usuario\OneDrive\Documentos\MATLAB\MEDARA\mensaje.json';

% Bucle de simulación (ej. 10 iteraciones)
for k = 1:10
    % === Leer el archivo como si viniera de AWS ===
    if isfile(archivoMensaje)
        try
            fid = fopen(archivoMensaje,'r');   % Abrir archivo
            raw = fread(fid,inf);             % Leer contenido
            fclose(fid);                      % Cerrar
            str = char(raw)';                 % Convertir a string
            
            % Decodificar JSON
            data = jsondecode(str);
            
            % Mostrar mensaje recibido
            fprintf("\nIteración %d - Mensaje recibido:\n", k);
            disp(data);

            % Extraer datos
            longitudes = data.ParametrosEntrada.Longitudes;
            angulos = data.ParametrosEntrada.Angulos;

            disp("Datos recibidos desde JSON externo:");
            disp(data);

            % Operación (ejemplo cinemática directa SCARA)
            L1 = longitudes(1); L2 = longitudes(2); L3 = longitudes(3);
            theta1 = deg2rad(angulos(1));
            theta2 = deg2rad(angulos(2));
            theta3 = deg2rad(angulos(3));

            x = L1*cos(theta1) + L2*cos(theta1+theta2);
            y = L1*sin(theta1) + L2*sin(theta1+theta2);
            z = L3 + theta3;

            % Resultados
            resultados = struct("Resultados",struct("X",x,"Y",y,"Z",z));
            jsonResult = jsonencode(resultados);

            disp("Resultados procesados:");
            disp(jsonResult);
        catch ME
            warning("Error al leer mensaje: %s", ME.message);
        end
    else
        warning("El archivo %s no existe.", archivoMensaje);
    end
    
    % Pausa para simular espera de nuevos mensajes
    pause(10);
end


%% Mensaje tiempo Real Json Posicion
clc; clear all; close all;

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
archivoMensaje = 'C:\Users\Usuario\OneDrive\Documentos\MATLAB\MEDARA\Posicion.json';  % Ruta al archivo JSON

% Bucle principal de simulación
for k = 1:20
    % Leer mensaje JSON externo
    if isfile(archivoMensaje)
        try
            str = fileread(archivoMensaje);    % Leer archivo completo
            data = jsondecode(str);             % Decodificar JSON

            % Obtener posición deseada desde el JSON
            xb = data.PosicionDeseada.X;
            yb = data.PosicionDeseada.Y;
        catch ME
            warning("Error al leer JSON: %s", ME.message);
            % Valores por defecto si falla
            xb = 5; yb = 0;
        end
    else
        warning("Archivo JSON no encontrado. Usando valores por defecto.");
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

