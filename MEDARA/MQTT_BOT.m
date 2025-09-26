clc; clear all; close all;

% Broker MQTT con credenciales
brokerAddress = "tcp://broker.hivemq.com";  % Cambia si usas otro broker
clientID = "myClient";
port = 1883;
username = "miUsuario";      % tu usuario
password = "miClave123";     % tu contraseña

% Crear cliente MQTT con autenticación
mqClient = mqttclient(brokerAddress, ...
    ClientID=clientID, ...
    Port=port, ...
    Username=username, ...
    Password=password);

% Verificar conexión
if mqClient.Connected
    disp("MQTT client connected successfully.");
else
    error("Failed to connect to MQTT broker.");
end

% Topic para suscribirse y publicar
topic = "trubits/mqTop48";
subscribe(mqClient, topic);

% Publicar un mensaje simple
write(mqClient, topic, "70");
pause(1);
write(mqClient, topic, "100");
pause(1);

% Leer mensaje
msg = read(mqClient, Topic=topic);
disp("Dato recibido: " + msg.Data);

% Crear struct y enviar JSON
dato.Sensor = "Temperatura";
dato.Valor = 26.5;
dato.Unidad = "C";
jsonData = jsonencode(dato);

write(mqClient, topic, jsonData);
disp("Dato enviado en JSON: " + jsonData);
pause(1);

% Leer JSON y decodificar
msg = read(mqClient, Topic=topic);
pause(1);

disp("Dato recibido en bruto: " + msg.Data);
datoRecibido = jsondecode(msg.Data);
disp(datoRecibido);