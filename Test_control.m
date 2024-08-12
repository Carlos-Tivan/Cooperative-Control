
%%%%%% EJEMPLO DE INICIALIZACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Inicialización de variables
x1 = 0; % Posición inicial en el eje X
y1 = 0; % Posición inicial en el eje Y
phi = 45*(pi/180); % Ángulo de orientación inicial del robot en radianes (45 grados convertido a radianes)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION VIRTUAL ROBOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Configuracion de escena

scene=figure;  % Crear figura (Escena)
sizeScreen=get(0,'ScreenSize'); % Retorna el tamaño de la pantalla del computador
set(scene,'position',sizeScreen); % Congigurar tamaño de la figura
axis equal; % Establece la relación de aspecto para que las unidades de datos sean las mismas en todas las direcciones.
axis([-2 2 -2 2 0 3]); % Ingresar limites minimos y maximos en los ejes x y z [minX maxX minY maxY minZ maxZ]
view([0 45]); % Orientacion de la figura
grid on; % Mostrar líneas de cuadrícula en los ejes
subtitle('Carlos Tivan - Jefferson Chanaluisa');
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); % Etiqueta de los ejes
camlight right % Luz para la escena

% Grafico del robot en la posicion inicial
MobileRobot; %Función que define la estructura del robot
scale = 1; % Define la escala para el gráfico del robot
H1=MobilePlot(x1(1),y1(1),phi(1),scale,'b'); % Grafica el robot en la posición inicial (0,0) con orientación de 45 grados y color azul

