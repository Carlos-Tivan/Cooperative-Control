
%%%%%% EJEMPLO DE INICIALIZACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Inicializaci�n de variables
x1 = 0; % Posici�n inicial en el eje X
y1 = 0; % Posici�n inicial en el eje Y
phi = 45*(pi/180); % �ngulo de orientaci�n inicial del robot en radianes (45 grados convertido a radianes)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION VIRTUAL ROBOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Configuracion de escena

scene=figure;  % Crear figura (Escena)
sizeScreen=get(0,'ScreenSize'); % Retorna el tama�o de la pantalla del computador
set(scene,'position',sizeScreen); % Congigurar tama�o de la figura
axis equal; % Establece la relaci�n de aspecto para que las unidades de datos sean las mismas en todas las direcciones.
axis([-2 2 -2 2 0 3]); % Ingresar limites minimos y maximos en los ejes x y z [minX maxX minY maxY minZ maxZ]
view([0 45]); % Orientacion de la figura
grid on; % Mostrar l�neas de cuadr�cula en los ejes
subtitle('Carlos Tivan - Jefferson Chanaluisa');
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); % Etiqueta de los ejes
camlight right % Luz para la escena

% Grafico del robot en la posicion inicial
MobileRobot; %Funci�n que define la estructura del robot
scale = 1; % Define la escala para el gr�fico del robot
H1=MobilePlot(x1(1),y1(1),phi(1),scale,'b'); % Grafica el robot en la posici�n inicial (0,0) con orientaci�n de 45 grados y color azul

