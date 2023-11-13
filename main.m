%BY: Carlos Tivan Jefferson Chanaluisa 
%DATE: 13/11/2023 

clear %elimina variables 
close all  %cierra todas las figuras graficas 
clc %limpia la ventana de windows 


% Tiempo de simulacion 

tiempo_final = 20; %tiempo de simulacion en segundos 
tiempo_muestreo = 0.1; %tiempo de muestreo en segundos 

%array de tiempo 

tiempo = 0:tiempo_muestreo :tiempo_final; 

%longitud del vector 
long = length(tiempo);
disp('La longitud del vector es de:')
disp(long)

%Parametro del robot 

distancia_A = 0.3; %distancia hacia el punto de control en metros (m)

%%%Condiciones del robot con el angulo 
%posicion centro del eje 

x1 = zeros(1,long+1); %posicion en el centro del eje que une las ruedas al eje x
y1 = zeros(1,long+1); %posicion en el centro del eje que une las ruedas al eje y

x1(1)= 0; %1posicion inicial eje x (1m) 
y1(1)= -0; %-1posicion inicial eje y (-1m)

%%%Modelo geometrico

hx = zeros(1,long+1); %posicion en el punto de control eje x dado en metros (m)
hy = zeros(1,long+1); %posicion en el punto de control eje y dado en metros (m)
phi = zeros(1,long+1); %orientacion del robot en rad

phi(1) = 0; %orientacion inicial del robot 

%Valores en x
hx(1) = x1(1) + distancia_A*cos(phi(1)); %posicion en el punto de control eje x 
disp('Posicion en el punto X');
disp(hx(1));

%Valores en y 
hy(1) = y1(1) + distancia_A*sin(phi(1)); %posicion en el punto de control eje y
disp('Posicion en el punto Y');
disp(hy(1));

% Velocidades de referencia 

%nota: la funciones crea una matriz o vector de valores 1 

%velocidad angular
velocidad_angular = 0.0*ones(1,long);

%velocidad lineal 
velocidad_lineal = -0.4*ones(1,long); %0.2


%%%Bucle de iteracion 
%integro valores de velocidad angular 
%metodo euler 

for i=1:long
    
    %Modelo cinematico 
    
    phi(i+1)=phi(i)+velocidad_angular(i)*tiempo_muestreo; %orientacion en el tiempo de muestreo
    
    %Punto central 
    xp1 = velocidad_lineal(i)*cos(phi(i+1));
    yp1 = velocidad_lineal(i)*sin(phi(i+1));
    
    x1(i+1) = tiempo_muestreo*xp1+x1(i); 
    y1(i+1) = tiempo_muestreo*yp1+y1(i);    
    
    %Posicion punto de interes mediante el modelo geometrico
    hx(1) = x1(i+1) + distancia_A*cos(phi(i+1)); %desplazamiento en X
    hy(1) = y1(i+1) + distancia_A*sin(phi(i+1)); %desplazamiento en Y
    
end 


    %Simulacion 
    scene = figure; % Creates a new figure object and assigns it to the variable "scene"
    sizeScreen = get(0,'ScreenSize'); % Retrieves the screen size of the computer and stores it in the variable "sizeScreen"
    set(scene,'position',sizeScreen); % Sets the position and size of the figure to match the screen size of the computer
    axis equal ; % escala entre los ejes x,y,z 
    axis([-20 20 -20 20 0 10]); % Sets the limits of the x, y, and z-axes -10, 10, -10, 10, 0, 3
    view([0,45]); %orientacion de la figura 
    grid on; %,muestra cuadricula en el plano 
    camlight right %luz en la escena derecha , izquierda 
    campos([180,-90,120]); %130 -60 50 configura la vista tridimencional Angulo de vista , Angulo de elevacion , Distancia
    title('Simulacion'); % Titulo de la simulacion 
    subtitle('Carlos Tivan - Jefferson Chanaluisa');
    xlabel('x(m)');  % Adds a label to the x-axis
    ylabel('y(m)'); % Adds a label to the y-axis
    zlabel('z(m)'); % Adds a label to the z-axis

%Muestro la posicion inicial 

% llamo a la funcion robot()
MobileRobot;
scale = 2 ; 

H1 = MobilePlot(x1(1),y1(1),phi(1),scale,'b');
hold on 


% Grafica de trayectoria 
H2 = plot(hx(1),hy(1),'k','lineWidth',2);

% Inicio del ciclo for 
step = 20; % pasos de la simulacion

for k=1:step:long
    
    delete(H1);
    delete(H2);
    H1 = MobilePlot(x1(k),y1(k),phi(k),scale,'b'); % scale ajusta el tamaño del grafico  
    H2 = plot(hx(1:k),hy(1:k),'r','lineWidth',2); % lineWidth especifica el grosor de la linea a graficar 
    pause(0.1); 
    
end 




