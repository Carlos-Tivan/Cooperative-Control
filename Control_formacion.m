clear
close all
clc

% TIEMPO 

tiempo_final = 20; %tiempo de simulacion en segundos
tiempo_muestreo = 0.1; %tiempo de muestreo en segundos
tiempo = 0:tiempo_muestreo :tiempo_final; % vector tiempo 
long = length(tiempo); % Muestras 


% PARAMETROS DEL ROBOT 

robot_izquierda = 0.2; % Distancia hacia el punto de control en metros (m)
robot_derecha = 0.2; % Distancia hacia el punto de control en metros (m)
altura_robot = 0.3; % Alto de la plataforma cm 

% CONDICIONES INICIALES

%% Robot 1 (IZQUIERDA) 
x1 = zeros (1,long+1); % Posición en el centro del eje que une las ruedas (eje x) en metros (m)
y1 = zeros (1,long+1); % Posición en el centro del eje que une las ruedas (eje y) en metros (m)
x1(1) = 1;          % Posicion punto de interes dentro del eje X 
y1(1) = -1;        % Posicion de interes en eje Y 
%%
% Modelo geometrico de punto de interes 

hx1 = zeros(1, long+1);  % Posicion en el punto de control (eje x) en metros (m)
hy1 = zeros(1, long+1);  % Posicion en el punto de control (eje y) en metros (m)
phi1 = zeros(1, long+1); % Orientacion del robot en radianes (rad)
phi1(1) = 0;   % Orientacion inicial del robot
hx1(1) = x1(1) + robot_izquierda*cos(phi1(1)); % Posicion en el punto de control del robot en el eje x
hy1(1) = y1(1) + robot_izquierda*sin(phi1(1)); % Posicion en el punto de control del robot en el eje y


%% Robot 2 (DERECHA)

x2 = zeros (1,long+1); % Posición en el centro del eje que une las ruedas (eje x) en metros (m)
y2 = zeros (1,long+1); % Posición en el centro del eje que une las ruedas (eje y) en metros (m)
x2(1) = 2;          % 2 Posicion inicial eje x
y2(1) = 1;        %  1 Posicion inicial eje y
%%


% Modelo geometrico punto de interes 
hx2 = zeros(1, long+1);  % Posicion en el punto de control (eje x) en metros (m)
hy2 = zeros(1, long+1);  % Posicion en el punto de control (eje y) en metros (m)
phi2 = zeros(1, long+1); % Orientacion del robot en radianes (rad)
phi2(1) = 0 ;   % Orientacion inicial del robot 0
hx2(1) = x2(1) + robot_derecha*cos(phi2(1)); % Posicion en el punto de control del robot en el eje x
hy2(1) = y2(1) + robot_derecha*sin(phi2(1)); % Posicion en el punto de control del robot en el eje y


% Modelo geometrico del objeto a transportar  
hx = zeros(1, long+1);       % Posicion en el punto de control (eje x) en metros (m)
hy = zeros(1, long+1);       % Posicion en el punto de control (eje y) en metros (m)
d = zeros(1, long+1);     % Distancia del objeto en metros (m)
betha = zeros(1, long+1); % Orientación del objeto en radianes (rad)
hx(1)=(hx1(1)+hx2(1))/2; % Posicion en el punto de control (eje x) en metros (m)
hy(1)=(hy1(1)+hy2(1))/2; % Posicion en el punto de control (eje x) en metros (m)
d(1)=sqrt((hx2(1)-hx1(1))^2+(hy2(1)-hy1(1))^2); % Distancia inicial del objeto en metros (m)
betha(1)=atan2(hy2(1)-hy1(1),hx2(1)-hx1(1)); % Orientación inicial del objeto en radianes (rad)


%% POSICION DESEADA DEL OBJETO 

hxd = 7;     % Posicion deseada eje x 6
hyd = -7;   % Posicion deseada eje y -7,7 
distancia= 2; % distancia del objeto y los robots en metros (m)
angulo= 0*(pi/180); % % Orientación deseada del objeto a transportar en radianes (rad)
%%

%% VARIABLES PARA ERRORES 
hxe = zeros(1,long);   
hye = zeros(1,long);    
distancia_error = zeros(1,long);
bethae = zeros(1,long);

hxe1 = zeros(1,long);   
hye1 = zeros(1,long); 
hxe2 = zeros(1,long);   
hye2 = zeros(1,long); 

%% 
%% VELOCIDADES DE REFERENCIA 
% Robot 1 
uRef1 = zeros(1,long); 
wRef1 = zeros(1,long); 
% Robot 2
uRef2 = zeros(1,long); 
wRef2 = zeros(1,long); 

% VELOCIDADES DE DESEADAS PARA CADA ROBOT
% Robot 1 
hxd1 = zeros(1,long); 
hyd1 = zeros(1,long); 
% Robot 2
hxd2 = zeros(1,long); 
hyd2 = zeros(1,long); 
%%

%% BUCLE DE CONTROL 
for k=1:long

    %%%%%%%%%%%%%%%%%%%%%% CONTROLADOR DE FORMACION %%%%%%%%%%%%%%%%%%%%%%%

    % a) ERROR
    
    hxe(k)=hxd-hx(k);      
    hye(k)=hyd-hy(k);     
    distancia_error(k) =distancia-d(k);           
    bethae(k)=angulo-betha(k); 


    he=[hxe(k);hye(k);distancia_error(k);bethae(k)];      

    % b) MATRIX JACOBIANA

    JF11 = 1/2;
    JF12 = 0;
    JF13 = 1/2;
    JF14 = 0;

    JF21 = 0;
    JF22 = 1/2;
    JF23 = 0;
    JF24 = 1/2;


    JF31 = (hx1(k) - hx2(k))/((hx1(k) - hx2(k))^2 + (hy1(k) - hy2(k))^2)^(1/2);
    JF32 = (hy1(k) - hy2(k))/((hx1(k) - hx2(k))^2 + (hy1(k) - hy2(k))^2)^(1/2);
    JF33 =  -(hx1(k) - hx2(k))/((hx1(k) - hx2(k))^2 + (hy1(k) - hy2(k))^2)^(1/2);
    JF34 =  -(hy1(k) - hy2(k))/((hx1(k) - hx2(k))^2 + (hy1(k) - hy2(k))^2)^(1/2);



    JF41 =  -(hy1(k) - hy2(k))/((hx1(k) - hx2(k))^2 + (hy1(k) - hy2(k))^2);
    JF42 =  (hx1(k) - hx2(k))/((hx1(k) - hx2(k))^2 + (hy1(k) - hy2(k))^2);
    JF43 =  (hy1(k) - hy2(k))/((hx1(k) - hx2(k))^2 + (hy1(k) - hy2(k))^2);
    JF44 =  -(hx1(k) - hx2(k))/((hx1(k) - hx2(k))^2 + (hy1(k) - hy2(k))^2);


    JF=[JF11 JF12 JF13 JF14 ;...
        JF21 JF22 JF23 JF24 ;...
        JF31 JF32 JF33 JF34 ;...
        JF41 JF42 JF43 JF44 ];


    % c) PARAMETROS DE CONTROL

    K =  0.1*[1 0 0 0;...
            0 1 0 0;...
            0 0 1 0;...
            0 0 0 1];


    % d) LEY DE CONTROL LYAPUNOV

    qpf=pinv(JF)*(K*he); 

    % e) CALCULE LA TRAYECTORIA DESEADA PARA CADA ROBOT (EULER HACIA ADELANTE)
    if k==1

        hxd1(k)=hx1(k);    
        hyd1(k)=hy1(k);              

        hxd2(k)=hx2(k); 
        hyd2(k)=hy2(k); 

    else
        hxd1(k)=hxd1(k-1)+tiempo_muestreo*qpf(1);    
        hyd1(k)=hyd1(k-1)+tiempo_muestreo*qpf(2);                 

        hxd2(k)=hxd2(k-1)+tiempo_muestreo*qpf(3);
        hyd2(k)=hyd2(k-1)+tiempo_muestreo*qpf(4); 

    end

    %%%%%%%%%%%%%%%%%%%% CONTROLADOR ROBOT 1 IZQUIERDA %%%%%%%%%%%%%%%%%%%%
    
    % a) ERROR
    
    hxe1(k) = hxd1(k) - hx1(k);           
    hye1(k) = hyd1(k) - hy1(k);        

     he1 = [hxe1(k);hye1(k)];

    % b) MATRIX JACOBIANA

    J111 = cos(phi1(k));
    J112 = -robot_izquierda*sin(phi1(k));

    J121 = sin(phi1(k));
    J122 = robot_izquierda*cos(phi1(k));

    J1 = [J111 J112 ;...
          J121 J122 ];
      
    % c) PARAMETROS DE CONTROL
    K1= 0.1*[1 0;...
             0 1];
     
    % d) VELOCIDADES DESEADAS
    
    hdp1=[qpf(1);qpf(2)];
    
    % e) LEY DE CONTROL LYAPUNOV

    qp1 = pinv(J1)*(hdp1+K1*he1);

    % f) SEPARAR ACCIONES DE CONTROL
    
    uRef1(k) = qp1(1); 
    wRef1(k) = qp1(2);
    
    %%%%%%%%%%%%%% APLICAR ACCIONES DE CONTROL AL ROBOT %%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%% MODELO CINEMATICO %%%%%%%%%%%%%%%%%%%%%%%%%

    phi1(k+1)=phi1(k)+wRef1(k)*tiempo_muestreo;

    xp1=uRef1(k)*cos(phi1(k+1));
    yp1=uRef1(k)*sin(phi1(k+1));

    x1(k+1)=tiempo_muestreo*xp1+ x1(k);
    y1(k+1)=tiempo_muestreo*yp1+ y1(k);
    hx1(k+1)=x1(k+1)+robot_izquierda*cos(phi1(k+1)); 
    hy1(k+1)=y1(k+1)+robot_izquierda*sin(phi1(k+1));

    %%%%%%%%%%%%%%%%%%%% CONTROLADOR ROBOT 2 DERECHO %%%%%%%%%%%%%%%%%%%%
    
    % a) ERROR
    hxe2(k) = hxd2(k) - hx2(k);
    hye2(k) = hyd2(k) - hy2(k);

    he2 = [hxe2(k);hye2(k)];

    % b) MATRIX JACOBIANA

    J211 = cos(phi2(k));
    J212 = -robot_derecha*sin(phi2(k));

    J221 = sin(phi2(k));
    J222 = robot_derecha*cos(phi2(k));

    J2 = [J211 J212 ;...
          J221 J222 ];
      
      
    % c) PARAMETROS DE CONTROL
    
    K2= 0.1*[1 0;...
             0 1];
    
    % d) VELOCIDADES DESEADAS    
    
    hdp2 =[qpf(3);qpf(4)];
    
    % e) LEY DE CONTROL LYAPUNOV
    
    qp2 = pinv(J2)*(hdp2+K2*he2);
    
    % f) SEPARAR ACCIONES DE CONTROL
    uRef2(k) = qp2(1); 
    wRef2(k) = qp2(2);
    
    %%%%%%%%%%%%%% APLICAR ACCIONES DE CONTROL AL ROBOT %%%%%%%%%%%%%%%%
    
    %%% MODELO CINEMATICO 
    
    phi2(k+1)=phi2(k)+wRef2(k)*tiempo_muestreo; 

    xp2=uRef2(k)*cos(phi2(k+1));
    yp2=uRef2(k)*sin(phi2(k+1));

    x2(k+1)=tiempo_muestreo*xp2 + x2(k);
    y2(k+1)=tiempo_muestreo*yp2 + y2(k);

    hx2(k+1)=x2(k+1)+robot_derecha*cos(phi2(k+1)); 
    hy2(k+1)=y2(k+1)+robot_derecha*sin(phi2(k+1));

    %%%¿ MODELO GEOMÉTRICO DEL OBJETO 

    hx(k+1)=(hx1(k+1)+hx2(k+1))/2;
    hy(k+1)=(hy1(k+1)+hy2(k+1))/2;

    d(k+1)=sqrt((hx2(k+1)-hx1(k+1))^2+(hy2(k+1)-hy1(k+1))^2);  

    betha(k+1)=atan2((hy2(k+1)-hy1(k+1)),(hx2(k+1)-hx1(k+1))); 
    
end

% SIMULACION VIRTUAL ROBOT 

% a) Configuracion de escena

scene=figure;  % Crear figura (Escena)
sizeScreen=get(0,'ScreenSize'); % Retorna el tamaño de la pantalla del computador
set(scene,'Color','white');
set(gca,'FontWeight','bold') ;
set(scene,'position',sizeScreen); % Congigurar tamaño de la figura
axis equal; % Establece la relación de aspecto para que las unidades de datos sean las mismas en todas las direcciones.
axis([-10 10 -10 10 0 4]); % Ingresar limites minimos y maximos en los ejes x y z [minX maxX minY maxY minZ maxZ]
view([0 45]); % Orientacion de la figura
campos([130,-60,50]);
grid on; % Mostrar líneas de cuadrícula en los ejes
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); % Etiqueta de los ejes
title('Transporte Cooperativo');
subtitle('Carlos Tivan - Jefferson Chanaluisa');
camlight right % Luz para la escena
box on; % Mostrar contorno de ejes

% b) Graficar robots en la posicion inicial
MobileRobot;
scale = 2;
H1=MobilePlot(x1(1),y1(1),phi1(1),scale,'r');
hold on
H2=MobilePlot(x2(1),y2(1),phi2(1),scale,'b');

% c) Graficar objeto a transladar 
object;
color_Object = [0.6 0.4 0.2]; % color del objeto
scaleObject = 2; % color del objeto
H3=objectPlot(hx(1),hy(1),scale*altura_robot,betha(1),scaleObject,color_Object);

% d) Graficar Trayectorias
H4=plot(hx1(1),hy1(1),'green','lineWidth',2); %robot 1
H5=plot(hx2(1),hy2(1),'blue','lineWidth',2); %robot 2 
H6=plot(hx(1),hy(1),'red','lineWidth',2); % objeto 

% e) Variable de posicion deseada

H7=plot(hxd,hyd,'Or','lineWidth',2);

%%
%% f) Bucle de simulacion de movimiento del robot
step=20; % pasos de simulacion durante el ciclo for 

for i=1:step:long 
    % Elimina los objetos gráficos anteriores para evitar la superposición
    delete(H1);    
    delete(H2);
    delete(H3);
    delete(H4);
    delete(H5);
    delete(H6);

    % Crea un nuevo gráfico para el Robot 1 con color azul
    H1=MobilePlot(x1(i),y1(i),phi1(i),scale,'blue'); 

    % Crea un nuevo gráfico para el Robot 2 con color cian
    H2=MobilePlot(x2(i),y2(i),phi2(i),scale,'cyan'); 

    % Crea un gráfico para el objeto a transportar con color y escala especificados
    H3=objectPlot(hx(i),hy(i),scale*altura_robot,betha(i),scaleObject,color_Object); 

    % Traza una línea verde que muestra la trayectoria del Robot 1 hasta el paso i
    H4=plot(hx1(1:i),hy1(1:i),'y','lineWidth',2); 

    % Traza una línea azul que muestra la trayectoria del Robot 2 hasta el paso i
    H5=plot(hx2(1:i),hy2(1:i),'b','lineWidth',2); 

    % Traza una línea roja que muestra la trayectoria del objeto hasta el paso i
    H6=plot(hx(1:i),hy(1:i),'r','lineWidth',2); 

    % Pausa por un tiempo de muestreo específico antes de la siguiente iteración
    pause(tiempo_muestreo);

end
clc