clear
close all
clc

%%% TIEMPO 

tf = 30;             % Tiempo de simulacion en segundos (s)
ts = 0.1;            % Tiempo de muestreo en segundos (s)
t = 0: ts: tf;       % Vector de tiempo
N = length(t);       % Muestras

%%% PARAMETROS DEL ROBOT 

a1 = 0.5; % Distancia hacia el punto de control en metros (m)
a2 = 0.5; % Distancia hacia el punto de control en metros (m)
hRobot = 0.25; % Alto de la plataforma

%%% CONDICIONES INICIALES

% Robot 1 (IZQUIERDA)

x1 = zeros (1,N+1); % Posición en el centro del eje que une las ruedas (eje x) en metros (m)
y1 = zeros (1,N+1); % Posición en el centro del eje que une las ruedas (eje y) en metros (m)

x1(1) = 2;          % Posicion inicial eje x
y1(1) = -2;        % Posicion inicial eje y

%%% MODELO GEOMETRICO 

hx1 = zeros(1, N+1);  % Posicion en el punto de control (eje x) en metros (m)
hy1 = zeros(1, N+1);  % Posicion en el punto de control (eje y) en metros (m)
phi1 = zeros(1, N+1); % Orientacion del robot en radianes (rad)

phi1(1) = 0;   % Orientacion inicial del robot

hx1(1) = x1(1) + a1*cos(phi1(1)); % Posicion en el punto de control del robot en el eje x
hy1(1) = y1(1) + a1*sin(phi1(1)); % Posicion en el punto de control del robot en el eje y


% Robot 2 (DERECHA)

x2 = zeros (1,N+1); % Posición en el centro del eje que une las ruedas (eje x) en metros (m)
y2 = zeros (1,N+1); % Posición en el centro del eje que une las ruedas (eje y) en metros (m)

x2(1) = 2;          % Posicion inicial eje x
y2(1) = 1;        % Posicion inicial eje y

%%% MODELO GEOMETRICO 

hx2 = zeros(1, N+1);  % Posicion en el punto de control (eje x) en metros (m)
hy2 = zeros(1, N+1);  % Posicion en el punto de control (eje y) en metros (m)
phi2 = zeros(1, N+1); % Orientacion del robot en radianes (rad)

phi2(1) = 0;   % Orientacion inicial del robot

hx2(1) = x2(1) + a2*cos(phi2(1)); % Posicion en el punto de control del robot en el eje x
hy2(1) = y2(1) + a2*sin(phi2(1)); % Posicion en el punto de control del robot en el eje y


%%%% MODELO DEL OBJETO A TRANSPORTAR 

hx = zeros(1, N+1);       % Posicion en el punto de control (eje x) en metros (m)
hy = zeros(1, N+1);       % Posicion en el punto de control (eje y) en metros (m)
d = zeros(1, N+1);     % Distancia del objeto en metros (m)
betha = zeros(1, N+1); % Orientación del objeto en radianes (rad)

hx(1)=(hx1(1)+hx2(1))/2; % Posicion en el punto de control (eje x) en metros (m)
hy(1)=(hy1(1)+hy2(1))/2; % Posicion en el punto de control (eje x) en metros (m)

d(1)=sqrt((hx2(1)-hx1(1))^2+(hy2(1)-hy1(1))^2); % Distancia inicial del objeto en metros (m)

betha(1)=atan2(hy2(1)-hy1(1),hx2(1)-hx1(1)); % Orientación inicial del objeto en radianes (rad)

%%% POSICION DESEADA 

hxd = -7;     % Posicion deseada eje x
hyd = 6.5;   % Posicion deseada eje y

dd=2.5;              % Longitud deseada del objeto en metros (m)
betad=45*(pi/180); % % Orientación deseada del objeto a transportar en radianes (rad)


%%% VARIABLES PARA ERRORES 

hxe = zeros(1,N);   
hye = zeros(1,N);    
de = zeros(1,N);
bethae = zeros(1,N);

hxe1 = zeros(1,N);   
hye1 = zeros(1,N); 
hxe2 = zeros(1,N);   
hye2 = zeros(1,N); 

%%% VELOCIDADES DE REFERENCIA 

uRef1 = zeros(1,N); 
wRef1 = zeros(1,N); 
uRef2 = zeros(1,N); 
wRef2 = zeros(1,N); 

%%%% VELOCIDADES DE DESEADAS PARA CADA ROBOT

hxd1 = zeros(1,N); 
hyd1 = zeros(1,N); 
hxd2 = zeros(1,N); 
hyd2 = zeros(1,N); 

%%% BUCLE DE CONTROL 

for k=1:N 

    %%%%%%%%%%%%%%%%%%%%%% CONTROLADOR DE FORMACION %%%%%%%%%%%%%%%%%%%%%%%

    % a) ERROR
    
    hxe(k)=hxd-hx(k);      
    hye(k)=hyd-hy(k);     
    de(k) =dd-d(k);           
    bethae(k)=betad-betha(k); 
    he=[hxe(k);hye(k);de(k);bethae(k)];      

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

    % e) CALCULO LA TRAYECTORIA DESEADA PARA CADA ROBOT (EULER HACIA ADELANTE)

    if k==1

        hxd1(k)=hx1(k);    
        hyd1(k)=hy1(k);              

        hxd2(k)=hx2(k); 
        hyd2(k)=hy2(k); 

    else
        hxd1(k)=hxd1(k-1)+ts*qpf(1);    
        hyd1(k)=hyd1(k-1)+ts*qpf(2);                 

        hxd2(k)=hxd2(k-1)+ts*qpf(3);
        hyd2(k)=hyd2(k-1)+ts*qpf(4); 

    end

    %%%%%%%%%%%%%%%%%%%% CONTROLADOR ROBOT 1 IZQUIERDA %%%%%%%%%%%%%%%%%%%%
    
    % a) ERROR
    
    hxe1(k) = hxd1(k) - hx1(k);           
    hye1(k) = hyd1(k) - hy1(k);        

     he1 = [hxe1(k);hye1(k)];

    % b) MATRIX JACOBIANA

    J111 = cos(phi1(k));
    J112 = -a1*sin(phi1(k));

    J121 = sin(phi1(k));
    J122 = a1*cos(phi1(k));

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

    phi1(k+1)=phi1(k)+wRef1(k)*ts;

    xp1=uRef1(k)*cos(phi1(k+1));
    yp1=uRef1(k)*sin(phi1(k+1));

    x1(k+1)=ts*xp1+ x1(k);
    y1(k+1)=ts*yp1+ y1(k);

    hx1(k+1)=x1(k+1)+a1*cos(phi1(k+1)); 
    hy1(k+1)=y1(k+1)+a1*sin(phi1(k+1));

    %%%%%%%%%%%%%%%%%%%% CONTROLADOR ROBOT 2 DERECHO %%%%%%%%%%%%%%%%%%%%
    
    % a) ERROR
    hxe2(k) = hxd2(k) - hx2(k);
    hye2(k) = hyd2(k) - hy2(k);

    he2 = [hxe2(k);hye2(k)];

    % b) MATRIX JACOBIANA

    J211 = cos(phi2(k));
    J212 = -a2*sin(phi2(k));

    J221 = sin(phi2(k));
    J222 = a2*cos(phi2(k));

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
    
    phi2(k+1)=phi2(k)+wRef2(k)*ts; 

    xp2=uRef2(k)*cos(phi2(k+1));
    yp2=uRef2(k)*sin(phi2(k+1));

    x2(k+1)=ts*xp2 + x2(k);
    y2(k+1)=ts*yp2 + y2(k);

    hx2(k+1)=x2(k+1)+a2*cos(phi2(k+1)); 
    hy2(k+1)=y2(k+1)+a2*sin(phi2(k+1));

    %%%¿ MODELO GEOMÉTRICO DEL OBJETO 

    hx(k+1)=(hx1(k+1)+hx2(k+1))/2;
    hy(k+1)=(hy1(k+1)+hy2(k+1))/2;

    d(k+1)=sqrt((hx2(k+1)-hx1(k+1))^2+(hy2(k+1)-hy1(k+1))^2);  

    betha(k+1)=atan2((hy2(k+1)-hy1(k+1)),(hx2(k+1)-hx1(k+1))); 
    
end

%%% SIMULACION VIRTUAL ROBOT 

% a) Configuracion de escena

scene=figure;  % Crear figura (Escena)
sizeScreen=get(0,'ScreenSize'); % Retorna el tamaño de la pantalla del computador
set(scene,'Color','white');
set(gca,'FontWeight','bold') ;
set(scene,'position',sizeScreen); % Configura el tamaño de la figura
axis equal; % Establece la relación de aspecto para que las unidades de datos sean las mismas en todas las direcciones.
axis([-15 15 -15 15 0 5]); % Ingresar limites minimos y maximos en los ejes x y z [minX maxX minY maxY minZ maxZ]
view([0 45]); % Orientacion de la figura
campos([130,-60,50]);
grid on; % Mostrar líneas de cuadrícula en los ejes
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); % Etiqueta de los ejes
title('Simulación Transporte Cooperativo');
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
cObject = [0.6 0.4 0.2]; % color del objeto
scaleObject = 3; % color del objeto
H3=objectPlot(hx(1),hy(1),scale*hRobot,betha(1),scaleObject,cObject);

% d) Graficar Trayectorias
H4=plot(hx1(1),hy1(1),'g','lineWidth',1);
H5=plot(hx2(1),hy2(1),'b','lineWidth',1);
H6=plot(hx(1),hy(1),'r','lineWidth',1);

% e) Graficar posicion deseada

H7=plot(hxd,hyd,'Or','lineWidth',1);


% f) Bucle de simulacion de movimiento del robot

step=6; % pasos para simulacion

for i=1:step:N

    delete(H1);    
    delete(H2);
    delete(H3);
    delete(H4);
    delete(H5);
    delete(H6);

    H1=MobilePlot(x1(i),y1(i),phi1(i),scale,'r');
    H2=MobilePlot(x2(i),y2(i),phi2(i),scale,'b');
    H3=objectPlot(hx(i),hy(i),scale*hRobot,betha(i),scaleObject,cObject);
    H4=plot(hx1(1:i),hy1(1:i),'g','lineWidth',1);
    H5=plot(hx2(1:i),hy2(1:i),'b','lineWidth',1);
    H6=plot(hx(1:i),hy(1:i),'r','lineWidth',1);
    pause(ts);

end
