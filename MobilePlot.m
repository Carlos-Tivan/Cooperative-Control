% Autor: Carlos Tivan , Jefferson Chanaluisa 
% Fecha : 13/11/2023

function Mobile_Graph = MobilePlot(x, y, angz, scale, color)

% MobilePlot - Dibuja un robot m�vil 3D en una posici�n, orientaci�n y escala espec�ficas.
%
% Sintaxis:
%    Mobile_Graph = MobilePlot(x, y, angz, scale, color)
%
% Entradas:
%    x - Traslaci�n en el eje X.
%    y - Traslaci�n en el eje Y.
%    angz - �ngulo de rotaci�n alrededor del eje Z (en radianes).
%    scale - Factor de escala aplicado al robot.
%    color - Color de la primera parte del robot.
%
% Salidas:
%    Mobile_Graph - Arreglo de manejadores gr�ficos de los objetos creados por la funci�n `patch` para cada parte del robot.
%
% Descripci�n:
%    Esta funci�n toma los v�rtices de las partes de un robot m�vil, los rota alrededor del eje Z seg�n el �ngulo `angz`,
%    los escala y traslada a la posici�n especificada por `(x, y)`. Luego, se dibujan las tres partes del robot en un
%    gr�fico 3D utilizando la funci�n `patch`, con los colores especificados para cada parte.
%
% Variables globales:
%    Uniciclo - Estructura global que contiene los v�rtices y caras de las partes del robot m�vil.
%
% Ejemplo:
%    % Par�metros de entrada
%    x = 5; y = 10; angz = pi/4; scale = 2; color = 'red';
%
%    % Dibujar el robot m�vil
%    Mobile_Graph = MobilePlot(x, y, angz, scale, color);
%
% Notas:
%    - Aseg�rate de que la estructura global `Uniciclo` est� definida antes de llamar a esta funci�n.
%    - La matriz de rotaci�n `Rz` se utiliza para rotar los v�rtices del robot alrededor del eje Z.
%    - Cada parte del robot se dibuja con un color distinto:
%      - La primera parte usa el color especificado por el usuario.
%      - La segunda parte usa color negro ('k').
%      - La tercera parte usa un tono gris claro ([0.8 0.8 0.8]).
%
% Ver tambi�n:
%    patch, cos, sin
%


global Uniciclo

% Matriz de rotaci�n Z
Rz = [cos(angz), -sin(angz), 0; sin(angz), cos(angz), 0; 0, 0, 1];

% Transformar y dibujar la primera parte del robot
robotPatch = Rz * Uniciclo.parte1Vertices;
robotPatch(1, :) = robotPatch(1, :) * scale + x;
robotPatch(2, :) = robotPatch(2, :) * scale + y;
robotPatch(3, :) = robotPatch(3, :) * scale;
Mobile_Graph(1) = patch('Faces', Uniciclo.parte1Faces, 'Vertices', robotPatch', 'FaceColor', color, 'EdgeColor', 'none');

% Transformar y dibujar la segunda parte del robot
robotPatch = Rz * Uniciclo.parte2Vertices;
robotPatch(1, :) = robotPatch(1, :) * scale + x;
robotPatch(2, :) = robotPatch(2, :) * scale + y;
robotPatch(3, :) = robotPatch(3, :) * scale;
Mobile_Graph(2) = patch('Faces', Uniciclo.parte2Faces, 'Vertices', robotPatch', 'FaceColor', 'k', 'EdgeColor', 'none');

% Transformar y dibujar la tercera parte del robot
robotPatch = Rz * Uniciclo.parte3Vertices;
robotPatch(1, :) = robotPatch(1, :) * scale + x;
robotPatch(2, :) = robotPatch(2, :) * scale + y;
robotPatch(3, :) = robotPatch(3, :) * scale;
Mobile_Graph(3) = patch('Faces', Uniciclo.parte3Faces, 'Vertices', robotPatch', 'FaceColor', [0.8, 0.8, 0.8], 'EdgeColor', 'none');



