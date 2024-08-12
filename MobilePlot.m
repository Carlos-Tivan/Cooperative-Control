% Autor: Carlos Tivan , Jefferson Chanaluisa 
% Fecha : 13/11/2023

function Mobile_Graph = MobilePlot(x, y, angz, scale, color)

% MobilePlot - Dibuja un robot móvil 3D en una posición, orientación y escala específicas.
%
% Sintaxis:
%    Mobile_Graph = MobilePlot(x, y, angz, scale, color)
%
% Entradas:
%    x - Traslación en el eje X.
%    y - Traslación en el eje Y.
%    angz - Ángulo de rotación alrededor del eje Z (en radianes).
%    scale - Factor de escala aplicado al robot.
%    color - Color de la primera parte del robot.
%
% Salidas:
%    Mobile_Graph - Arreglo de manejadores gráficos de los objetos creados por la función `patch` para cada parte del robot.
%
% Descripción:
%    Esta función toma los vértices de las partes de un robot móvil, los rota alrededor del eje Z según el ángulo `angz`,
%    los escala y traslada a la posición especificada por `(x, y)`. Luego, se dibujan las tres partes del robot en un
%    gráfico 3D utilizando la función `patch`, con los colores especificados para cada parte.
%
% Variables globales:
%    Uniciclo - Estructura global que contiene los vértices y caras de las partes del robot móvil.
%
% Ejemplo:
%    % Parámetros de entrada
%    x = 5; y = 10; angz = pi/4; scale = 2; color = 'red';
%
%    % Dibujar el robot móvil
%    Mobile_Graph = MobilePlot(x, y, angz, scale, color);
%
% Notas:
%    - Asegúrate de que la estructura global `Uniciclo` esté definida antes de llamar a esta función.
%    - La matriz de rotación `Rz` se utiliza para rotar los vértices del robot alrededor del eje Z.
%    - Cada parte del robot se dibuja con un color distinto:
%      - La primera parte usa el color especificado por el usuario.
%      - La segunda parte usa color negro ('k').
%      - La tercera parte usa un tono gris claro ([0.8 0.8 0.8]).
%
% Ver también:
%    patch, cos, sin
%


global Uniciclo

% Matriz de rotación Z
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



