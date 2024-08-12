% Autor: Carlos Tivan , Jefferson Chanaluisa 
% Fecha : 13/11/2023
% objectPlot - Dibuja un objeto 3D rotado, escalado y trasladado.
%
% Sintaxis:
%    object = objectPlot(x, y, z, psi, scale, color)
%
% Entradas:
%    x - Traslación en el eje X.
%    y - Traslación en el eje Y.
%    z - Traslación en el eje Z.
%    psi - Ángulo de rotación en el eje Z (en radianes).
%    scale - Factor de escala aplicado al objeto.
%    color - Color de la superficie del objeto.
%
% Salidas:
%    object - Maneja el objeto gráfico creado por la función `patch`.
%
% Descripción:
%    Esta función toma las coordenadas de vértices de un objeto 3D almacenado globalmente en `Object.parte1Vertices`,
%    las rota alrededor del eje Z según el ángulo `psi`, las escala por un factor `scale`, y las traslada a la posición
%    especificada por `(x, y, z)`. Luego, se genera y devuelve un objeto gráfico utilizando la función `patch`, 
%    que dibuja la superficie del objeto en un gráfico 3D.
%
% Variables globales:
%    Object - Estructura global que contiene las caras y vértices del objeto 3D.
%
% Ejemplo:
%    % Definir parámetros
%    x = 5; y = 10; z = 15; psi = pi/4; scale = 2; color = 'red';
%    
%    % Dibujar objeto
%    object = objectPlot(x, y, z, psi, scale, color);
%
% Notas:
%    - Asegúrate de que la estructura global `Object` esté definida antes de llamar a esta función.
%    - La matriz de rotación `Rz` se calcula para rotar el objeto alrededor del eje Z.
%    - Los vértices se transforman mediante la matriz de rotación, luego se escalan y finalmente se trasladan.
%    - `patch` es utilizado para crear la visualización del objeto con las caras y vértices especificados.
%
% Ver también:
%    patch, cos, sin
%

function object = objectPlot(x,y,z,psi,scale,color)
global Object

% Rotación
Rz=[cos(psi),-sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1]; % Matrix de rotacion en el eje z
robotPatch=Rz*Object.parte1Vertices; 

% Traslación y escala
robotPatch(1, :) = robotPatch(1, :) * scale + x;
robotPatch(2, :) = robotPatch(2, :) * scale + y;
robotPatch(3, :) = robotPatch(3, :) * scale + z;

% Creación del objeto gráfico
object = patch('Faces', Object.parte1Faces, 'Vertices', robotPatch', 'FaceColor', color, 'EdgeColor', 'none');
