% Autor:
%       Carlos Tivan - Jefferson Chanaluisa 
% Fecha: 13/11/2023
% MobileRobot - Carga y almacena las partes de un robot móvil desde archivos STL.
%
% Descripción:
%    Esta función lee los archivos STL que representan las partes de un robot móvil y almacena sus vértices y caras
%    en una estructura global llamada `Uniciclo`. La función utiliza la función `stlRead` para leer los archivos STL,
%    que deben estar exportados en metros y en formato binario.
%
% Archivos requeridos:
%    - parte1.stl: Archivo STL que contiene la primera parte del robot.
%    - parte2.stl: Archivo STL que contiene la segunda parte del robot.
%    - parte3.stl: Archivo STL que contiene la tercera parte del robot.
%
% Variables globales:
%    Uniciclo - Estructura global que almacena los vértices y las caras de las tres partes del robot móvil.
%               Contiene los siguientes campos:
%               - parte1Vertices: Vértices de la primera parte del robot.
%               - parte1Faces: Caras de la primera parte del robot.
%               - parte2Vertices: Vértices de la segunda parte del robot.
%               - parte2Faces: Caras de la segunda parte del robot.
%               - parte3Vertices: Vértices de la tercera parte del robot.
%               - parte3Faces: Caras de la tercera parte del robot.
%
% Ejemplo:
%    % Inicializar el robot móvil
%    MobileRobot;
%
%    % Acceder a los vértices y caras de la primera parte del robot
%    global Uniciclo;
%    verticesParte1 = Uniciclo.parte1Vertices;
%    facesParte1 = Uniciclo.parte1Faces;
%
% Notas:
%    - Asegúrate de que los archivos STL (`parte1.stl`, `parte2.stl`, `parte3.stl`) estén en el mismo directorio que
%      el script, o proporciona la ruta completa a los archivos.
%    - Los archivos STL deben estar en metros y exportados en formato binario para ser leídos correctamente por `stlRead`.
%
%


function MobileRobot
% Leer los archivos STL y obtener los vértices y caras
parte1 = stlRead('parte1.stl'); % Exportar STL en metros y formato binario
parte2 = stlRead('parte2.stl'); % Exportar STL en metros y formato binario
parte3 = stlRead('parte3.stl'); % Exportar STL en metros y formato binario

% Definir la variable global para almacenar las partes del robot
global Uniciclo;

% Almacenar los vértices y caras de cada parte en la estructura global
Uniciclo.parte1Vertices = parte1.vertices';
Uniciclo.parte1Faces = parte1.faces;

Uniciclo.parte2Vertices = parte2.vertices';
Uniciclo.parte2Faces = parte2.faces;

Uniciclo.parte3Vertices = parte3.vertices';
Uniciclo.parte3Faces = parte3.faces;


