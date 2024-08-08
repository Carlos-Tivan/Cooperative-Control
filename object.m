function object
    % Define una nueva función llamada 'object'. Esta función no toma parámetros de entrada ni de salida.
    
global Object
    % Declara una variable global llamada 'Object' que será accesible desde cualquier
    % función o script que también la declare como global.
    
    estanteria = stlRead('estanteria.stl');
    % Llama a la función 'stlRead' para leer el archivo STL llamado 'estanteria.stl'.
    % 'stlRead' devuelve una estructura con los datos del modelo 3D, incluyendo vértices y caras.
    % El archivo STL está en metros y en formato binario.
    
    Object.parte1Vertices = estanteria.vertices';
    % Asigna los vértices del modelo 3D a la propiedad 'parte1Vertices' del objeto global 'Object'.
    % La transposición de los vértices (indicada por el apóstrofo ') se realiza para ajustar el formato.
    
    Object.parte1Faces = estanteria.faces;
    % Asigna las caras del modelo 3D a la propiedad 'parte1Faces' del objeto global 'Object'.
    
end
% Marca el final de la definición de la función 'object'.
