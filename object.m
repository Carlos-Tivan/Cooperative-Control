function object
global Object

estanteria = stlRead('estanteria.stl'); % Exportar stl en metros y formato binario

Object.parte1Vertices=estanteria.vertices';
Object.parte1Faces=estanteria.faces;
end

