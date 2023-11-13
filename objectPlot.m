function object = objectPlot(x,y,z,psi,scale,color)
global Object

% Rotacion 

Rz=[cos(psi), -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1]; % Matrix de rotacion en el eje z
robotPatch=Rz*Object.parte1Vertices; 

% Traslacion y escala

robotPatch(1,:)=robotPatch(1,:)*scale+x;
robotPatch(2,:)=robotPatch(2,:)*scale+y;
robotPatch(3,:)=robotPatch(3,:)*scale+z;

     
object = patch('Faces',Object.parte1Faces,'Vertices',robotPatch','FaceColor',color,'EdgeColor','none');


