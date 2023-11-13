

%Modelo Cinematico 

clc
clear 
close 

%{

NOTA: Syms usado para crear simbolos indicando las,
      variables a utilizar hx1, hy1, hx2 , hy2 defiendo 
      una parte real
%}

syms hx1 hy1 hx2 hy2 real 



%Ecuacion 1 

hx  = (hx1+hx2)/2; 
hy  = (hy1+hy2)/2;

%Ecuacion 2 

d = sqrt((hx2-hx1)^2+(hy2-hy1)^2); %Formula de distancia 

angle_betha =  atan2(hy2-hy1,hx2-hx1);


%Matrix JF ec1 

Jf11 = simplify(diff(hx,hx1)); %row 1 and col 1
Jf12 = simplify(diff(hx,hy1)); %row 1 and col 2
Jf13 = simplify(diff(hx,hx2)); %row 1 and col 3
Jf14 = simplify(diff(hx,hy2)); %row 1 and col 4

%%Print results 

Jf11
Jf12
Jf13
Jf14

%Matrix JF ec2 

Jf21 = simplify(diff(hy,hx1)); %row 2 and col 1
Jf22 = simplify(diff(hy,hy1)); %row 2 and col 2
Jf23 = simplify(diff(hy,hx2)); %row 2 and col 3
Jf24 = simplify(diff(hy,hy2)); %row 2 and col 4

%%Print results 

Jf21
Jf22
Jf23
Jf24

%Matrix JF ec3 

Jf31 = simplify(diff(d,hx1)); %row 3 and col 1
Jf32 = simplify(diff(d,hy1)); %row 3 and col 2
Jf33 = simplify(diff(d,hx2)); %row 3 and col 3
Jf34 = simplify(diff(d,hy2)); %row 3 and col 4

%%Print results 

Jf31
Jf32
Jf33
Jf34

%Matrix JF ec4

Jf41 = simplify(diff(angle_betha,hx1)); %row 4 and col 1
Jf42 = simplify(diff(angle_betha,hy1)); %row 4 and col 2
Jf43 = simplify(diff(angle_betha,hx2)); %row 4 and col 3
Jf44 = simplify(diff(angle_betha,hy2)); %row 4 and col 4

%%Print results 

Jf41
Jf42
Jf43 
Jf44







