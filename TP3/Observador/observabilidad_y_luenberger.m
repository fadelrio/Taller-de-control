clc; %clear all; close all;

A = [0,1,0,0;
    0,-4.7,-25,0;
    0,0,0,1;
    0,0,-262.0676,-39.5509];

B = [0;
    0;
    0;
    114.3906];

C = [1,0,0,0;
    0,0,1,0];

D = 0;


O = [C;
    C*A;
    C*A*A
    C*A*A*A];

Cont = [B, A*B, A*A*B, A*A*A*B];

rank(O)
rank(Cont)

%Este es observable y controlable

C = [1,0,0,0];

O = [C;
    C*A;
    C*A*A
    C*A*A*A];

Cont = [B, A*B, A*A*B, A*A*A*B];

rank(O)
rank(Cont)

%Este también es observable y controlable

C = [0,0,1,0];

O = [C;
    C*A;
    C*A*A
    C*A*A*A];

Cont = [B, A*B, A*A*B, A*A*A*B];

rank(O)
rank(Cont)

%Este ya no es observable pero si controlable 

%%

A = [0,1,0,0;
    0,-4.7,-25,0;
    0,0,0,1;
    0,0,-262.0676,-39.5509];

B = [0;
    0;
    0;
    114.3906];

C = [1,0,0,0;
    0,0,1,0];

D = 0;

T_s = 0.02;

A_d = eye(4)+A*T_s;

B_d = B*T_s;

C_d = C;

D_d = D;

L = place(A_d',C_d',[exp(-T_s*31.1333*2)+2e-4,exp(-T_s*31.1333*1)+3e-4, exp(-T_s*31.1333*5)+4e-4, exp(-T_s*31.1333*5)]);

L = L'
%%

close all;

largo = 500;

inicio_real = 100;

fin_real = inicio_real + largo;

T_s = 1/50;

t = T_s*(1:(largo+1));

figure;
plot(t, out.distancia(inicio_real:fin_real))
hold on;
plot(t, out.distancia_h(inicio_real:fin_real))
legend("Real","Observado")
title("Posición carro")
xlim([0 10])
grid
hold on;

figure;
plot(t, out.tita_barra(inicio_real:fin_real))
hold on;
plot(t, out.tita_barra_h(inicio_real:fin_real))
legend("Real","Observado")
title("Ángulo barra")
xlim([0 10])
grid
hold on;

figure;
plot(t, out.velocidad(inicio_real:fin_real))
hold on;
plot(t, out.velocidad_h(inicio_real:fin_real))
legend("Real","Observado")
title("Velocidad carro")
xlim([0 10])
grid
hold on;

figure;
plot(t, out.velocidad_ang(inicio_real:fin_real))
hold on;
plot(t, out.velocidad_ang_h(inicio_real:fin_real))
legend("Real","Observado")
title("Velocidad angular barra")
xlim([0 10])
grid
hold on;

