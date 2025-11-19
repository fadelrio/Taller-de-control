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

D = [0;
    0];

T_s = 0.02;

A_d = eye(4)+A*T_s;

B_d = B*T_s;

C_d = C;

D_d = D;

K = place(A_d,-B_d,[exp(-T_s*15)+2e-4,exp(-T_s*5)+3e-4, exp(-T_s*15)+4e-4, exp(-T_s*8)])

eig(A_d + B_d*K)

K

L = place(A_d',C_d',[exp(-T_s*31.1333*2)+2e-4,exp(-T_s*31.1333*1)+3e-4, exp(-T_s*31.1333*5)+4e-4, exp(-T_s*31.1333*5)]);

L = L'

F = pinv(C_d*inv(eye(4)-(A_d+B_d*K))*B_d)

