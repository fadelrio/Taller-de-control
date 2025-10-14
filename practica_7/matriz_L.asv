
T_s = 0.02;

p1 = -8.0022 + 2.9579i;
p2 = -8.0022 - 2.9579i;

k = 28;

A_d = [1,T_s;-p1*p2*T_s,1-(p1+p2)*T_s];

B_d = [0;k*T_s];

C_d = [1,0];

D_d = 0;

L = acker(A_d', C_d',[exp(-T_s*(sqrt(p1*p2))*3.75),exp(-T_s*(sqrt(p1*p2))*3.75)]);
L = L'