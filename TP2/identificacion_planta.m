alfa = 0;
beta = 0;
gamma = 0;

T_s = 1/50;

A = [alfa;beta;gamma];
Y = out.Y(3:142);
X = [out.tita(3:142), out.Y(2:(142-1)),out.Y(1:(142-2))];


A = inv(transpose(X)*X)*transpose(X)*Y;

polo = (1/T_s)*log(A(3));

ganancia = -9.8*(1+polo)/((exp(T_s)-1)*(exp(T_s)+A(3)));

P = zpk([],[0 , -10], ganancia)
P_d = zpk([],[1 A(3)],A(1),T_s)
%%
close all;

t = 1/50*(1:142-1);
figure;
plot(t, out.Y(1:(142-1)));
hold on;
Y_1 = lsim(P,out.tita(1:(142-1)),t);
plot(t,Y_1)
legend

figure;
step(P);