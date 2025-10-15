alfa = 0;
beta = 0;
gamma = 0;

T_s = 1/50;

A = [alfa;beta;gamma];
Y = out.Y(3:end);
X = [out.tita(3:end), out.Y(2:(end-1)),out.Y(1:(end-2))];


A = inv(transpose(X)*X)*transpose(X)*Y;

polo = (1/T_s)*log(A(3));

ganancia = -A(1)*(1+polo)/((exp(T_s)-1)*(exp(T_s)+A(3)));

P = zpk([],[0 , polo], ganancia)
P_d = zpk([],[1 A(3)],A(1),T_s)
%%
close all;

t = 1/50*(0:2576-1);
figure;
plot(t, out.Y);
hold on;
Y_1 = lsim(P,out.tita,t);
Y_2 = lsim(P_d,out.tita,t);
plot(t,Y_1)
plot(t,Y_2)
legend

figure;
step(P);