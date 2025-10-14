alfa = 0;
beta = 0;
gamma = 0;

T_s = 0.02;

A = [alfa;beta;gamma];
Y = out.Y(3:end);
X = [out.tita(3:end), out.Y(2:(end-1)),out.Y(1:(end-2))];


A = inv(transpose(X)*X)*transpose(X)*Y;

polo = (1/T_s)*log(A(3));

ganancia = A(1)*(1+polo)/((exp(T_s)-1)*(exp(T_s)+A(3)));

P = zpk([],[0 , polo], ganancia)
%%
t = 1/50*(0:6716-1);
figure;
plot(t, out.Y);
hold on;
lsim(P,out.tita,t);