close all; clear all; clc;

syms h u y real

Q_i = 8/(1000*60);
d = 0.01065;
l_1 = 0.1;
l_2 = 0.4;
L = 0.9;
h_0 = 0.45;
g = 9.8;

Q_cero = (((d^2)*pi)/4)*u*sqrt(2*g*h);

hdot = (Q_i-Q_cero)/(l_1 + (h/L)*(l_2-l_1))^2;

y = h;

%%


h0 = sym('0.45');
u0 = subs( solve( hdot == 0, u ), h, h0 )

A = double( subs( jacobian(hdot, h), [h u], [h0 u0] ) )
B = double( subs( jacobian(hdot, u), [h u], [h0 u0] ) )
C = 1;
D = 0;

P = tf(ss(A, B, C, D))
%%

close all;

optionss=bodeoptions;
%optionss.MagVisible='off';
optionss.PhaseMatching='on';
optionss.PhaseMatchingValue=0;
optionss.PhaseMatchingFreq=.00001;
optionss.Grid='on';

C = db2mag(18)*zpk([-0.01],[0],-1);

L = minreal(P*C);

bode(L, optionss);
legend

opt = stepDataOptions('StepAmplitude', 0.1);

S = 1/(1+L);
T = 1-S;
CS = minreal(C*S);


figure();
step(T, opt); grid on 

figure();
step(CS, opt); legend; grid on 

C_digit = c2d(C, 1, 'zoh');




