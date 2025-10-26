close all
clear all

s = tf('s');

T_s = 1/50;
p_25 = 0.91;
k_25 = -0.01;
p_cont_25 = 1/T_s*log(p_25);
k_c25 = k_25/T_s^2;
P_carr_barra = zpk([],[0,p_cont_25], k_c25);

P_barra = zpk([],[-4.209 -31.13],57.195);

P = minreal(P_barra*P_carr_barra);


optionss=bodeoptions;
%optionss.MagVisible='off';
optionss.PhaseMatching='on';
optionss.PhaseMatchingValue=-80;
optionss.PhaseMatchingFreq=0.0001;
optionss.Grid='on';

figure
bode(P, optionss);
legend
%%
kp = -1.5;
ki = 0;
kd = -0.0015;

C = zpk((kd*s*s+kp*s+ki)/s);

L = minreal(P*C);

%%
figure
rlocus(1/(1+L))
%%

figure
bode(L, optionss);
legend 

S = 1/(1+L);

rta_pert = P*S;

figure
step(rta_pert);
legend




