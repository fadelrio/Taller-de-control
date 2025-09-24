close all; 


s = tf('s');


optionss=bodeoptions;
%optionss.MagVisible='off';
optionss.PhaseMatching='on';
optionss.PhaseMatchingValue=-170;
optionss.PhaseMatchingFreq=.1;
optionss.Grid='on';


p1 = -8.0022 + 2.9579i;
p2 = -8.0022 - 2.9579i;

k = 28;

P = zpk([],[p1 p2],k);

figure();
bode(P, optionss, {0.01,1000});
set(findall(gcf,'type','line'),'linewidth',2);
legend

%%
close all;


T_s = 0.020; 

C = db2mag(8.66)*zpk([-6],[0],1);

L = minreal(P*C);

S = 1/(1+L);

T = 1-S;

PS = P*S;

CS = C*S;

figure();
optionss.PhaseMatchingValue=-180;
optionss.PhaseMatchingFreq=20;
bode( L, optionss, {0.01,1000});
set(findall(gcf,'type','line'),'linewidth',2);
legend

C_back = c2d(C,T_s,'zoh');

C_bil = c2d(C,T_s,'tustin');




%%
close all;
figure();
step(S);
legend

figure();
step(T);
legend

figure();
step(PS);
legend

figure();
step(CS);
legend

