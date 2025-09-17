alfa = 0;
beta = 0;
gamma = 0;

A = [alfa;beta;gamma];
Y = out.Y(3:end);
X = [out.Y(2:(end-1)), out.Y(1:(end-2)), out.U(3:(end))];

A = inv(transpose(X)*X)*transpose(X)*Y

p1 = (A(1)+sqrt(A(1)^2+4*A(2)))/2

p2 = (A(1)-sqrt(A(1)^2+4*A(2)))/2

p_cont1 = 50*log(p1)
p_cont2 = 50*log(p2)

P = zpk([],[p1 p2],A(3),1/50);
%%
P_cont = zpk([],[p_cont1 p_cont2],.393);

step(P_cont);


