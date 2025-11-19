close all;

largo = 600;

diferencia_simumreal = 0;


inicio_simu = 1;

fin_simu = inicio_simu + largo + diferencia_simumreal;


inicio_real = 164;

fin_real = inicio_real + largo;

T_s = 1/50;


t = T_s*(1:(largo+1));

t_2 = T_s*(1:(largo+1+diferencia_simumreal));


Simulado = simu_carrito(inicio_simu:fin_simu);
Real = carrito(inicio_real:fin_real);

figure;
plot(t_2, Simulado, 'LineWidth', 1.5)
hold on;
plot(t, Real, 'LineWidth', 1.5)
legend("Simulado", "Observado")
grid
hold on;

