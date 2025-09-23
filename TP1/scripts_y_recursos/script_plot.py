import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# === Parámetros ===
fs = 50  # Frecuencia de muestreo en Hz
archivo1 = "servo_u_1.csv"
archivo2 = "servo_y_1.csv"

# === Cargar datos ===
# Se asume que los archivos tienen una sola columna de valores sin encabezado
data1 = pd.read_csv(archivo1, header=None).squeeze("columns").to_numpy()
data2 = pd.read_csv(archivo2, header=None).squeeze("columns").to_numpy()

data1_recorte = data1[202:252]
data2_recorte = data2[202:252]
# === Crear vector de tiempo ===
N = min(len(data1_recorte), len(data2_recorte))  # Usar la cantidad mínima de muestras
t = np.arange(N) / fs

# === Graficar ===
plt.figure(figsize=(10,5))
plt.plot(t, data1_recorte, label="Referencia servo")
plt.plot(t, data2_recorte, label="Angulo medido IMU")
plt.xlabel("Tiempo [s]")
plt.ylabel("Angulo [°]")
plt.title("Tiempo de respuesta del servomotor")
plt.legend()
plt.grid(True)
plt.show()
