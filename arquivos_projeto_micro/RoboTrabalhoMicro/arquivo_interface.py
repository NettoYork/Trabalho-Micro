import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
import serial
import time

filename2 = 'teste_dados2.txt'
n_valores = 10

SERIAL_PORT = 'COM10'  # Substitua pelo nome da porta serial do seu dispositivo
BAUD_RATE = 115200  # Certifique-se de que corresponde à taxa de transmissão do seu ESP32

def read_serial_data(port, baud_rate, output_file):
    with serial.Serial(port, baud_rate, timeout=0.1) as ser:
        while True:
            line = ser.readline().decode('utf-8').strip()
            if line:
                print(f"Dados recebidos: {line}")
                
                with open(output_file, 'r') as file:
                    linhas = file.readlines()
                
                linhas.append(f"{line}\n")
                
                if len(linhas) > n_valores:
                    linhas = linhas[-n_valores:]
                
                with open(output_file, 'w') as file:
                    file.writelines(linhas)

            time.sleep(0.1)

def distancia(frame):
    global valores_distancia

    try:
        with open(filename2, 'r') as file_g2:
            linhas = file_g2.readlines()
            valores_distancia = [float(linha.split()[0]) for linha in linhas if len(linha.split()) >= 2]
        
        if len(valores_distancia) > 0:
            ax.cla()
            ax.plot(theta[:len(valores_distancia)], valores_distancia, color='red')  # Gráfico vermelho
            ax.set_ylim(0, max(valores_distancia) + 5)
            ax.set_title('Distância captada pelo ultrassom', color='white')  # Título branco
    except Exception as e:
        print(f"Erro na função `distancia`: {e}")

def encoders(frame):
    global valores_encoders

    try:
        with open(filename2, 'r') as file_g:
            linhas = file_g.readlines()
            valores_encoders = [int(linha.split()[1]) - int(linha.split()[2]) for linha in linhas if len(linha.split()) >= 3]
        
        if len(valores_encoders) > 0:
            ax2.cla()
            ax2.plot(theta2[:len(valores_encoders)], valores_encoders, color='blue')
            ax2.set_ylim(min(valores_encoders) - 5, max(valores_encoders) + 5)
            ax2.set_title('Diferença do valor dos encoders em relação ao tempo', color='white')  # Título branco
        else:
            print("Nenhum dado de encoder disponível para plotar.")
    except Exception as e:
        print(f"Erro na função `encoders`: {e}")

root = tk.Tk()
root.title("Interface com o usuário")
root.configure(bg='black')  # Fundo da interface preto

def sair():
    root.quit()
    root.destroy()

fig, ax = plt.subplots(subplot_kw={'projection': 'rectilinear'}, facecolor='black')  # Fundo do gráfico preto
theta = np.linspace(0, np.pi, n_valores)
ax.tick_params(colors='white')  # Cores dos ticks
ax.spines['bottom'].set_color('white')  # Cor da borda inferior
ax.spines['top'].set_color('white')  # Cor da borda superior
ax.spines['right'].set_color('white')  # Cor da borda direita
ax.spines['left'].set_color('white')  # Cor da borda esquerda
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

fig2, ax2 = plt.subplots(subplot_kw={'projection': 'rectilinear'}, facecolor='black')  # Fundo do gráfico preto
theta2 = np.linspace(0, 200, n_valores)
ax2.tick_params(colors='white')  # Cores dos ticks
ax2.spines['bottom'].set_color('white')
ax2.spines['top'].set_color('white')
ax2.spines['right'].set_color('white')
ax2.spines['left'].set_color('white')
canvas2 = FigureCanvasTkAgg(fig2, master=root)
canvas2.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

btn_sair = tk.Button(root, text="Sair", command=sair, font=("Helvetica", 12), bg='black', fg='white')
btn_sair.pack(pady=10)

ani = FuncAnimation(fig, distancia, interval=500, cache_frame_data=False)
ani2 = FuncAnimation(fig2, encoders, interval=500, cache_frame_data=False)

import threading
thread = threading.Thread(target=read_serial_data, args=(SERIAL_PORT, BAUD_RATE, filename2))
thread.daemon = True
thread.start()

root.mainloop()










