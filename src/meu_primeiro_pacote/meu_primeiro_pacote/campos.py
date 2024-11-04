import numpy as np                   # Para criar arrays
from matplotlib import pyplot as plt # Para leitura do mapa
import random                        # Para contorno
import math                          # Para operações

# Constantes para os campos
katt, krep, raio, delta = 1.0, 500.0, 100.0, 1e-3  # Fatores de escala e parâmetros

# Função para calcular o campo atrativo em direção ao objetivo
def busca_do_objetivo(q, qgoal):
    return 0.5 * katt * np.linalg.norm(np.array(q) - np.array(qgoal)) ** 2

# Função para calcular o campo repulsivo a partir dos obstáculos
def desvio_dos_obstaculos(q, obstacles):
    Urep = 0
    q_array = np.array(q)  # Converte q para array NumPy
    for obstacle in obstacles:
        obstacle_array = np.array(obstacle)  # Converte o obstáculo para array NumPy
        distancia = np.linalg.norm(q_array - obstacle_array)
        if distancia <= raio:
            Urep += 0.5 * krep * ((1 / distancia) - (1 / raio)) ** 2
    return Urep

# Função para calcular o valor total de potencial U em uma posição
def calcular_potencial(q, goal, obstacles):
    return busca_do_objetivo(q, goal) + desvio_dos_obstaculos(q, obstacles)

# Função para encontrar o caminho mais baixo de potencial (gradiente)
def campos_potenciais(start, goal, obstacles, max_steps=1000):
    path = [start]
    current_position = np.array(start)

    for _ in range(max_steps):
        # Calcula os vizinhos
        movimentos = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, 1), (1, -1), (1, 1), (-1, -1)]
        neighbors = [(current_position[0] + dx, current_position[1] + dy) for dx, dy in movimentos
                     if 0 <= current_position[0] + dx < image_copia.shape[0] and 0 <= current_position[1] + dy < image_copia.shape[1]]

        # Avalia o valor de potencial para cada vizinho
        potenciais = [calcular_potencial(neighbor, goal, obstacles) for neighbor in neighbors]
        next_position = neighbors[np.argmin(potenciais)]

        if potenciais[np.argmin(potenciais)] < delta and next_position != goal:
            next_position = (current_position[0], current_position[1] + random.choice([-1, 1]))  # Movimento aleatório

        path.append(tuple(next_position))
        current_position = np.array(next_position)

        # Verifica se o objetivo foi alcançado
        if np.linalg.norm(current_position - goal) < 1.0:
            break

    return path

pgmf = open('my_map.pgm', 'rb')
image = plt.imread(pgmf)
pgmf.close()
image_copia = 1.0 * (image > 250)

image_copia = 1.0 * (image > 250)  # Converte a matriz para binário

start, goal = (350, 100), (90, 340)  # Posição inicial e do objetivo

# Extrai obstáculos
obstacles = [(i, j) for i in range(image_copia.shape[0]) for j in range(image_copia.shape[1]) if image_copia[i, j] == 0]

# Executa o algoritmo e exibe o caminho encontrado no mapa
path = campos_potenciais(start, goal, obstacles)
for cell in path:
    image_copia[int(cell[0]), int(cell[1])] = 0.5

plt.imshow(image_copia, interpolation='nearest', cmap='hot')
plt.title("Campos Potenciais")
plt.show()
