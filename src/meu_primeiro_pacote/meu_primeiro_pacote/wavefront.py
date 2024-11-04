import cv2
import numpy as np
from matplotlib import pyplot as plt

# Função para aplicar o algoritmo Wavefront
def wavefront(matrix, goal):
    rows, cols = matrix.shape
    wavefront_field = np.full((rows, cols), np.inf)  # Inicializa com infinito
    wavefront_field[goal] = 2  # Objetivo marcado com valor 2
    process_list = [goal]  # Lista de células a processar

    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]  # Movimentos permitidos

    while process_list:
        current = process_list.pop(0)
        current_value = wavefront_field[current]

        for direction in directions:
            neighbor = (current[0] + direction[0], current[1] + direction[1])

            # Verifica limites e se é um espaço livre
            if (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and matrix[neighbor] == 1):
                if wavefront_field[neighbor] > current_value + 1:
                    wavefront_field[neighbor] = current_value + 1
                    process_list.append(neighbor)

    return wavefront_field

# Carrega o mapa e converte para binário (0 = obstáculo, 1 = livre)
pgmf = open('my_map.pgm', 'rb')
image = plt.imread(pgmf)
pgmf.close()
image_copia = 1.0 * (image > 250)

goal = (250, 125)  # Define o objetivo
robo = (350, 50)   # Define a posição inicial do robô

# Executa o algoritmo Wavefront para criar o campo de proximidade
wavefront_field = wavefront(image_copia, goal)

# Exibição do campo de proximidade
plt.figure("Figura 2")
plt.imshow(wavefront_field, cmap='viridis', interpolation='nearest')
plt.colorbar(label="Distância do Objetivo")
plt.title("Campo de Proximidade")
plt.show()

# Busca do menor caminho até o objetivo
ponto_inicial = robo
caminho = [list(robo)]
menor = wavefront_field[robo[0]][robo[1]] + 1
menor_posicao = ponto_inicial  # Inicializa menor_posicao com o ponto inicial do robô
parar = False

while True:
    for l in range(1, -2, -1):
        for c in range(1, -2, -1):
            try:
                valor = wavefront_field[ponto_inicial[0] + l][ponto_inicial[1] + c]
                if 1 < valor < menor:
                    menor = valor
                    menor_posicao = (ponto_inicial[0] + l, ponto_inicial[1] + c)

                if menor_posicao == goal:
                    parar = True
                    break
            except IndexError:
                continue
        if parar:
            break

    ponto_inicial = menor_posicao
    caminho.append(menor_posicao)

    if parar:
        break

# Colorindo o caminho em vermelho
image_com_caminho = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)

for pos in caminho:
    image_com_caminho[pos[0], pos[1]] = [0, 0, 255]  # Pinta em vermelho

# Exibe a imagem final com o caminho
cv2.imshow('Caminho Encontrado', image_com_caminho)
cv2.waitKey(0)
cv2.destroyAllWindows()
