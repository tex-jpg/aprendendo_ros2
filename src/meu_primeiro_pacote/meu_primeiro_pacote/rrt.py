import cv2
import numpy as np
import random
import math
from matplotlib import pyplot as plt

# Carregar imagem e definir parâmetros iniciais
arquivo_mapa = open('my_map.pgm', 'rb')
mapa_imagem = plt.imread(arquivo_mapa)
mapa_binario = 1.0 * (mapa_imagem > 250)

objetivo = (80, 325)
posicao_robo = (300, 25)

# Marcar posição do robô e do objetivo na imagem
mapa_binario[objetivo[0]][objetivo[1]] = 0
mapa_binario[posicao_robo[0]][posicao_robo[1]] = 0

# Parâmetros do algoritmo RRT
fator_crescimento = 10
nos_explorados = [posicao_robo]
caminhos_explorados = []
pais_nos, filhos_nos = [], []
max_interacoes = 1000

# Função para encontrar o nó mais próximo
def encontrar_no_mais_proximo(nos_explorados, ponto_aleatorio):
    no_mais_proximo = min(nos_explorados, key=lambda no: math.dist(no, ponto_aleatorio))
    return no_mais_proximo

# Função para gerar um novo nó
def gerar_novo_no(no_mais_proximo, ponto_aleatorio, fator_crescimento):
    vetor_direcao = (ponto_aleatorio[0] - no_mais_proximo[0], ponto_aleatorio[1] - no_mais_proximo[1])
    modulo_vetor = math.sqrt(vetor_direcao[0]**2 + vetor_direcao[1]**2)
    if modulo_vetor < fator_crescimento:
        return ponto_aleatorio
    fator = fator_crescimento / modulo_vetor
    return (int(no_mais_proximo[0] + vetor_direcao[0] * fator), int(no_mais_proximo[1] + vetor_direcao[1] * fator))

# Executar o algoritmo RRT
for _ in range(max_interacoes):
    ponto_aleatorio = (random.randint(0, 350), random.randint(0, 350))
    no_mais_proximo = encontrar_no_mais_proximo(nos_explorados, ponto_aleatorio)
    novo_no = gerar_novo_no(no_mais_proximo, ponto_aleatorio, fator_crescimento)
    if mapa_binario[novo_no[0]][novo_no[1]] == 1.0:
        nos_explorados.append(novo_no)
        caminhos_explorados.append((no_mais_proximo, novo_no))
        pais_nos.append(no_mais_proximo)
        filhos_nos.append(novo_no)
        if math.dist(novo_no, objetivo) < fator_crescimento:
            break

# Converter imagem para RGB
mapa_colorido = cv2.cvtColor(mapa_imagem.copy(), cv2.COLOR_GRAY2RGB)

# Desenhar o caminho explorado com a cor vermelha
for (ponto1, ponto2) in caminhos_explorados:
    cv2.line(mapa_colorido, ponto1[::-1], ponto2[::-1], (0, 0, 255), 1)  # Vermelho

# Desenhar o caminho final com outra cor (verde)
caminho_final, no_filho = [], no_mais_proximo
while no_filho != posicao_robo:
    indice = filhos_nos.index(no_filho)
    no_pai = pais_nos[indice]
    caminho_final.append((no_pai, no_filho))
    no_filho = no_pai

# Colorir o caminho final em verde
for (ponto1, ponto2) in caminho_final:
    cv2.line(mapa_colorido, ponto1[::-1], ponto2[::-1], (0, 255, 0), 2)  # Verde

# Exibir imagem final com o caminho
plt.imshow(mapa_colorido)
plt.title('Caminho do robô até o objetivo')
plt.show()
