import cv2
import numpy as np
from matplotlib import pyplot as plt
import math
from math import *
from matplotlib.colors import Normalize

pgmf = open('src/meu_primeiro_pacote/meu_primeiro_pacote/my_map.pgm', 'rb')
matrix = plt.imread(pgmf)

matrixcop = 1.0 * (matrix > 250)

final = (35, 373) 
inicio = (359, 36) 

matrixcop[final[0]][final[1]] = 0 
matrixcop[inicio[0]][inicio[1]] = 0   

def menor_valor(lista_f, lista_c):
    if not lista_f or not lista_c :
        return 
    
    lista_f = np.array(lista_f)
    lista_c = np.array(lista_c)

    lista_ind = np.where(lista_f == min(lista_f))
    lista_ind = lista_ind[0]

    h_min = math.dist(lista_c[lista_ind[0]], final)

    menor_indice = lista_ind[0]

    for i in lista_ind:
        h = math.dist(lista_c[i], final)
        if (h < h_min):
            h_min = h
            menor_indice = i

    return menor_indice

matrixcop[final[0]][final[1]] = 2 
matrixcop[inicio[0]][inicio[1]] = 1 

menor_h = 1000
ponto = inicio
parar = False
coordenadas = list()
caminho_f = list ()

while(1):
    for l in range (-1,2):
        for c in range (-1,2):
            try:
                if(matrixcop[ponto[0]+l][ponto[1]+c] == 1):
                    g = math.dist(inicio, (ponto[0]+l,ponto[1]+c))
                    h = math.dist((ponto[0]+l,ponto[1]+c), final)
                    f = g + h

                    matrixcop[ponto[0]+ l][ponto[1]+c]= f
                    caminho_f.append(f)
                    coordenadas.append([ponto[0]+ l, ponto[1]+c])

                if(ponto[0]+l == final[0] and ponto[1]+c == final[1]):
                    parar = True
                    break
            except: 
                continue
        if(parar == True):
            break   
    if(parar == True):
           break   
    
    menor_ind = menor_valor(caminho_f, coordenadas)
    if menor_ind is None:
        print("Erro: não foi encontrado um índice válido para continuar.")
        continue
    else:
        ponto = coordenadas.pop(menor_ind)
        caminho_f.pop(menor_ind)


ponto_inicial = inicio
caminho = [inicio]
menor = matrixcop[inicio[0]][inicio[1]] + 2
menor_posicao = inicio
parar = False
listafechada = list()

while(1):
    for l in range (1,-2,-1):
        for c in range (1,-2,-1):
            try:          
                if(matrixcop[ponto_inicial[0]+l][ponto_inicial[1]+c] > 1 and matrix_copia[ponto_inicial[0]+l][ponto_inicial[1]+c] < menor and ([ponto_inicial[0]+l],[ponto_inicial[1]+c]) not in listafechada and (([ponto_inicial[0]+l],[ponto_inicial[1]+c]) != ([ponto_inicial[0]], [ponto_inicial[1]]))):
                    menor = matrixcop[ponto_inicial[0]+l][ponto_inicial[1]+c]
                    menor_posicao = (ponto_inicial[0]+l, ponto_inicial[1]+c)

                if(ponto_inicial[0]+l == final[0] and ponto_inicial[1]+c == final[1]):
                    parar = True
                    break

                listafechada.append(([ponto_inicial[0]+l],[ponto_inicial[1]+c]))
            except: 
                continue

        if(parar == True):
            break 

    ponto_inicial = menor_posicao
    caminho.append(menor_posicao)
    menor = menor + 2

    if(parar == True):
        break 
    
#colorindo caminho :)

imagem_caminho = matrix.copy()
imagem_caminho = cv2.cvtColor(imagem_caminho, cv2.COLOR_GRAY2RGB)

for i in caminho:

    imagem_caminho[i[0]][i[1]] = [254, 0, 0]

plt.imshow(imagem_caminho)
plt.title('Caminho calculado')
plt.show()