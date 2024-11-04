import numpy as np
from matplotlib import pyplot as plt

# Calcula distância Euclidiana
def dist(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

# Calcula probabilidade de ocupação
def p_occ(mapa, pos):
    r, occ = 3, 0
    for a in range(-r, r + 1):
        for b in range(-r, r + 1):
            viz = (pos[0] + a, pos[1] + b)
            if 0 <= viz[0] < mapa.shape[0] and 0 <= viz[1] < mapa.shape[1]:
                occ += 1 - mapa[viz]
    return occ / ((2 * r + 1) ** 2)

def A_star(mapa, ini, obj):
    fila, custo, cam = [(0, ini)], {ini: 0}, {ini: None}
    moves = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    
    while fila:
        _, atual = min(fila, key=lambda x: x[0])
        fila.remove((_, atual))
        if atual == obj:
            break
        
        for d in moves:
            viz = (atual[0] + d[0], atual[1] + d[1])
            if 0 <= viz[0] < mapa.shape[0] and 0 <= viz[1] < mapa.shape[1] and mapa[viz] == 1:
                n_custo = custo[atual] + dist(atual, viz)
                if viz not in custo or n_custo < custo[viz]:
                    custo[viz] = n_custo
                    pri = n_custo * p_occ(mapa, viz) + dist(viz, obj)
                    fila.append((pri, viz))
                    cam[viz] = atual
    
    caminho, atual = [], obj
    while atual:
        caminho.append(atual)
        atual = cam.get(atual)
    
    return caminho[::-1]

# Carrega mapa e configura
with open('my_map.pgm', 'rb') as f:
    mapa = plt.imread(f)
mapa = (mapa > 250).astype(float)

# Ponto inicial e objetivo
obj, ini = (150, 250), (350, 100)

# Executa e exibe o caminho
caminho = A_star(mapa, ini, obj)
for p in caminho:
    mapa[p] = 0.5

plt.imshow(mapa, cmap='gray')
plt.title("Caminho Encontrado:")
plt.show()
