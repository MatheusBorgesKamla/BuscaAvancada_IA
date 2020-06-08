# # Realizando leitura do txt e criando grafo 
# Primeiramente, temos que fazer a leitura do txt, identificando as dimensões do tabuleiro na primeira linha e posteriormente lendo cada asterístico como 
#um espaço em branco no tabuleiro (posiçoes que pode andar), traço representando obstáculos, jogo da velha a posição inicial e cifrão a posição objetivo. 
#Além disso, precisamos criar um grafo guardando as posições permitidas como vértices. Com o intuito de não consumir tanta memória (matriz muito esparsa) iremos 
#utilizar a representação de **lista de adjacência**, em que cada vértice deve ter uma lista com seus vizinhos/adjacentes

#Abrindo arquivo para leitura -> passar o caminho correto para o txt (aqui deixei na mesma pasta)
import sys
f = open(sys.argv[1],'r')

#Lendo primeira linha para obter dimensoes
line = f.readline()
#Separando as strings de dimensão
dimensions = line.split()
#Transformando em inteiros
rows = int(dimensions[0])
cols = int(dimensions[1])
#print("Numero de linhas: ", rows, "\nNumero de colunas: ", cols)

#Lendo o resto de linhas
maze = f.readlines()

#Iremos transformar a matriz de caracteres em uma imagem do labarinto para uma melhor visualização
import numpy as np
from matplotlib import pyplot as plt

#Criando imagem de mesma dimensão do labirinto
img_maze = np.zeros((rows, cols,3), np.uint8)

#Pintando os pixels de posição respectiva a matriz de caracteres
for i in range(0, rows):
    for j in range(0, cols):
        if(maze[i][j] == '-'): #Obstáculos em preto
            img_maze[i][j] = [0,0,0]
        elif(maze[i][j] == '*'): #Posições permitidas em branco
            img_maze[i][j] = [250,240,250]
        elif(maze[i][j] == '#'): #Posição inicial em azul
            img_maze[i][j] = [0,0,255]
        elif(maze[i][j] == '$'): #Posição final em vermelho
            img_maze[i][j] = [255,0,0]
#Plotando a imagem
f, ax = plt.subplots(figsize=(20, 10))
ax.imshow(img_maze)
ax.set_title('Labirinto', fontsize=18)
#ax.axis('off')

#Dicionário que guardará o grafo
graph = {}
#Guardará a posicao inicial (#)
ini_position = 0
#Guardará a saída do grafo ($)
fin_position = 0
#Vamos percorrer a matriz de caracteres
for i in range(0, rows):
    for j in range(0, cols):
        #Se é - nem considero como vertice (nao pode passar)
        if(maze[i][j] != '-'):
            
            #Lista que guarda os vertices adjacentes(que possui conexao) ao em analise atual
            adj_list = []
            
            #Condicao para guardar a posicao inicial
            if(maze[i][j] =='#'):
                ini_position = (i,j)
                
            #Condicao para guardar a posicao de saida do labirinto
            elif(maze[i][j] == '$'):
                fin_position = (i,j)
            
            #Checando as posicoes adjacentes ao vertice atual
            for aux in [(i,j+1),(i+1,j),(i,j-1),(i-1,j)]: #A forma como ordena influencia bastante os algoritmos !!
                #Checo se e uma posicao valida (se existe a posicao)
                if((aux[0] >= 0) and (aux[0] < rows) and (aux[1] >= 0) and (aux[1] < cols)):
                    #Se essa posicao adjacente tiver conexao 
                    if(maze[aux[0]][aux[1]] != '-'):
                        adj_list.append((aux[0],aux[1]))
            graph[(i,j)] = adj_list

# # Busca em profundidade 
# Iremos implementar agora um algoritmo que realiza uma busca em profundidade no grafo **(Depth_First_Search)**, guardando os vétices visitados com sua respectiva 
#profundidade, uma lista com os vértices indicando o caminho da posição inicial a saída do labirinto na ordem de busca do DFS e depois a imagem com o caminho indicado

def DFS(graph, vertix, visited, cont, target, path):
    #Guarda o vértice visitado com a sua profundidade
    visited[vertix] = cont
    
    #Insere vertice na lista que guarda o caminho
    path.append(vertix)
    
    #Se o vertice final for encontrado para e retorna verdadeiro
    if(vertix == target):
        return True
    
    #Para cada vizinho do vertice atual 
    for neighboor in graph[vertix]:
        #Checa se ele já não foi visitado
        if neighboor not in visited:
            #Se nao, visita ele e retorna verdadeiro se o seu vizinho pode ser visitado
            if DFS(graph, neighboor, visited, cont+1, target, path):
                return True
    path.pop()
    return False
    

def Depth_First_Search(graph, v_ini, v_fin):
    #Dicionário com os vértices visitados e a profundidade de cada
    visited = {}
    #Deque que guardará o caminho exato entre o vertice inicial e final
    path = []
    #Contador inicializado com 1
    cont = 1
    #Chama a funcao que realmente vai realizar a busca em profundidade
    test = DFS(graph, v_ini, visited, cont, v_fin, path)
    if not test:
        print("Nao existe um caminho valido entre a posicao inicial e a saida")
    return visited, path

#Aplicando o DFS no grafo gerado, tomando tempo e printando informações
import timeit
start_time = timeit.default_timer()
DFS_visited, DFS_path = Depth_First_Search(graph, ini_position, fin_position)
fim = timeit.default_timer() - start_time

print("\n----- Busca em profundidade: -----\nTotal de vertices visitados: ",len(DFS_visited))

print("\nCaminho gerado:\n",DFS_path)

print("\nTamanho do caminho: ", len(DFS_path))

print("\nTempo de execução: ", fim)

#Gerando imagem com o caminho gerado pelo DFS
img_DFS = np.copy(img_maze)
for v in DFS_path:
    img_DFS[v[0]][v[1]] = [0,255,0]

img_DFS[ini_position[0]][ini_position[1]] = [0,0,255]
img_DFS[fin_position[0]][fin_position[1]] = [255,0,0]
    
f1, ax1 = plt.subplots(figsize=(20, 10))
ax1.imshow(img_DFS)
ax1.set_title('Labirinto - Busca em Profundidade', fontsize=18)
#ax1.axis('off')


# # Busca em largura
# Iremos implementar agora um algoritmo que realiza uma busca em largura no grafo **(Breadth First Search)**, guardando os vétices visitados com sua respectiva 
#profundidade informada, uma lista com os vértices indicicando o caminho da posição inicial a saída do labirinto na ordem de busca do BFS e depois a imagem com 
#o caminho indicado


def Breadth_First_Search(graph, v_ini, v_fin):
    #Fila que guardará lista de caminhos com vértices a serem visitados
    queue = [[v_ini]]
    #Dicionário que guardará vértices visitados com sua profundidade
    visited = {}
    #Enquanto existir vé
    while queue:
        #Responsável por pegar a profundidade do vértice que será analisado
        depth = len(queue[0])
        # Pega o primeiro caminho na fila, removendo-o de tal
        path = queue.pop(0)

        # Pega o último vértice do caminho (vértice que será analisado)
        vertex = path[-1]
        
        # Checa se tal vértice não é o vértice final (saída do labirinto)
        if vertex == v_fin:
            #Retorna o caminho e os visitados com profundidade
            return path, visited
        #Checa se tal vértice já não foi visitado para não visitar novamente
        elif vertex not in visited:
            # Para cada vizinho do vértice atual em analise
            for neighbour in graph[vertex]:
                #Cria um novo caminho tomando o anterior e adicionando o vizinho no final
                new_path = list(path)
                #Enfilero o caminho
                new_path.append(neighbour)
                
                queue.append(new_path)

            # Marco o vértice analisado como visitado
            visited[vertex] = depth

#Aplicando o DFS no grafo gerado, tomando tempo e printando informações
start_time = timeit.default_timer()
BreadthFS_path, BreadthFD_visited = Breadth_First_Search(graph, ini_position, fin_position)
fim = timeit.default_timer() - start_time

print("\n----- Busca em largura: -----")

print("\nTotal de vertices visitados: ",len(BreadthFD_visited))

print("\nCaminho gerado:\n",BreadthFS_path)

print("\nTamanho do caminho: ", len(BreadthFS_path))

print("\nTempo de execução: ", fim)


# In[14]:


#Gerando imagem com o caminho da busca em largura
img_BreadthFS = np.copy(img_maze)
for v in BreadthFS_path:
    img_BreadthFS[v[0]][v[1]] = [0,255,0]

img_BreadthFS[ini_position[0]][ini_position[1]] = [0,0,255]
img_BreadthFS[fin_position[0]][fin_position[1]] = [255,0,0]
    
f2, ax2 = plt.subplots(figsize=(20, 10))
ax2.imshow(img_BreadthFS)
ax2.set_title('Labirinto - Busca em Largura', fontsize=18)
#ax1.axis('off')


# # Gerando grafos com peso 
# Para rodar os algoritmos de busca informada precisamos primeiramente gerar um grafo com pesos. Dessa forma iremos criar uma função para calcular os 
#pesos se baseando na distância em módulo de cada vértice ao vértice final (saída do labirinto)

#Função que calcula a distância em módulo de um ponto a outro
def distance_calc(target, pos):
    dist_x = abs(target[0] - pos[0])
    dist_y = abs(target[1] - pos[1])
    mod = ((dist_x**2) + (dist_y**2))**(1/2)
    return mod

#Classe utilizada para montar grafo com pesos (guardará os verticés adjacentes e o peso desse respectivo ao vértice final)
class vertex_weight:
    def __init__(self, vertex, weigth):
        self.vertex = vertex
        self.weigth = weigth
    
    def __str__(self):
        s = "adj: (" + str(self.vertex[0]) + "," + str(self.vertex[1]) + ") w: " + str(self.weigth)
        return s

#Dicionário que guardará o grafo com os pesos para o Best-First
weighted_graph_best = {}

#Para cada vértice do grafo
for source_vertex in graph:
    #Lista auxiliar para armazenar os vértices vizinhos com os pesos da aresta
    aux = []
    #Para cada vértice adjacente do atual
    for dst_vertex in graph[source_vertex]:
        #Calcula a distância do adjacente ao vértice final para ser o peso da ligação
        dist = distance_calc(fin_position, dst_vertex)
        #Adiciona na lista auxiliar
        aux.append(vertex_weight(dst_vertex,dist))
    #Adiciona a lista de vértices adjacentes no dicionário com o respectivo vértice
    weighted_graph_best[source_vertex] = aux

# # Busca Best-First Search
# Iremos implementar agora um algoritmo que realiza uma busca informada, a **Best-First Search** que utiliza dos pesos para determinar o melhor caminho a se 
#seguir, sendo semelhante a busca em largura, porém agora manteremos a fila utilizada ordenada do menor peso para o maior com o intuito de explorar os vizinhos 
#que possui uma ligação de peso menor


#Função que irá partir do vértice final encontrando seus antecessores e assim formando o caminho e deixando ordenado
def backtrace(parent, start, end):
    path = [end]
    cont = 0
    while path[-1] != start:
        path.append(parent[path[-1]])
        cont = cont + 1
    path.reverse()
    return path

def Best_First_Search(graph, v_ini, v_fin):
    #Fila que guardará os vértices a serem explorados (explorar os vizinhos) de forma ordenada (menor pesos no início da fila)
    priority_queue = []
    #Guarda os vértices visitados com respectiva profundidade
    visited = {}
    #Contador de profundidade para os vértices visitados
    cont = 0
    #Insiro o primeiro vértice como visitado já
    visited[v_ini] = cont
    #Insiro o primeiro vértice na fila   
    priority_queue.append(vertex_weight(v_ini, distance_calc(v_fin,v_ini)))
    #Dicionário que guarda um vértice com seu antecessor na ordem de exploração
    parent = {}
    #Enquanto existe vértices a na fila
    while priority_queue:
        #Pego o vértice no início da fila removendo-o de lá
        aux = priority_queue.pop(0)
        vertex = aux.vertex
        #Se ele for o vértice final
        if vertex == v_fin:
            #Retorno os visitados e faço a construção do caminho com a backtrace
            return backtrace(parent, v_ini, v_fin), visited
        #Se não
        else:
            #Incremento o contador de profundidade
            cont = cont + 1
            #Para cada vizinho do vértice atual
            for neighbor in graph[vertex]:
                #Se o vizinho já não foi visitado
                if neighbor.vertex not in visited:
                    #Adiciona como visitado e sua profundidade
                    visited[neighbor.vertex] = cont
                    #Adiciono o vizinho na filaprint("\nTempo de execução: ", fim) para depois explorá-lo
                    priority_queue.append(neighbor)
                    #Ordeno a fila de acordo com o peso da ligação
                    priority_queue.sort(key=lambda x: x.weigth)
                    #Adiciono o vizinho com seu "pai" que é o vértice atual em análise
                    parent[neighbor.vertex] = vertex # In[20]:


#Aplicando o Best First Search no grafo com pesos e printando as informações
start_time = timeit.default_timer()
BestFS_path, BestFS_visited = Best_First_Search(weighted_graph_best, ini_position, fin_position)
fim = timeit.default_timer() - start_time

print("\n----- Best First Search: -----")

print("\nTotal de vertices visitados: ",len(BestFS_visited))

print("\nCaminho gerado:\n",BestFS_path)

print("\nTamanho do caminho: ", len(BestFS_path))

print("\nTempo de execução: ", fim)


#Classe utilizada para montar grafo com pesos (guardará os verticés adjacentes e o peso desse respectivo ao vértice final)
class vertex_weight_star:
    def __init__(self, vertex, weigth_h, weigth_g):
        self.vertex = vertex
        self.weigth_h = weigth_h
        self.weigth_g = weigth_g
        self.weigth_f = weigth_h + weigth_g
    
    def __str__(self):
        s = "adj: (" + str(self.vertex[0]) + "," + str(self.vertex[1]) + ") h: " + str(self.weigth_h) + " g: " + str(self.weigth_g)+ " f: " + str(self.weigth_f)
        return s


# In[10]:


#Dicionário que guardará o grafo com os pesos
double_weighted_graph = {}

#Para cada vértice do grafo
for source_vertex in graph:
    #Lista auxiliar para armazenar os vértices vizinhos com os pesos da aresta
    aux = []
    #Para cada vértice adjacente do atual
    for dst_vertex in graph[source_vertex]:
        #Calcula a distância do adjacente ao vértice final para ser o peso da ligação
        dist = distance_calc(fin_position, dst_vertex)
        #Adiciona na lista auxiliargraph
        aux.append(vertex_weight_star(dst_vertex,dist,100))
    #Adiciona a lista de vértices adjacentes no dicionário com o respectivo vértice
    double_weighted_graph[source_vertex] = aux


# # Busca A*
# 
# Iremos implementar agora um algoritmo que realiza uma busca informada, a A* que utiliza da distância da posição de origem e da posição de destino para determinar o melhor caminho a se seguir
# 

#Função que irá partir do vértice final encontrando seus antecessores e assim formando o caminho e deixando ordenado
def backtrace(parent, start, end):
    path = [end]
    cont = 0
    while path[-1] != start:
        path.append(parent[path[-1]])
        cont = cont + 1
    path.reverse()
    return path

def A_Star(graph, v_ini, v_fin):
    #Fila que guardará os vértices a serem explorados (explorar os vizinhos) de forma ordenada (menor pesos no início da fila)
    fila_prioridade = []
    #Guarda os vértices visitados com respectiva profundidade
    visited = {}
    #Contador de profundidade para os vértices visitados
    cont = 0
    #Insiro o primeiro vértice como visitado já
    visited[v_ini] = cont
    #Insiro o primeiro vértice na fila   
    fila_prioridade.append(vertex_weight_star(v_ini, distance_calc(v_fin,v_ini),0))
    #Dicionário que guarda um vértice com seu antecessor na ordem de exploração
    parent = {}
    #Enquanto existe vértices a na fila
    while fila_prioridade:
        #Pego o vértice no início da fila removendo-o de lá
        aux = fila_prioridade.pop(0)
        vertex = aux.vertex
        #Se ele for o vértice final
        if vertex == v_fin:
            #Retorno os visitados e faço a construção do caminho com a backtrace
            return backtrace(parent, v_ini, v_fin), visited
        #Se não
        else:
            #Incremento o contador de profundidade
            cont = cont + 1
            #Para cada vizinho do vértice atual
            for neighbor in graph[vertex]:
                #Se o vizinho já não foi visitado
                if neighbor.vertex not in visited:
                    #Adiciona como visitado e sua profundidade
                    visited[neighbor.vertex] = cont
                    #É atribuido o valor heurístico g(n) ao novo nó, somando a distância deste ao seu pai com g(n-1) de seu pai
                    neighbor.weigth_g = distance_calc(neighbor.vertex,vertex) + aux.weigth_g
                    #É atualizado o valor heurístico f(n), sendo f(n) = g(n) + h(n)
                    neighbor.weigth_f = neighbor.weigth_g + neighbor.weigth_h
                    #Adiciono o vizinho na filaprint("\nTempo de execução: ", fim) para depois explorá-lo
                    fila_prioridade.append(neighbor)
                    #Ordeno a fila de acordo com o peso da ligação
                    fila_prioridade.sort(key=lambda x: x.weigth_f)
                    #Adiciono o vizinho com seu "pai" que é o vértice atual em análise
                    parent[neighbor.vertex] = vertex
#Aplicando o A Star Search no grafo com pesos e printando as informações
start_time = timeit.default_timer()
AStar_path, AStar_visited = A_Star(double_weighted_graph, ini_position, fin_position)
fim = timeit.default_timer() - start_time

print("\n----- Busca A*: -----")

print("\nTotal de vertices visitados: ",len(AStar_visited),"\n")

print("\n\nCaminho gerado:\n",AStar_path)

print("\nTamanho do caminho: ", len(AStar_path))

print("\nTempo de execução: ", fim)

#Gerando imagem do labirinto com o caminho
img_AStar = np.copy(img_maze)
for v in AStar_path:
    img_AStar[v[0]][v[1]] = [0,255,0]

img_AStar[ini_position[0]][ini_position[1]] = [0,0,255]
img_AStar[fin_position[0]][fin_position[1]] = [255,0,0]
    
f4, ax4 = plt.subplots(figsize=(20, 10))
ax4.imshow(img_AStar)
ax4.set_title('Labirinto - Busca A*', fontsize=18)

def HC(graph, vertix_w, visited, cont, target, path):
    #Guarda o vértice visitado com a sua profundidade
    visited[vertix_w.vertex] = cont
    
    #Insere vertice na lista que guarda o caminho
    path.append(vertix_w.vertex)
    
    #Se o vertice final for encontrado para e retorna verdadeiro
    if(vertix_w.vertex == target.vertex):
        return True
    
    #Para cada vizinho do vertice atual
    aux = []
    for neighboor in graph[vertix_w.vertex]:
        #Checa se ele já não foi visitado
        if neighboor.vertex not in visited:
            aux.append(neighboor)
            #Se nao, checa se o peso do vizinho e menor do que o atual
            if(neighboor.weigth <= vertix_w.weigth):
                #Se for menor, visita ele
                if HC(graph, neighboor, visited, cont+1, target, path):
                    return True
    if len(aux) != 0:
        if HC(graph, aux.pop(0), visited, cont+1, target, path):
                return True
    path.pop()
    return False
    

def Hill_Climbing(graph, v_ini, v_fin):
    #Dicionário com os vértices visitados e a profundidade de cada
    visited = {}
    #Deque que guardará o caminho exato entre o vertice inicial e final
    path = []
    #Contador inicializado com 1
    cont = 1
    #Transformando vertice final e inicial com seus pesos
    v_ini_w = vertex_weight(v_ini, distance_calc(v_fin,v_ini))
    v_fin_w = vertex_weight(v_fin, distance_calc(v_fin,v_fin))
    #Chama a funcao que realmente vai realizar a busca em profundidade
    test = HC(graph, v_ini_w, visited, cont, v_fin_w, path)
    if not test:
        print("Nao existe um caminho valido entre a posicao inicial e a saida")
    return visited, path

#Iremos reutilizar o código do Best_First para o A*, pois o que mudará mesmo será a heurística
start_time = timeit.default_timer()
HC_visited, HC_path = Hill_Climbing(weighted_graph_best, ini_position, fin_position)
fim = timeit.default_timer() - start_time

print("\n----- Busca Hill Climbing: -----")

print("\nTotal de vertices visitados: ",len(HC_visited),"\n")

print("\n\nCaminho gerado:\n",HC_path)

print("\nTamanho do caminho: ", len(HC_path))

print("\nTempo de execução: ", fim)

#Gerando imagem do labirinto com o caminho
img_HC = np.copy(img_maze)
for v in HC_path:
    img_HC[v[0]][v[1]] = [0,255,0]

img_HC[ini_position[0]][ini_position[1]] = [0,0,255]
img_HC[fin_position[0]][fin_position[1]] = [255,0,0]
    
f5, ax5 = plt.subplots(figsize=(20, 10))
ax5.imshow(img_HC)
ax5.set_title('Labirinto - Busca Hill Climbing', fontsize=18)

#Mostrando as imagens
plt.show()