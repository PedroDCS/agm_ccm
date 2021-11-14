'''
Nome Discente: Pedro Daniel Camargos Soares
Matrícula: 0020640
Data: 07/11/2021

Declaro que sou o único autor e responsável por este programa. Todas as partes do programa, exceto as que
foram fornecidas pelo professor ou copiadas do livro ou das bibliotecas de Sedgewick e Wayne, foram 
desenvolvidas por mim. declaro também que sou responsável por todas  as eventuais cópias deste programa e
que não distribui nem facilitei a distribuição de cópias. 
'''

'''
Algoritmo com objetivo de econtrar o caminho de custo mínimo (CCM) em um grafo G (V,A)
Utilizando o algoritmo de Dijkstra com e sem fila de prioridade
'''




from collections import defaultdict
import math
from heapq import *
import time
import sys
class Grafo(object):
    # Implementação básica de um grafo.

    def __init__(self, v):
        # numero de vertices
        self.vertices = v
        # representação por list de adjacencia
        self.adj = defaultdict(list)

    def adiciona_aresta(self, u, v, peso):
        # Adiciona uma aresta entre os vertices 'u' e 'v' e o seu peso
        # lista de adjacencia normal
        self.adj[u].insert(0, [v, peso])
        self.adj[v].insert(0, [u, peso])

    # se caso for printar o grafo, isso o formatara
    def __str__(self):
        return '{}({})'.format(self.__class__.__name__, dict(self.adj))

    # implementa a fila de prioridades usando a estrutura de dados heap
    def dijkstra_heap(self):
        # ira guarda o custo ate determinado vertice
        custo = {}
        # o custo do vertice inicial para ele mesmo é 0
        custo[0] = 0
        # ira guardar a distancia ate os vertices
        distancia_vertice = [0] * int(self.vertices)

        # inicia todos os vertices com custo infinito
        custo = [float('inf')] * int(self.vertices)

        # ira guardar o caminho do vertice inicial ate cada vertice do grafo

        # adiciona o vertice inicial no caminho de todos os vertices, já que todos começam por ele
        caminho = [[0]] * int(self.vertices)

        # inicia a fila de prioridade, co o vertice inicial 0 e seu peso sendo 0
        fila_prioridade = [(0, 0)]

        # enquanto a fila não estiver vazia
        while fila_prioridade:

            (cost, v1) = heappop(fila_prioridade)
            distancia_vertice[v1] += cost
            for vizinho, peso in self.adj[v1]:
                custo_vertice = peso + distancia_vertice[v1]
                if custo[vizinho] == float("inf") or custo[vizinho] > custo_vertice:
                    custo[vizinho] = custo_vertice

                    if not(v1 in caminho[v1]):
                        caminho[v1].append(v1)
                    caminho[vizinho] = caminho[v1] + [vizinho]

                    heappush(fila_prioridade, (custo_vertice, vizinho))

        return caminho, custo

    # sem utilizar fila de prioridades
    def dijkstra(self):
        # ira auxiliar para pegar o menor visinho do vertice que estiver sendo executado
        controle_aux = {}

        # ira guarda o custo ate determinado vertice
        custo = {}

        # ira guardar a distancia ate os vertices
        distancia_vertice = {}

        # guarda os vertices ainda não visitados
        vertices_nao_visitados = []

        # vertice atual, começando pelo vertice inicial 0
        atual = 0

        # a distancia pro vertice inicial pra ele mesmo é 0
        distancia_vertice[atual] = 0

        # ira guardar o caminho do vertice inicial ate cada vertice do grafo
        caminho = []

        # para cada vertice do grafo
        for vertice in self.adj.keys():
            # inclui os vertices nos não visitados
            vertices_nao_visitados.append(vertice)
            # inicia todos os vertices com custo infinito
            custo[vertice] = float('inf')
            # adiciona o vertice inicial no caminho de todos os vertices, já que todos começam por ele
            caminho.append([0])

        # o custo do vertice inicial para ele mesmo é 0
        custo[atual] = 0

        # remove o vertice inicial dos não visitados, já que ele já foi tratado
        vertices_nao_visitados.remove(atual)

        # enquanto ouver vertices não visitados
        while vertices_nao_visitados:

            # para cada aresta conectada ao vertice atual...
            for vizinho, peso in self.adj[atual]:
                # o peso para chegar ate o vertice em questão sera a distancia ate o vertice atual + o peso da aresta
                custo_vertice = peso + distancia_vertice[atual]

                # caso ele ainda não tenha sido visitado ou já tenha sido mas com um custo maior, faça...
                if custo[vizinho] == float("inf") or custo[vizinho] > custo_vertice:
                    # o novo custo ate o vertice vizinho sera o custo calculado
                    custo[vizinho] = custo_vertice
                    # o novo custo ate o vertice vizinho é adicionado ao controle
                    controle_aux[vizinho] = custo[vizinho]
                    # caso o vertice atual ainda não estaja adicionado ao caminho minimo, ele é adicionado
                    if not(atual in caminho[atual]):
                        caminho[atual].append(atual)
                    # atualiza o caminho mais curto ate o vertice vizinho
                    caminho[vizinho] = caminho[atual] + [vizinho]

            # caso o controle esteja vazio, interrompe a execução
            if controle_aux == {}:
                break
            # seleciona o menor vizinho com base no peso para ser o proximo vertive atual
            # seleciona o menor vizinho
            menor_vizinho = min(controle_aux.items(), key=lambda x: x[1])
            atual = menor_vizinho[0]

            # distancia do novo vertice atual é atualizada
            distancia_vertice[atual] = menor_vizinho[1]
            # remove o novo vertice atual dos que ainda não foram vizitados
            vertices_nao_visitados.remove(atual)
            # limpa o controle auxiliar do vertice atual, já que ele já foi visitado
            del controle_aux[atual]

        return caminho, custo


def main(args):
    def arquivosaida(arq, alg, tempo, caminho, custo):
        arquivo = open(arq+".txt", 'w')
        arquivo.write(
            "==================== Caminho Custo Minimo ====================\n")
        arquivo.write(alg+"\n")
        arquivo.write(
            "==============================================================\n")
        arquivo.write("Tempo de execucao do Algoritmo: " + str(tempo)+"\n")

        arquivo.write(
            "==============================================================\n")
        caminho_str = ''
        for vertice in range(custo.__len__()):
            caminho_str = caminho_str + "Caminho Minimo de 0 ate: "+str(vertice)+":\n" + \
                "Custo do Caminho Minimo: "+str(custo[vertice])+"\n" + \
                "Caminho Minimo: \n" +str(caminho[vertice])+"\n\n"
        arquivo.write(caminho_str)
 

        arquivo.write(
            "==============================================================\n")


        arquivo.close()

    def lerArq(arq):
        arquivo = open(str(arq), 'r')
        vertices = arquivo.readline()
        arestas = arquivo.readline()
        linhas = arquivo.readlines()
        arquivo.close()
        return vertices, arestas, linhas

    if args.__len__() < 4:
        print("falta de parametros")
        exit()
    
    algoritmo = int(args[1])
    entrada = args[2]
    saida = args[3]

    vertices, arestas, linhas = lerArq(entrada)

    graph = Grafo(vertices)
    for linha in linhas:
        aux = linha.split()
        graph.adiciona_aresta(int(aux[0]), int(aux[1]), float(aux[2]))

    
    if algoritmo == 1:
        algoritmo = "algoritmo de Dijkstra sem utilizar fila de prioridades"
        print(algoritmo)
        inicio = time.time()
        caminho, custo = graph.dijkstra()
        fim = time.time()
        arquivosaida(saida, algoritmo, fim - inicio, caminho, custo)
    elif algoritmo == 2:
        algoritmo = "algoritmo de Dijkstra com a fila de prioridades"
        print(algoritmo)
        inicio = time.time()
        caminho, custo = graph.dijkstra_heap()
        fim = time.time()
        arquivosaida(saida, algoritmo, fim - inicio, caminho, custo)
    else:
        print("Digite uma entrada de algoritmo valida")
   
    print("--------------------------------------------------")


if __name__ == "__main__":
    print("--------------------------------------------------")
    main(sys.argv)
    '''
    def lerArq(arq):
        arquivo = open(str(arq), 'r')
        vertices = arquivo.readline()
        arestas = arquivo.readline()
        linhas = arquivo.readlines()
        arquivo.close()
        return vertices, arestas, linhas
    grafopequeno = lerArq("ccm/testes/grafo-grande.txt")
    
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/ccm/fontes/main.py",
            1, grafopequeno, 'saidasccmgrande/djgrafo-pequeno']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/ccm/fontes/main.py",
            2, grafopequeno, 'saidasccmgrande/dj_heapgrafo-pequeno']
    main_testes(args)

    def lerArq(arq):
        arquivo = open(str(arq), 'r')
        vertices = arquivo.readline()
        arestas = arquivo.readline()
        linhas = arquivo.readlines()
        arquivo.close()
        return vertices, arestas, linhas
    grafomedio1 = lerArq("ccm/testes/grafo-medio1.txt")
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/ccm/fontes/main.py",
            1, grafomedio1, 'saidasccmgrande/djgrafo-medio1']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/ccm/fontes/main.py",
            2, grafomedio1, 'saidasccmgrande/dj_heapgrafo-medio1']
    main_testes(args)

    def lerArq(arq):
        arquivo = open(str(arq), 'r')
        vertices = arquivo.readline()
        arestas = arquivo.readline()
        linhas = arquivo.readlines()
        arquivo.close()
        return vertices, arestas, linhas
    grafomedio2 = lerArq("ccm/testes/grafo-medio2.txt")
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/ccm/fontes/main.py",
            1, grafomedio2, 'saidasccmgrande/djgrafo-medio2']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/ccm/fontes/main.py",
            2, grafomedio2, 'saidasccmgrande/dj_heapgrafo-medio2']
    main_testes(args)

    def lerArq(arq):
        arquivo = open(str(arq), 'r')
        vertices = arquivo.readline()
        arestas = arquivo.readline()
        linhas = arquivo.readlines()
        arquivo.close()
        return vertices, arestas, linhas
    grafomedio3 = lerArq("ccm/testes/grafo-medio3.txt")

    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/ccm/fontes/main.py",
            1, grafomedio3, 'saidasccmgrande/djgrafo-medio3']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/ccm/fontes/main.py",
            2, grafomedio3, 'saidasccmgrande/dj_heapgrafo-medio3']
    main_testes(args)

    '''