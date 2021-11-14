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
Algoritmo com objetivo de econtrar o valor de uma árvore geradora de custo mínimo (AGM) em um grafo G (V,A)
Utilizando o algoritmo de kruskal e o algoritmo de Prim, bem como suas variações.
'''




from collections import defaultdict
import math
from heapq import *
import time
import sys
import re
class Grafo(object):
    # Implementação básica de um grafo.

    def __init__(self, v):
        # numero de vertices
        self.vertices = v
        # representação por list de adjacencia
        self.adj = defaultdict(list)
        # uma lista simpres com as arestas, facilitando o algoritmo de Kruskal
        self.lista_arestas = []

    def adiciona_aresta(self, u, v, peso):
        # Adiciona uma aresta entre os vertices 'u' e 'v' e o seu peso
        # lista de adjacencia normal
        self.adj[u].insert(0, [v, peso])
        self.adj[v].insert(0, [u, peso])
        # lista com as arestas, facilitando na ordenação do algoritmo de Kruskal
        self.lista_arestas.append([u, v, peso])

    # se caso for printar o grafo, isso o formatara
    def __str__(self):
        return '{}({})'.format(self.__class__.__name__, dict(self.adj))

    # algoritmo de Kruskal sem utilizar a estrutura union-find ;

    def kruskal_Simples(self):
        # funcao auxiliar para verificar em qual floresta cada arvore se encontra
        def indexFloresta(vertice, arvores):
            
            for index in (range(arvores.__len__())):
                if(vertice in arvores[index]):

                    return index

        # lista que ira guardar os vertices da AGM
        agm = []
        # peso da AGM, começa com peso zerado
        peso_agm = 0
        # cria uma floresta com cada vertice do grafo como uma arvore
        arvores = []
        for node in range(int(self.vertices)):
            arvores.append({node})

        # ordena as arestas do grafo de forma crescente de acordo com o peso de cada aresta
        grafoordenado = sorted(self.lista_arestas, key=lambda item: item[2])

        # para cada aresta ordenada, faça...
        for vertice1, vertice2, peso in grafoordenado:
            index1 = indexFloresta(vertice1, arvores)
            index2 = indexFloresta(vertice2, arvores)

            # caso os vertices estejam em arvores diferentes
            if(index1 != index2):
                # adiciona a aresta na AGM
                agm.append([vertice1, vertice2, peso])
                # adiciona o peso da aresta ao peso total da AGM
                peso_agm += float(peso)
                # Faz a união das arvores
                auxA = arvores[index1]
                auxB = arvores[index2]
                arvores.remove(auxA)
                arvores.remove(auxB)
                arvores.append(auxA.union(auxB))
        # retorna a arvore geradora minima
        return agm, peso_agm

    # algoritmo de Kruskal utilizando union-find implementada com abordagem quickfind
    def kruskalUnionFind_quickfind(self):
        # faz a uniao de duas arvores da floresta, fazendo com que ambas tenham a mesma identificação
        def union(idsArvores, arvore1, arvore2):
            # pegua os IDs de cada arvore
            arvore1Id = idsArvores[arvore1]
            arvore2Id = idsArvores[arvore2]
            # irá fazer a uniao das duas arvores fazendo os IDs de ambas serem iguais
            for index in range(0, len(idsArvores)):
                if(idsArvores[index] == arvore1Id):
                    idsArvores[index] = arvore2Id
            return idsArvores

        # Retorna a arvore em que o vertice em questao se encontra
        def quickfind(idsArvores, p):
            return(idsArvores[p])

        # lista que ira guardar os vertices da AGM
        agm = []
        # peso da AGM, começa com peso zerado
        peso_agm = 0
        # cria um vetor com um ID para cada vertice do grafo
        idsArvores = list(range(0, int(self.vertices)))

        # ordena as arestas do grafo de forma crescente de acordo com o peso de cada aresta
        grafoordenado = sorted(self.lista_arestas, key=lambda item: item[2])

        # para cada aresta ordenada, faça...
        for vertice1, vertice2, peso in grafoordenado:
            # verifica se os vertices já estao na mesma arvore, caso não, sera adicionado ao AGM
            if(quickfind(idsArvores, vertice1) != quickfind(idsArvores, vertice2)):
                # adiciona a aresta na AGM
                agm.append([vertice1, vertice2, peso])
                # adiciona o peso da aresta ao peso total da AGM
                peso_agm += float(peso)
                # faz a uniao das duas arvores que contenham os vertices
                idsArvores = union(idsArvores, vertice1, vertice2)

        # retorna a arvore geradora minima
        return agm, peso_agm

    # algoritmo de Kruskal que utilizando union-find implementada com a abordagem quick-union;
    def kruskalUnionFind_quickunion(self):
        # funcao auxiliar para encontrar a raiz de cada arvore
        def raiz(raizesArvores, arv):
            while(arv != raizesArvores[arv]):
                arv = raizesArvores[arv]
            return arv

        # faz a uniao de duas arvores da floresta, fazendo com que ambas tenham a mesma raiz
        def quickunion(raizesArvores, arv1, arv2):
            # pegua a raiz de cada arvore
            raiz1 = raiz(raizesArvores, arv1)
            raiz2 = raiz(raizesArvores, arv2)
            # união das arvores, fazendo com que ambas tenham a mesma raiz
            raizesArvores[raiz1] = raiz2
            return raizesArvores

        # Retorna a arvore em que o vertice em questao se encontra
        def find(raizesArvores, i):
            while i != raizesArvores[i]:
                i = raizesArvores[i]
            return i

        # lista que ira guardar os vertices da AGM
        agm = []
        # peso da AGM, começa com peso zerado
        peso_agm = 0
        # cria uma lista com a raiz de cada arvore
        raizesArvores = list(range(0, int(self.vertices)))

        # ordena as arestas do grafo de forma crescente de acordo com o peso de cada aresta
        grafoordenado = sorted(self.lista_arestas, key=lambda item: item[2])

        # para cada aresta ordenada, faça...
        for vertice1, vertice2, peso in grafoordenado:
            # verifica se os vertices já estao na mesma arvore, caso não, sera adicionado ao AGM
            if(find(raizesArvores, vertice1) != find(raizesArvores, vertice2)):
                # adiciona a aresta na AGM
                agm.append([vertice1, vertice2, peso])
                # adiciona o peso da aresta ao peso total da AGM
                peso_agm += float(peso)
                # faz a uniao das duas arvores que contenham os vertices
                raizesArvores = quickunion(raizesArvores, vertice1, vertice2)

        # retorna a arvore geradora minima
        return agm, peso_agm

    # algoritmo de Kruskal utilizando union-find implementada com a abordagem weighted quick-union;
    def kruskalUnionFind_weightedquickunion(self):
        # funcao auxiliar para encontrar a raiz de cada arvore
        def raiz(raizesArvores, arv):
            while(arv != raizesArvores[arv]):
                arv = raizesArvores[arv]
            return arv

        # faz a uniao de duas arvores da floresta, fazendo com que ambas tenham a mesma raiz
        def quickunion(raizesArvores, pesos, arv1, arv2):
            # pegua a raiz de cada arvore
            raiz1 = raiz(raizesArvores, arv1)
            raiz2 = raiz(raizesArvores, arv2)
            # união das arvores, fazendo com que ambas tenham a mesma raiz
            # Escolhe a arvore com menor peso para que ela seja unida com a que já possui o maior peso
            if pesos[raiz1] <= pesos[raiz2]:
                raizesArvores[raiz1] = raiz2
                pesos[raiz2] += pesos[raiz1]
            else:
                raizesArvores[raiz2] = raiz1
                pesos[raiz1] += pesos[raiz2]
            return raizesArvores

        # Retorna a arvore em que o vertice em questao se encontra
        def find(raizesArvores, i):
            while i != raizesArvores[i]:
                i = raizesArvores[i]
            return i

        # lista que ira guardar os vertices da AGM
        agm = []
        # peso da AGM, começa com peso zerado
        peso_agm = 0
        # cria uma lista com a raiz de cada arvore
        raizesArvores = list(range(0, int(self.vertices)))
        pesos = [1] * int(self.vertices)

        # ordena as arestas do grafo de forma crescente de acordo com o peso de cada aresta
        grafoordenado = sorted(self.lista_arestas, key=lambda item: item[2])

        # para cada aresta ordenada, faça...
        for vertice1, vertice2, peso in grafoordenado:
            # verifica se os vertices já estao na mesma arvore, caso não, sera adicionado ao AGM
            if(find(raizesArvores, vertice1) != find(raizesArvores, vertice2)):
                # adiciona a aresta na AGM
                agm.append([vertice1, vertice2, peso])
                # adiciona o peso da aresta ao peso total da AGM
                peso_agm += float(peso)
                # faz a uniao das duas arvores que contenham os vertices
                raizesArvores = quickunion(
                    raizesArvores, pesos, vertice1, vertice2)

        # retorna a arvore geradora minima
        return agm, peso_agm

    # algoritmo de Kruskal utilizando union-find implementada com a abordagem quick-union com path compression;
    def kruskalUnionFind_pathcompressionquickunion(self):
        # funcao auxiliar para encontrar a raiz de cada arvore
        def raiz(raizesArvores, arv):
            # iteratively track up to find root
            while(arv != raizesArvores[arv]):
                # make every other node in path points to it grandparent's node
                raizesArvores[arv] = raizesArvores[raizesArvores[arv]]
                arv = raizesArvores[arv]
            return arv

        # faz a uniao de duas arvores da floresta, fazendo com que ambas tenham a mesma raiz
        def quickunion(raizesArvores, arv1, arv2):
            # pegua a raiz de cada arvore
            raiz1 = raiz(raizesArvores, arv1)
            raiz2 = raiz(raizesArvores, arv2)
            # união das arvores, fazendo com que ambas tenham a mesma raiz
            raizesArvores[raiz1] = raiz2
            return raizesArvores

        # Retorna a arvore em que o vertice em questao se encontra
        def find(raizesArvores, i):
            while i != raizesArvores[i]:
                i = raizesArvores[i]
            return i

        # lista que ira guardar os vertices da AGM
        agm = []
        # peso da AGM, começa com peso zerado
        peso_agm = 0
        # cria uma lista com a raiz de cada arvore
        raizesArvores = list(range(0, int(self.vertices)))

        # ordena as arestas do grafo de forma crescente de acordo com o peso de cada aresta
        grafoordenado = sorted(self.lista_arestas, key=lambda item: item[2])

        # para cada aresta ordenada, faça...
        for vertice1, vertice2, peso in grafoordenado:
            # verifica se os vertices já estao na mesma arvore, caso não, sera adicionado ao AGM
            if(find(raizesArvores, vertice1) != find(raizesArvores, vertice2)):
                # adiciona a aresta na AGM
                agm.append([vertice1, vertice2, peso])
                # adiciona o peso da aresta ao peso total da AGM
                peso_agm += float(peso)
                # faz a uniao das duas arvores que contenham os vertices
                raizesArvores = quickunion(raizesArvores, vertice1, vertice2)

        # retorna a arvore geradora minima
        return agm, peso_agm

    # algoritmo de Kruskal utilizando union-find implementada com a abordagem weighted quick-union com path compression;
    def kruskalUnionFind_weightedquickunionpathcompression(self):
        # funcao auxiliar para encontrar a raiz de cada arvore
        def raiz(raizesArvores, arv):
            while(arv != raizesArvores[arv]):
                raizesArvores[arv] = raizesArvores[raizesArvores[arv]]
                arv = raizesArvores[arv]
            return arv

        # faz a uniao de duas arvores da floresta, fazendo com que ambas tenham a mesma raiz
        def quickunion(raizesArvores, pesos, arv1, arv2):
            # pegua a raiz de cada arvore
            raiz1 = raiz(raizesArvores, arv1)
            raiz2 = raiz(raizesArvores, arv2)
            # união das arvores, fazendo com que ambas tenham a mesma raiz
            # Escolhe a arvore com menor peso para que ela seja unida com a que já possui o maior peso
            if pesos[raiz1] <= pesos[raiz2]:
                raizesArvores[raiz1] = raiz2
                pesos[raiz2] += pesos[raiz1]
            else:
                raizesArvores[raiz2] = raiz1
                pesos[raiz1] += pesos[raiz2]
            return raizesArvores

        # Retorna a arvore em que o vertice em questao se encontra
        def find(raizesArvores, i):
            while i != raizesArvores[i]:
                i = raizesArvores[i]
            return i

        # lista que ira guardar os vertices da AGM
        agm = []
        # peso da AGM, começa com peso zerado
        peso_agm = 0
        # cria uma lista com a raiz de cada arvore
        raizesArvores = list(range(0, int(self.vertices)))
        pesos = [1] * int(self.vertices)

        # ordena as arestas do grafo de forma crescente de acordo com o peso de cada aresta
        grafoordenado = sorted(self.lista_arestas, key=lambda item: item[2])

        # para cada aresta ordenada, faça...
        for vertice1, vertice2, peso in grafoordenado:
            # verifica se os vertices já estao na mesma arvore, caso não, sera adicionado ao AGM
            if(find(raizesArvores, vertice1) != find(raizesArvores, vertice2)):
                # adiciona a aresta na AGM
                agm.append([vertice1, vertice2, peso])
                # adiciona o peso da aresta ao peso total da AGM
                peso_agm += float(peso)
                # faz a uniao das duas arvores que contenham os vertices
                raizesArvores = quickunion(
                    raizesArvores, pesos, vertice1, vertice2)

        # retorna a arvore geradora minima
        return agm, peso_agm

    # algoritmo de Prim sem utilizar fila de prioridades
    def prim(self):
        # lista que ira guardar os vertices da AGM
        agm = []
        # peso da AGM, começa com peso zerado
        peso_agm = 0

        # guarda o peso da aresta mais leve de cada vértice que a conecta à AGM parcialmente construída;
        # começa todos com peso infinito
        chave = [math.inf] * int(self.vertices)
        # o peso do vertice inicial para ele mesmo é sempre 0
        chave[0] = 0

        # guarda o vertice pai de cada vertice para a construção da AGM, no começo, não se sabe o pai de ninguem
        pai = [None] * int(self.vertices)

        # fila auxiliar que guarda todas as arestas e seus pesos minimos de acordo com que são visitados
        fila_aux = []
        for i in range(0, int(self.vertices)):
            fila_aux.append([i, math.inf])
        # o peso do vertice inicial é sempre 0
        fila_aux[0][1] = 0

        # fila  que ira guardar as arestas que ainda não foram visitadas
        fila = list(range(0, int(self.vertices)))

        # função auxiliar para ajudar a escolher o vertice minimo de acordo com o peso
        def chave_peso(e):
            return e[1]

        # enquanto ainda ouver vertices não visitados do grafo, continua a executar
        while fila:
            # pegua da fila auxiliar o vertice e o peso da sua aresta com o pai da fila
            vertice, peso_aresta = min(fila_aux, key=chave_peso)

            # caso não seja o vertice inicial, adiciona a sua aresta a AGM e soma o peso ao peso total
            if vertice != 0:
                agm.append([pai[vertice], vertice, peso_aresta])
                peso_agm = peso_agm+float(peso_aresta)

            # apos visitado, adiciona peso infinito a aresta novamente
            fila_aux[vertice][1] = math.inf
            # remove o vertice visitado da fila
            fila.remove(vertice)

            # para cada aresta conectada ao vertice atual, faça...
            for vizinho, peso in self.adj[vertice]:
                # caso o vertice conectado pela aresta ainda nao tenha sido visitado e seu peso
                # seja menor que o peso observado ate então, faça...
                if (vizinho in fila) and (peso < chave[vizinho]):
                    # seu peso é atualizado na fila auxiliar
                    fila_aux[vizinho][1] = peso
                    # o peso a é atualizado no vetor auxiliar chave
                    chave[vizinho] = peso
                    # adiciona o pai do vertice vizinho como o vertice atual
                    pai[vizinho] = vertice

        return agm, peso_agm

    # algoritmo de Prim utilizando um heap como estrutura de dados para implementação da fila de prioridades
    def prim_heap(self):
        # lista que ira guardar os vertices da AGM
        agm = []
        # peso da AGM, começa com peso zerado
        peso_agm = 0

        # guarda o peso da aresta mais leve de cada vértice que a conecta à AGM parcialmente construída;
        # começa todos com peso infinito
        chave = [math.inf] * int(self.vertices)
        # o peso do vertice inicial para ele mesmo é sempre 0
        chave[0] = 0

        # guarda o vertice pai de cada vertice para a construção da AGM, no começo, não se sabe o pai de ninguem
        pai = [None] * int(self.vertices)

        # lista com os vertices que ainda não foram visitados
        naofoi = list(range(0, int(self.vertices)))

        # função auxiliar para ajudar a escolher o vertice minimo de acordo com o peso
        def chave_peso(e):
            return e[1]

        # inicia a fila de prioridade com o vertice 0 e peso 0
        fila_heap = [(0, 0)]
        heapify(fila_heap) 
        # enquanto a fila de prioridade não estiver fazia, continua a executar
        while fila_heap:

            #u = heappop(fila_heap)
            peso_aresta, vertice = heappop(fila_heap)

            # verifica se esse vertice ainda nao foi visitado em uma iteração anterior
            # caso tenha sido, pula para a proxima
            if not(vertice in naofoi):
                continue

            # remove o vertice atual dos vertices que aindanão foram visitados
            naofoi.remove(vertice)

            # caso não seja o vertice inicial, adiciona a sua aresta a AGM e soma o peso ao peso total
            if vertice != 0:
                agm.append([pai[vertice], vertice, peso_aresta])
                peso_agm = peso_agm+float(peso_aresta)

            # para cada aresta conectada ao vertice atual, faça...
            for vizinho, peso in self.adj[vertice]:
                # caso o vertice conectado pela aresta ainda nao tenha sido visitado e seu peso
                # seja menor que o peso observado ate então, faça...
                if (vizinho in naofoi) and (peso < chave[vizinho]):
                    # o peso a é atualizado no vetor auxiliar chave
                    chave[vizinho] = peso
                    # adiciona o pai do vertice vizinho como o vertice atual
                    pai[vizinho] = vertice
                    # adiciona o vertice ainda não visitado na fila de prioridade
                    heappush(fila_heap, (peso, vizinho))

        return agm, peso_agm


def main(args):
    def arquivosaida(arq, alg, tempo, peso, arvore):
        arquivo = open(arq+".txt", 'w')
        arquivo.write(
            "====================Arvore Geradora Minima====================\n")
        arquivo.write(alg+"\n")
        arquivo.write(
            "==============================================================\n")
        arquivo.write("Tempo de execucao do Algoritmo: " + str(tempo)+"\n")
        arquivo.write("Peso da Arvore Geradora Minima: " + str(peso)+"\n")
        arquivo.write(
            "==============================================================\n")

        arv = re.sub(r"\], ", '\n', str(arvore))
        arv = re.sub(r"\[|\]", "", arv)
        arv = re.sub(r",", "", arv)

        arquivo.write("Arestas da Arvore geradora minima \n")
        arquivo.write("Vertice1, Vertice 2, Peso: \n")

        arquivo.writelines(str(arv))
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
        algoritmo = "algoritmo de Kruskal sem utilizar a estrutura union-find"
        print(algoritmo)
        inicio = time.time()
        arvore, peso = graph.kruskal_Simples()
        fim = time.time()
        arquivosaida(saida, algoritmo, fim - inicio, peso, arvore)
    elif algoritmo == 2:
        algoritmo = "algoritmo de Kruskal utilizando union-find implementada com abordagem quickfind"
        print(algoritmo)
        inicio = time.time()
        arvore, peso = graph.kruskalUnionFind_quickfind()
        fim = time.time()
        arquivosaida(saida, algoritmo, fim - inicio, peso, arvore)
    elif algoritmo == 3:
        algoritmo = "algoritmo de Kruskal que utilizando union-find implementada com a abordagem quick-union"
        print(algoritmo)
        inicio = time.time()
        arvore, peso = graph.kruskalUnionFind_quickunion()
        fim = time.time()
        arquivosaida(saida, algoritmo, fim - inicio, peso, arvore)
    elif algoritmo == 4:
        algoritmo = "algoritmo de Kruskal utilizando union-find implementada com a abordagem weighted quick-union;"
        print(algoritmo)
        inicio = time.time()
        arvore, peso = graph.kruskalUnionFind_weightedquickunion()
        fim = time.time()
        arquivosaida(saida, algoritmo, fim - inicio, peso, arvore)
    elif algoritmo == 5:
        algoritmo = "algoritmo de Kruskal utilizando union-find implementada com a abordagem quick-union com path compression;"
        print(algoritmo)
        inicio = time.time()
        arvore, peso = graph.kruskalUnionFind_pathcompressionquickunion()
        fim = time.time()
        arquivosaida(saida, algoritmo, fim - inicio, peso, arvore)
    elif algoritmo == 6:
        algoritmo = "algoritmo de Kruskal utilizando union-find implementada com a abordagem weighted quick-union com path compression;"
        print(algoritmo)
        inicio = time.time()
        arvore, peso = graph.kruskalUnionFind_weightedquickunionpathcompression()
        fim = time.time()
        arquivosaida(saida, algoritmo, fim - inicio, peso, arvore)
    elif algoritmo == 7:
        algoritmo = "algoritmo de Prim sem utilizar fila de prioridades"
        print(algoritmo)
        inicio = time.time()
        arvore, peso = graph.prim()
        fim = time.time()
        arquivosaida(saida, algoritmo, fim - inicio, peso, arvore)
    elif algoritmo == 8:
        algoritmo = "algoritmo de Prim utilizando um heap como estrutura de dados para implementação da fila de prioridades"
        print(algoritmo)
        inicio = time.time()
        arvore, peso = graph.prim_heap()
        fim = time.time()
        arquivosaida(saida, algoritmo, fim - inicio, peso, arvore)
    else:
        print("Digite uma entrada de algoritmo valida")
    # print(graph)

def main_testes(args):
    def arquivosaida(arq, alg, tempo, peso, arvore):
        arquivo = open(arq+".txt", 'w')
        arquivo.write(
            "====================Arvore Geradora Minima====================\n")
        arquivo.write(alg+"\n")
        arquivo.write(
            "==============================================================\n")
        arquivo.write("Tempo de execucao do Algoritmo: " + str(tempo)+"\n")
        arquivo.write("Peso da Arvore Geradora Minima: " + str(peso)+"\n")
        arquivo.write(
            "==============================================================\n")

        arv = re.sub(r"\], ", '\n', str(arvore))
        arv = re.sub(r"\[|\]", "", arv)
        arv = re.sub(r",", "", arv)

        arquivo.write("Arestas da Arvore geradora minima \n")
        arquivo.write("Vertice1, Vertice 2, Peso: \n")

        arquivo.writelines(str(arv))
        arquivo.close()

    if args.__len__() < 4:
        exit()

    algoritmo = int(args[1])
    entrada = args[2]
    saida = args[3]

    vertices, arestas, linhas = entrada

    graph = Grafo(vertices)
    for linha in linhas:
        aux = linha.split()
        graph.adiciona_aresta(int(aux[0]), int(aux[1]), float(aux[2]))

    if algoritmo == 1:
        algoritmo = "algoritmo de Kruskal sem utilizar a estrutura union-find"
        print(algoritmo)
        inicio = time.time()
        arvore, peso = graph.kruskal_Simples()
        fim = time.time()
        arquivosaida(saida, algoritmo, fim - inicio, peso, arvore)
    elif algoritmo == 2:
        algoritmo = "algoritmo de Kruskal utilizando union-find implementada com abordagem quickfind"
        print(algoritmo)
        inicio = time.time()
        arvore, peso = graph.kruskalUnionFind_quickfind()
        fim = time.time()
        arquivosaida(saida, algoritmo, fim - inicio, peso, arvore)
    elif algoritmo == 3:
        algoritmo = "algoritmo de Kruskal que utilizando union-find implementada com a abordagem quick-union"
        print(algoritmo)
        inicio = time.time()
        arvore, peso = graph.kruskalUnionFind_quickunion()
        fim = time.time()
        arquivosaida(saida, algoritmo, fim - inicio, peso, arvore)
    elif algoritmo == 4:
        algoritmo = "algoritmo de Kruskal utilizando union-find implementada com a abordagem weighted quick-union;"
        print(algoritmo)
        inicio = time.time()
        arvore, peso = graph.kruskalUnionFind_weightedquickunion()
        fim = time.time()
        arquivosaida(saida, algoritmo, fim - inicio, peso, arvore)
    elif algoritmo == 5:
        algoritmo = "algoritmo de Kruskal utilizando union-find implementada com a abordagem quick-union com path compression;"
        print(algoritmo)
        inicio = time.time()
        arvore, peso = graph.kruskalUnionFind_pathcompressionquickunion()
        fim = time.time()
        arquivosaida(saida, algoritmo, fim - inicio, peso, arvore)
    elif algoritmo == 6:
        algoritmo = "algoritmo de Kruskal utilizando union-find implementada com a abordagem weighted quick-union com path compression;"
        print(algoritmo)
        inicio = time.time()
        arvore, peso = graph.kruskalUnionFind_weightedquickunionpathcompression()
        fim = time.time()
        arquivosaida(saida, algoritmo, fim - inicio, peso, arvore)
    elif algoritmo == 7:
        algoritmo = "algoritmo de Prim sem utilizar fila de prioridades"
        print(algoritmo)
        inicio = time.time()
        arvore, peso = graph.prim()
        fim = time.time()
        arquivosaida(saida, algoritmo, fim - inicio, peso, arvore)
    elif algoritmo == 8:
        algoritmo = "algoritmo de Prim utilizando um heap como estrutura de dados para implementação da fila de prioridades"
        print(algoritmo)
        inicio = time.time()
        arvore, peso = graph.prim_heap()
        fim = time.time()
        arquivosaida(saida, algoritmo, fim - inicio, peso, arvore)
    else:
        print("Digite uma entrada de algoritmo valida")
    # print(graph)



if __name__ == "__main__":
    print("------------------------------------------------")
    #main(sys.argv)
    '''
    def lerArq(arq):
        arquivo = open(str(arq), 'r')
        vertices = arquivo.readline()
        arestas = arquivo.readline()
        linhas = arquivo.readlines()
        arquivo.close()
        return vertices, arestas, linhas
    grafopequeno = lerArq("agm/testes/grafo-pequeno.txt")
    
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            1, grafopequeno, 'saidaTeste/kruskal1grafo-pequeno']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            2, grafopequeno, 'saidaTeste/kruskal2grafo-pequeno']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            3, grafopequeno, 'saidaTeste/kruskal3grafo-pequeno']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            4, grafopequeno, 'saidaTeste/kruskal4grafo-pequeno']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            5, grafopequeno, 'saidaTeste/kruskal5grafo-pequeno']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            6, grafopequeno, 'saidaTeste/kruskal6grafo-pequeno']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            7, grafopequeno, 'saidaTeste/prim1grafo-pequeno']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            8, grafopequeno, 'saidaTeste/prim2grafo-pequeno']
    main_testes(args)

    def lerArq(arq):
        arquivo = open(str(arq), 'r')
        vertices = arquivo.readline()
        arestas = arquivo.readline()
        linhas = arquivo.readlines()
        arquivo.close()
        return vertices, arestas, linhas
    grafomedio1 = lerArq("agm/testes/grafo-medio1.txt")
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            1, grafomedio1, 'saidaTeste/kruskal1grafo-medio1']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            2, grafomedio1, 'saidaTeste/kruskal2grafo-medio1']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            3, grafomedio1, 'saidaTeste/kruskal3grafo-medio1']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            4, grafomedio1, 'saidaTeste/kruskal4grafo-medio1']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            5, grafomedio1, 'saidaTeste/kruskal5grafo-medio1']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            6, grafomedio1, 'saidaTeste/kruskal6grafo-medio1']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            7, grafomedio1, 'saidaTeste/prim1grafo-medio1']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            8, grafomedio1, 'saidaTeste/prim2grafo-medio1']
    main_testes(args)
    '''
    def lerArq(arq):
        arquivo = open(str(arq), 'r')
        vertices = arquivo.readline()
        arestas = arquivo.readline()
        linhas = arquivo.readlines()
        arquivo.close()
        return vertices, arestas, linhas
    grafomedio2 = lerArq("agm/testes/grafo-medio2.txt")
    '''args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            1, grafomedio2, 'saidaTeste/kruskal1grafo-medio2']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            2, grafomedio2, 'saidaTeste/kruskal2grafo-medio2']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            3, grafomedio2, 'saidaTeste/kruskal3grafo-medio2']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            4, grafomedio2, 'saidaTeste/kruskal4grafo-medio2']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            5, grafomedio2, 'saidaTeste/kruskal5grafo-medio2']
    main_testes(args)'''
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            6, grafomedio2, 'saidaTeste/kruskal6grafo-medio2']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            7, grafomedio2, 'saidaTeste/prim1grafo-medio2']
    #main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            8, grafomedio2, 'saidaTeste/prim2grafo-medio2']
    main_testes(args)
    def lerArq(arq):
        arquivo = open(str(arq), 'r')
        vertices = arquivo.readline()
        arestas = arquivo.readline()
        linhas = arquivo.readlines()
        arquivo.close()
        return vertices, arestas, linhas
    grafomedio3 = lerArq("agm/testes/grafo-medio3.txt")

    '''args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            1, grafomedio3, 'saidaTeste/kruskal1grafo-medio3']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            2, grafomedio3, 'saidaTeste/kruskal2grafo-medio3']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            3, grafomedio3, 'saidaTeste/kruskal3grafo-medio3']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            4, grafomedio3, 'saidaTeste/kruskal4grafo-medio3']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            5, grafomedio3, 'saidaTeste/kruskal5grafo-medio3']
    main_testes(args)'''
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            6, grafomedio3, 'saidaTeste/kruskal6grafo-medio3']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            7, grafomedio3, 'saidaTeste/prim1grafo-medio3']
    #main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            8, grafomedio3, 'saidaTeste/prim2grafo-medio3']
    main_testes(args)
    '''
    def lerArq(arq):
        arquivo = open(str(arq), 'r')
        vertices = arquivo.readline()
        arestas = arquivo.readline()
        linhas = arquivo.readlines()
        arquivo.close()
        return vertices, arestas, linhas
    grafogrande = lerArq("agm/testes/grafo-grande.txt")
    
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            1, grafogrande, 'saidaTeste/kruskal1grafo-grande']
    #main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            2, grafogrande, 'saidaTeste/kruskal2grafo-grande']
    #main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            3, grafogrande, 'saidaTeste/kruskal3grafo-grande']
    #main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            4, grafogrande, 'saidaTeste/kruskal4grafo-grande']
    #main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            5, grafogrande, 'saidaTeste/kruskal5grafo-grande']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            6, grafogrande, 'saidaTeste/kruskal6grafo-grande']
    main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            7, grafogrande, 'saidaTeste/prim1grafo-grande']
    #main_testes(args)
    args = ["d:/Aula/9º Semestre/Grafos/Trabalho/agm/fontes/main.py",
            8, grafogrande, 'saidaTeste/prim2grafo-grande']
    main_testes(args)
    '''
    print("------------------------------------------------")
