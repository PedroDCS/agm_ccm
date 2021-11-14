# agm-ccm
1. Encontrar o valor de uma árvore geradora de custo mínimo (AGM) em um grafo G (V,A).<br>
2. Encontrar o valor de um caminho de custo mínimo (CCM) em um grafo G (V,A).
<br>
<br>


## Arvore geradora de custo mínimo
### Algoritmos
1. algoritmo de Kruskal sem utilizar a estrutura union-find ;
2. algoritmo de Kruskal utilizando union-find implementada com abordagem quick-find;
3. algoritmo de Kruskal que utilizando union-find implementada com a abordagem quick-union;
4. algoritmo de Kruskal utilizando union-find implementada com a abordagem weighted quick-union;
5. algoritmo de Kruskal utilizando union-find implementada com a abordagem quick-union com path compression;
6. algoritmo de Kruskal utilizando union-find implementada com a abordagem weighted quick-union com path compression;
7. algoritmo de Prim sem utilizar fila de prioridades, e
8. algoritmo de Prim utilizando um heap como estrutura de dados para implementação da fila de prioridades.
<hr>
A execução do programa deve ser em linha de comando e receber 3 parâmetros:
<br>
x: onde x é um inteiro indicando qual versão do algoritmo utilizar
<br>
arquivo_entrada: um caminho para um arquivo de entrada. O arquivo de entrada possui o seguinte formato:
<br>
um número inteiro indicando a quantidade de vértices do grafo, um número inteiro indicando a quantidade de arestas no grafo, e uma sequência de triplas: (v a p). Onde v indica vértice, a aresta e p peso da aresta.
<br>
arq-out: um caminho para gerar de um arquivo de saída.

<br>
<br>

## Caminho de custo mínimo com algoritmo de Dijkstra 

1. uma versão do algoritmo de Dijkstra sem utilizar fila de prioridades, e
2. uma versão do algoritmo de Dijkstra que implementa a fila de prioridades usando a estrutura de dados heap.

<hr>
A execução do programa deve ser em linha de comando e receber 3 parâmetros:
<br>
x: onde x é um inteiro indicando qual versão do algoritmo utilizar
<br>
arquivo_entrada: um caminho para um arquivo de entrada. O arquivo de entrada possui o seguinte formato:
<br>
um número inteiro indicando a quantidade de vértices do grafo, um número inteiro indicando a quantidade de arestas no grafo, e uma sequência de triplas: (v a p). Onde v indica vértice, a aresta e p peso da aresta.
<br>
arq-out: um caminho para gerar de um arquivo de saída.
