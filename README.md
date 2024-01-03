# Biblioteca para manipulação de grafos


# Funcionalidades principais no código C#

## Classe Gráfico

Representa: Uma estrutura de dados de gráfico.
Propriedades:
nodes: Lista de nós no gráfico.
edges: Lista de arestas no gráfico.
uniqueEdges: Lista de arestas únicas (para processamento eficiente).
adjacencyList: Dicionário representando relacionamentos nó-vizinho.
adjacencyMatrix: Representação de matriz 2D do gráfico (opcionalmente calculada).
isDirected: Booleano indicando se o gráfico é dirigido.
Métodos:
Adicionar, remover e verificar nós e arestas.
Obter graus de nó e pesos de aresta.
Determinar completude e conectividade do gráfico.
Gerar gráficos aleatórios.
Encontrar pontes usando os algoritmos Naive e Tarjan.
## Classe Nó

Representa: Um nó (vértice) no gráfico.
Propriedades:
Id: Identificador exclusivo do nó.
Pode potencialmente conter dados adicionais (por exemplo, rótulos).
## Classe Aresta

Representa: Uma aresta (conexão) entre dois nós.
Propriedades:
Origem: Nó de origem.
Destino: Nó de destino.
Peso: Peso da aresta (opcional).
Label: Rótulo para a aresta (opcional).
## Pontos principais

Demonstração eficaz da representação e manipulação de gráficos em C#.
Oferece métodos para operações e algoritmos de gráfico comuns.
Suporta gráficos dirigidos e não dirigidos.
Inclui processamento paralelo para melhorar o desempenho.
