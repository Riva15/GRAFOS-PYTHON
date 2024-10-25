import networkx as nx
import matplotlib.pyplot as plt
import heapq

class GrafoDirigido:
    def __init__(self):
        self.nodos = {}

    def agregar_nodo(self, nodo):
        if nodo not in self.nodos:
            self.nodos[nodo] = []

    def agregar_arista(self, nodo_origen, nodo_destino, peso):
        if nodo_origen in self.nodos and nodo_destino in self.nodos:
            self.nodos[nodo_origen].append((nodo_destino, peso))

    def dijkstra(self, nodo_inicio):
        distancias = {nodo: float('inf') for nodo in self.nodos}
        distancias[nodo_inicio] = 0
        caminos = {nodo: [] for nodo in self.nodos}
        caminos[nodo_inicio] = [nodo_inicio]

        cola_prioridad = [(0, nodo_inicio)]
        heapq.heapify(cola_prioridad)

        while cola_prioridad:
            distancia_actual, nodo_actual = heapq.heappop(cola_prioridad)

            if distancia_actual > distancias[nodo_actual]:
                continue

            for vecino, peso in self.nodos[nodo_actual]:
                distancia = distancia_actual + peso
                if distancia < distancias[vecino]:
                    distancias[vecino] = distancia
                    caminos[vecino] = caminos[nodo_actual] + [vecino]
                    heapq.heappush(cola_prioridad, (distancia, vecino))

        return distancias, caminos

# Configuración del grafo de 10 nodos y ejecución de Dijkstra
grafo = GrafoDirigido()
for nodo in ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J']:
    grafo.agregar_nodo(nodo)

# Agregar aristas con pesos entre los nodos
aristas = [
    ('A', 'B', 2), ('A', 'C', 3), ('B', 'D', 1), ('C', 'E', 5),
    ('D', 'F', 2), ('E', 'G', 3), ('F', 'H', 4), ('G', 'I', 1),
    ('H', 'J', 2), ('B', 'E', 4), ('C', 'F', 6), ('D', 'G', 3),
    ('E', 'H', 2), ('F', 'I', 4), ('G', 'J', 3)
]

for origen, destino, peso in aristas:
    grafo.agregar_arista(origen, destino, peso)

distancias, caminos = grafo.dijkstra('A')

# Visualización del grafo con NetworkX y Matplotlib
G = nx.DiGraph()

# Agregar nodos y aristas con pesos al grafo de NetworkX
for nodo in grafo.nodos:
    G.add_node(nodo)

for nodo_origen, vecinos in grafo.nodos.items():
    for nodo_destino, peso in vecinos:
        G.add_edge(nodo_origen, nodo_destino, weight=peso)

# Disposición hexagonal personalizada de los nodos
hex_pos = {
    'A': (0, 0), 'B': (1, 1), 'C': (2, 0), 'D': (3, 1), 'E': (4, 0),
    'F': (1, -1), 'G': (3, -1), 'H': (5, 1), 'I': (4, -2), 'J': (6, 0)
}

# Dibujar nodos y etiquetas
nx.draw(G, hex_pos, with_labels=True, node_size=700, node_color="lightgreen", font_weight="bold", font_size=10)
nx.draw_networkx_edge_labels(G, hex_pos, edge_labels={(u, v): d['weight'] for u, v, d in G.edges(data=True)})

# Resaltar el camino mínimo desde 'A'
nodo_inicio = 'A'
for nodo_destino, camino in caminos.items():
    if nodo_destino != nodo_inicio and camino:
        nx.draw_networkx_edges(G, hex_pos, edgelist=[(camino[i], camino[i+1]) for i in range(len(camino)-1)],
                               edge_color="blue", width=2.5)

plt.title("Grafo Dirigido con Camino Mínimo desde 'A' en Disposición Hexagonal")
plt.show()
