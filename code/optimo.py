from math import inf, log
import heapq

def dijkstra_heap(graph, start, n):
    distances = [inf for _ in range(n)]
    distances[start] = 0
    heap = [(0, start)]

    while heap:
        (current_distance, current_node) = heapq.heappop(heap)
        
        # Si el nodo actual ya se procesó, se omite
        if current_distance > distances[current_node]:
            continue
        
        # Actualiza la distancia de los nodos vecinos si es más corta que la distancia actual
        for neighbor, weight in graph[current_node]:
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(heap, (distance, neighbor))
    
    return distances

def dijkstra_array(graph, start, n):    
    dist = [inf] * n
    visited = [False] * n
    dist[start] = 0
        
    for _ in range(n):        
        min_dist = inf
        min_node = None
        for node in range(n):
            if not visited[node] and dist[node] < min_dist:
                min_dist = dist[node]
                min_node = node        
        if min_node is None:
            break        
        visited[min_node] = True
                
        for neighbor, weight in graph[min_node]:            
            if weight > 0 and not visited[neighbor]:
                distance = dist[min_node] + weight
                if distance < dist[neighbor]:
                    dist[neighbor] = distance    
    return dist

def dijkstra(graph, start, n, m):
    if m * log(n) < n*n:
        return dijkstra_heap(graph, start, n)
    return dijkstra_array(graph, start, n)

def ElViaje(n, m, edges, graph, roads):

    # Primera Parte (calculando 2q Dijkstras)
    dist = [None] * n
    for u,v,l in roads:
        if dist[u] is None:
            dist[u] = dijkstra(graph, u, n, m)
        if dist[v] is None:
            dist[v] = dijkstra(graph, v, n, m)

    # Segunda Parte (comprobando aristas útiles)
    usefull_edge_count = 0     
    for x,y,w in edges:
        for u,v,l in roads:
            if  dist[u][x] + dist[v][y] + w <=l or \
                dist[u][y] + dist[v][x] + w <=l:
                usefull_edge_count += 1
                break
    return usefull_edge_count

def execute():
    n,m = map(int, input().split())

    graph = [[] for _ in range(n)]
    edges = []
    for _ in range(m):
        u,v,w = map(int, input().split())
        edges.append((u-1,v-1,w))
        graph[u-1].append((v-1,w))
        graph[v-1].append((u-1,w))

    roads=[]
    q = int(input())
    for _ in range(q):
        u,v,l = map(int, input().split())
        roads.append((u-1,v-1,l))

    print(ElViaje(n,m,edges,graph,roads))