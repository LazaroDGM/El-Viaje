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


#==============================N2===============================================
def multi_dijkstra_array(graph, dist, n):    
    visited = [False] * n

        
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

def ElViaje_n2(n, m, edges, graph, roads):
    
    if edges == None or edges == [] or roads == None or roads == []:
        return 0

    # Primera Parte (calculando a lo sumo n Dijkstras)
    d = [None] * n
    for u,v,l in roads:
        if d[u] is None:
            d[u] = dijkstra(graph, u, n, m)
        if d[v] is None:
            d[v] = dijkstra(graph, v, n, m)

    # Segunda Parte (comprobando aristas útiles)
    usefull_edge_count = 0
    usefull_edges_mark = [[False] * n for _ in range(n)]

    mark = [False] * n
    
    for _ in range(n):
        v = None
        dist = [inf] * n

        for vi, ui, li in roads:
            if v == None:
                if not mark[vi]:
                    v = vi
                    mark[v] = True
            if v == vi:
                dist[ui] = - li

        if v == None: break

        dist = multi_dijkstra_array(graph, dist, n)

        for x,y,w in edges:
            if not usefull_edges_mark[x][y]:
                if dist[x] <= -w - d[v][y] or dist[y] <= -w - d[v][x]:
                    usefull_edge_count +=1
                    usefull_edges_mark[x][y] = True
                    usefull_edges_mark[y][x] = True

    return usefull_edge_count