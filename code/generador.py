import random

def generate(n, w_mu, w_sigma, l_mu, l_sigma):
    edges = {}
    main_path = [i for i in range(1, n+1)]
    random.shuffle(main_path)

    m = random.choice([i for i in range(n-1, n*(n-1)//2 + 1)])

    for i in range(n-1):
        x = main_path[i]
        y = main_path[i+1]
        if x > y:
            x, y = y, x
        w = max(int(random.gauss(w_mu, w_sigma)), 0)
        edges[(x,y)] = w

    nodes = [i for i in range(1, n+1)]
    for _ in range(m-(n-1)):
        while True:
            x = random.choice(nodes)
            y = random.choice(nodes)
            if x > y:
                x, y = y, x
            if x != y and (x,y) not in edges:
                edges[(x,y)] = max(int(random.gauss(w_mu, w_sigma)), 0)
                break

    q = random.choice([i for i in range(1, m//2 + 1)])
    roads = []
    for _ in range(q):        
        u = random.choice(nodes)
        v = random.choice(nodes)
        l = max(0,int(random.gauss(l_mu, l_sigma)))
        roads.append((u,v,l))
    return n, edges, roads

def print_case(n, edges, roads):
    print(n, len(edges))
    for (x,y), w in edges.items():
        print(x,y,w)
    print(len(roads))
    for u,v,l in roads:
        print(u, v, l)

for i in range(10,601):
    print_case(*generate(i, 4, 2, 3, 1))

