
from heapq import heappop, heappush
import networkx as nx
from itertools import count
import matplotlib.pyplot as plt

#### Directed Dijkstra Algorithm ####

def dijkstra(graph, source, destination, weight = 'weight'):
    if source == destination:
        return({source: 0}, {source: [source]})
    push = heappush
    pop = heappop
    distance = {} #Dictionary which store final distances traversed
    path = {source : [source]} #Dictionary which stores paths
    seen = {source : 0}
    c = count()
    f = []
    push(f, (0, next(c), source))
    while f:
        (d, _,v) = pop(f)
        if v in distance :
            continue
        distance[v] = d
        if v == destination:
            break
        if graph.is_multigraph():
            data = []
            for w, keydata in graph[v].items():
                m_weight = min((dd.get(weight, 1)
                                 for k, dd in keydata.items()))
                data.append((w, {weight: m_weight}))
        else:
            data = iter(graph[v].items())

        for w, edgedata in data:
            vw_dist = distance[v] + edgedata.get(weight, 1)
            if w in distance:
                if vw_dist < distance[w]:
                    raise ValueError('Contradictory paths found:',
                                     'negative weights?')
            elif w not in seen or vw_dist < seen[w]:
                seen[w] = vw_dist
                push(f, (vw_dist, next(c), w))
                path[w] = path[v] + [w]
    return (distance, path)

def shortest_path(graph, source, target, weight = 'weight'):
    (length, path) = dijkstra(G, source, target, weight='weight')
    try:
        return path[target]
    except KeyError:
        raise nx.NetworkXNoPath(
            "node %s not reachable from %s" % (source, target))


G = nx.DiGraph()
G.add_edge('a','b',weight=1)
G.add_edge('a','f',weight=0.5)
G.add_edge('b','c',weight=0.5)
G.add_edge('c','d',weight=0.5)
G.add_edge('f','e',weight=0.6)
G.add_edge('e','d',weight=2)
G.add_edge('e','c',weight=1)
G.add_edge('f','b',weight=0.4)

nx.draw_networkx(G)
plt.show()
(length, path) = dijkstra(G, 'a' ,'d', weight='weight')
print(length)
print(path)

'''
(length1,path1) = shortest_path(G,'a','b',weight='weight')
print(length1)
print(path1)
'''