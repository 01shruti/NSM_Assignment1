from collections import deque
from heapq import heappop, heappush
import networkx as nx
from itertools import count
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
