
from heapq import heappop, heappush, heapify
import networkx as nx
from itertools import count
import matplotlib.pyplot as plt

'''
class P_Dict(dict):
    def __init__(self, *args, **kwargs):
        super(P_Dict, self).__init__(*args, **kwargs)
        self._rebuild_heap()

    def _rebuild_heap(self):
        self._heap = [(v, k) for k, v in self.items()]
        heapify(self._heap)

    def pop_smallest(self):
        heap = self._heap
        v, k = heappop(heap)
        while k not in self or self[k] != v:
            v, k = heappop(heap)
        del self[k]
        return k

    def __setitem__(self, key, val):
        super(P_Dict, self).__setitem__(key, val)

        if len(self._heap) < 2 * len(self):
            heappush(self._heap, (val, key))
        else:
            self._rebuild_heap()

def dijkstra(G,v,t):
    dist_Q = P_Dict()
    dist_Q[v] = 0
    final_dist = {}
    dist = {}
    dist[v] = 0
    pred = {}
    if v == t:
        path = []
        prev = t
        while prev != None:
            path.append(prev)
            prev = pred.get(prev, None)
        print('path :', str(path))
    else:
        while len(final_dist) <len(G) and len(dist_Q) != 0:
            w = dist_Q.pop_smallest()
            final_dist[w] = dist[w]
            for x in G[w]:
                   if x not in final_dist:
                       if x not in dist:
                           dist[x] = final_dist[w] + G[w][x]['weight']
                           dist_Q[x] = final_dist[w] + G[w][x]['weight']
                       elif final_dist[w] + G[w][x]['weight'] < dist[x]:
                           dist[x] = final_dist[w] + G[w][x]['weight']
                           dist_Q[x] = final_dist[w] + G[w][x]['weight']
                           pred[x] = w
    return final_dist
'''

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
G.add_edge(1,2,weight=1)
G.add_edge(1,6,weight=0.5)
G.add_edge(2,3,weight=0.5)
G.add_edge(3,4,weight=0.5)
G.add_edge(6,5,weight=0.6)
G.add_edge(5,4,weight=2)
G.add_edge(5,3,weight=1)
G.add_edge(6,2,weight=0.4)

#nx.draw_networkx(G)
#plt.show()
G1 = nx.to_dict_of_dicts(G)
#f = dijkstra(G1,2,6)
#print(f)
(length, path) = dijkstra(G, 1 ,4, weight='weight')
print(length)
print(path)


length1 = shortest_path(G,1,4,weight='weight')
print(length1)

