
from heapq import heappop, heappush, heapify
import networkx as nx
from itertools import count
import matplotlib.pyplot as plt


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
f = dijkstra(G1,1,4)
print(f)
