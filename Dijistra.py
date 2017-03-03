#Dijkstra Algorithm

import networkx as nx
import numpy as np
import matplotlib.pyplot as plt

G = nx.Graph()
G.add_edge(1,3,weight=0.6)
G.add_edge(1,4,weight=1)
G.add_edge(5,6,weight=2)
G.add_edge(1,2,weight=1)
G.add_edge(4,5,weight=0.9)
G.add_edge(3,7,weight=0.8)
G.add_edge(4,6,weight=0.9)

pos = nx.spring_layout(G) #Position for all nodes

nx.draw_networkx_nodes(G,pos,node_size=700)
nx.draw_networkx(G)
nx.draw_networkx_edges(G,pos,width=6)
nx.draw_networkx_labels(G,pos,font_size=20,font_family='sans-serif')
plt.axis('off')
plt.savefig("weighted_graph.png") # save as png
plt.show()

#degree_sequence = list(G.degree().values())
#print('Degree Sequence:', degree_sequence)
#nodes = list(G.nodes())
#print('Nodes:', nodes)
#K = []
#for i in nodes:
#    if G.degree(i) > 1:
#        K.append(i)
#print('K:',K)
#S=G.subgraph(K)
#nx.draw_networkx(S)

