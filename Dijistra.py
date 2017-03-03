#Dijkstra Algorithm

import networkx as nx
import numpy as np
import matplotlib.pyplot as plt

###Creating directed weighted graph

G = nx.DiGraph()
G.add_edge(1,3,weight=0.6)
G.add_edge(1,4,weight=1)
G.add_edge(5,6,weight=2)
G.add_edge(1,2,weight=1)
G.add_edge(4,5,weight=0.9)
G.add_edge(3,7,weight=0.8)
G.add_edge(4,6,weight=0.9)
G.add_edge(5,2,weight=1)
G.add_edge(3,4,weight=1)
G.add_edge(2,7,weight=1)

pos = nx.spring_layout(G) #Position for all nodes

nx.draw_networkx_nodes(G,pos,node_size=500)
nx.draw_networkx_edges(G,pos,width=4)
labels = nx.get_edge_attributes(G,'weight')
nx.draw_networkx_edge_labels(G,pos,edge_labels=labels,font_color='k',alpha=0,label_pos=0.5,font_size=11)
nx.draw_networkx_labels(G,pos,font_size=20,font_family='sans-serif')
plt.axis('off')
plt.savefig("weighted_graph.png") # save as png
plt.show()


###Creating undirected weighted graph
'''
G = nx.Graph()
G.add_edge(1,3,weight=0.6)
G.add_edge(1,4,weight=1)
G.add_edge(5,6,weight=2)
G.add_edge(1,2,weight=1)
G.add_edge(4,5,weight=0.9)
G.add_edge(3,7,weight=0.8)
G.add_edge(4,6,weight=0.9)

pos = nx.spring_layout(G) #Position for all nodes

nx.draw_networkx_nodes(G,pos,node_size=500)
nx.draw_networkx_edges(G,pos,width=4)
labels = nx.get_edge_attributes(G,'weight')
nx.draw_networkx_edge_labels(G,pos,edge_labels=labels,font_color='k',alpha=0,label_pos=0.5,font_size=11)
nx.draw_networkx_labels(G,pos,font_size=20,font_family='sans-serif')
plt.axis('off')
plt.savefig("weighted_graph.png") # save as png
plt.show()
'''
