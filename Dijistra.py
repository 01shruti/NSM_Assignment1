#Dijkstra Algorithm
from collections import deque
from heapq import heappush, heappop
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt



###Creating Directed weighted graph

G = nx.DiGraph()
G.add_edge('a','c',weight=0.6)
G.add_edge('a','d',weight=1)
G.add_edge('e','f',weight=2)
G.add_edge('a','b',weight=1)
G.add_edge('d','e',weight=2)
G.add_edge('c','g',weight=0.8)
G.add_edge('d','f',weight=0.9)
G.add_edge('e','b',weight=1)
G.add_edge('c','d',weight=1)
G.add_edge('b','g',weight=1)

pos = nx.spring_layout(G) #Position for all nodes

#nx.draw_networkx_nodes(G,pos,node_size=500)
#nx.draw_networkx_edges(G,pos,width=4)
#labels = nx.get_edge_attributes(G,'weight')
#nx.draw_networkx_edge_labels(G,pos,edge_labels=labels,font_color='k',alpha=0,label_pos=0.5,font_size=11)
#nx.draw_networkx_labels(G,pos,font_size=20,font_family='sans-serif')
#nx.draw_networkx(G)
#plt.axis('off')
#plt.savefig("weighted_graph.png") # save as png
#plt.show()

Graph = nx.to_dict_of_dicts(G)

visited=[]                      #Empty list to store visited vertices
distances={}                    #Empty dictionary to store weights of vertices
predecessors={}                 #Empty dictionary to store predecessor of traversed vertices

def dijkstra(graph,source,target):

    ###### Checking if source and target vertices are present in the graph

    if source not in graph:
        raise TypeError('The parent cannot be found in the Graph')
    if target not in graph:
        raise TypeError('The target cannot be found in the Graph')

    ###### When the path reaches its source after traversing shortest path

    if source == target:

        s_path=[]
        previous_vertex = target
        while previous_vertex != None:

            # Appending vertices into shortest path algorithm
            s_path.append(previous_vertex)

            previous_vertex=predecessors.get(previous_vertex,None)

        print('Shortest path for the graph is ' + str(s_path)+" and the minimum distance traversed is " +
              str(distances[target]))

    else :

        # Initialising minimum distance to zero

        if not visited:
            distances[source]=0

        # Traversing adjacent nodes for each vertex

        for adj_node in graph[source] :
            if adj_node not in visited:
                calc_distance = distances[source] + graph[source][adj_node]['weight']
                if calc_distance < distances.get(adj_node,float('inf')):
                    distances[adj_node] = calc_distance
                    predecessors[adj_node] = source

        # Appending visited vertex

        visited.append(source)

        unvisited={}
        for k in graph:
            if k not in visited:
                unvisited[k] = distances.get(k,float('inf'))
        x=min(unvisited, key=unvisited.get)
        dijkstra(graph,x,target)

if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    #unittest.main()
    dijkstra(Graph,'a','f')
