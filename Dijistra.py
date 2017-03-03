#Dijkstra Algorithm

import networkx as nx
import numpy as np
import matplotlib.pyplot as plt


###Creating directed weighted graph

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
labels = nx.get_edge_attributes(G,'weight')
#nx.draw_networkx_edge_labels(G,pos,edge_labels=labels,font_color='k',alpha=0,label_pos=0.5,font_size=11)
#nx.draw_networkx_labels(G,pos,font_size=20,font_family='sans-serif')
nx.draw_networkx(G)
plt.axis('off')
plt.savefig("weighted_graph.png") # save as png
plt.show()

G1 = nx.to_dict_of_dicts(G)

def dijkstra(graph,src,dest,visited=[],distances={},predecessors={}):

    if src not in graph:
        raise TypeError('the root of the shortest path tree cannot be found in the graph')
    if dest not in graph:
        raise TypeError('the target of the shortest path cannot be found in the graph')
    # ending condition
    if src == dest:
        # We build the shortest path and display it
        path=[]
        pred=dest
        while pred != None:
            path.append(pred)
            pred=predecessors.get(pred,None)
        print('shortest path: '+str(path)+" cost="+str(distances[dest]))
    else :
        # if it is the initial  run, initializes the cost
        if not visited:
            distances[src]=0
        # visit the neighbors
        for neighbor in graph[src] :
            if neighbor not in visited:
                new_distance = distances[src] + graph[src][neighbor]['weight']
                if new_distance < distances.get(neighbor,float('inf')):
                    distances[neighbor] = new_distance
                    predecessors[neighbor] = src
        # mark as visited
        visited.append(src)
        # now that all neighbors have been visited: recurse
        # select the non visited node with lowest distance 'x'
        # run Dijskstra with src='x'
        unvisited={}
        for k in graph:
            if k not in visited:
                unvisited[k] = distances.get(k,float('inf'))
        x=min(unvisited, key=unvisited.get)
        dijkstra(graph,x,dest,visited,distances,predecessors)

if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    #unittest.main()
    '''
    graph = {'s': {'a': 100, 'b': 1444},
            'a': {'s': 333, 'b': 444, 'c':118},
            'b': {'s': 422, 'a': 233, 'd': 222},
            'c': {'a': 22, 'd': 722, 't': 444},
            'd': {'b': 11, 'c': 111, 't': 555},
            't': {'c': 313, 'd': 115}}
    '''
    dijkstra(G1,'a','f')



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
