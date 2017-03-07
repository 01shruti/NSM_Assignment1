########################################################################################################################
#                                                                                                                      #
#                                           MIS40550 Network Software Modelling                                        #
#                                                                                                                      #
#                                     Assignment 1 : Bidirectional Dijkstra Algorithm                                  #
#                                                                                                                      #
#    Due Date : 12 March 2017                                                       Submission Date : 9 March 2017     #
#                                                                                                                      #
#    Author : Shruti Goyal                                                          Professor : Dr. James McDermott    #
#                                                                                                                      #
########################################################################################################################

########################################################################################################################
#                                                                                                                      #
#                                  Importing Desired libraries for Dijkstra Algorithm                                  #
#                                                                                                                      #
########################################################################################################################

import queue
import networkx as nx
import matplotlib.pyplot as plt

########################################################################################################################
#                                                                                                                      #
# Shortestpath_length() Function                                                                                       #
#                                                                                                                      #
# This function calculates the shortest path for Dijkstra and Bidirectional Dijkstra algorithm                         #
#                                                                                                                      #
# Parameters:                                                                                                          #
# -----------                                                                                                          #
# G : Graph generated using Networkx library                                                                           #
# path : Path generated through Dijkstra and Bidirectional Dijkstra algorithm                                          #
#                                                                                                                      #
# Returns:                                                                                                             #
# --------                                                                                                             #
# Function will return the path length of the shortest path generated from algorithms                                  #
#                                                                                                                      #
########################################################################################################################

def shortestpath_length(G, path):
    cost = 0
    for x in range(len(path) - 1):
        cost = cost + G[path[x]][path[x+1]]['weight']
    return cost


########################################################################################################################
#                                                                                                                      #
# traversalF() Function                                                                                                #
#                                                                                                                      #
# This function traverses the path from source node to target node for forward direction of algorithm                  #
#                                                                                                                      #
# Parameters:                                                                                                          #
# -----------                                                                                                          #
# node : Value of the node from which traversing has to be done                                                        #
# nodeList : List consists of nodes value                                                                              #
#                                                                                                                      #
# Returns:                                                                                                             #
# --------                                                                                                             #
# Function will return the path traversed in the forward direction from the algorithm                                  #
#                                                                                                                      #
########################################################################################################################

def traversalF(node, nodeList):
    path = []
    target = node
    while target is not None:
        path.append(target)
        target = nodeList[target]
    path.reverse()
    return path

########################################################################################################################
#                                                                                                                      #
# traversalB() Function                                                                                                #
#                                                                                                                      #
# This function traverses the path from source node to target node for backward direction of bidirectional             #
# Dijkstra algorithm                                                                                                   #
#                                                                                                                      #
# Parameters:                                                                                                          #
# -----------                                                                                                          #
# node : Value of the node from which traversing has to be done                                                        #
# nodeList1 : List consists of nodes value for forward direction                                                       #
# nodeList2 : List consists of nodes value for backward direction                                                      #
#                                                                                                                      #
# Returns:                                                                                                             #
# --------                                                                                                             #
# Function will return the path traversed from the Bidirectional Dijkstra algorithm                                    #
#                                                                                                                      #
########################################################################################################################

def traversalB(node, nodeList1, nodeList2):
    path = traversalF(node, nodeList1)
    temp = nodeList2[node]
    while temp is not None:
        path.append(temp)
        temp = nodeList2[temp]
    return path

########################################################################################################################
#                                                                                                                      #
# dijkstra() Function                                                                                                  #
#                                                                                                                      #
# This function implements the Dijkstra algorithm for calculation of shortest path from source to destination          #
#                                                                                                                      #
# Parameters:                                                                                                          #
# -----------                                                                                                          #
# graph : Graph generated from Networkx library                                                                        #
# source : Value of the source node from which path has to be calculated                                               #
# end : Value of the target node till which path has to be calculated                                                  #
#                                                                                                                      #
# Returns:                                                                                                             #
# --------                                                                                                             #
# Function will return the path traversed from the Dijkstra algorithm                                                  #
#                                                                                                                      #
# Description :                                                                                                        #
# -------------                                                                                                        #
# Type of data structure used in this algorithm is Priority Queue where                                                #
#                                                                                                                      #
########################################################################################################################

def dijkstra(graph, source, end):
    queue_object = queue.PriorityQueue()
    node = []
    cost = []
    start_weight = float("inf")
    for i in graph:
        weight = start_weight
        if source == i:
            weight = 0
        cost.append(weight)
        node.append(None)
    queue_object.put(([0, source]))
    while not queue_object.empty():
        vertex_row = queue_object.get()
        vertex = vertex_row[1]
        for edge in graph[vertex]:
            if cost[edge] > cost[vertex] + graph[vertex][edge]['weight']:
                cost[edge] = cost[vertex] + graph[vertex][edge]['weight']
                node[edge] = vertex
                queue_object.put(([cost[edge], edge]))

    path = traversalF(end, node)
    return path

def graph_generator():
    # G = nx.Graph()
    # G.add_edge(0, 1, weight=1)
    # G.add_edge(0, 5, weight=2)
    # G.add_edge(1, 2, weight=12)
    # G.add_edge(2, 3, weight=5)
    # G.add_edge(5, 4, weight=6)
    # G.add_edge(4, 3, weight=13)
    # G.add_edge(4, 2, weight=7)
    # G.add_edge(5, 1, weight=4)

    # G = nx.Graph()
    # G.add_edge(0, 1, weight=4)  # Adding Edges and weight
    # G.add_edge(0, 2, weight=2)
    # G.add_edge(1, 2, weight=1)
    # G.add_edge(1, 3, weight=5)
    # G.add_edge(2, 3, weight=8)
    # G.add_edge(2, 4, weight=10)
    # G.add_edge(4, 3, weight=2)
    # G.add_edge(3, 5, weight=6)
    # G.add_edge(4, 5, weight=3)
    #pos = nx.spring_layout(G)  # positions for all nodes
    # nx.draw_networkx(G,pos,node_size=700)
    #labels = nx.get_edge_attributes(G, 'weight')
    # nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
    #nx.draw_networkx(G)
    #plt.show()

    G1 = nx.Graph()
    G1.add_edge(0, 1, weight=1)
    G1.add_edge(0, 5, weight=0.5)
    G1.add_edge(1, 2, weight=0.5)
    G1.add_edge(2, 3, weight=0.5)
    G1.add_edge(5, 4, weight=0.6)
    G1.add_edge(4, 3, weight=2)
    G1.add_edge(4, 2, weight=1)
    G1.add_edge(5, 1, weight=0.4)
    return G1



########################################################################################################################
#                                                                                                                      #
#                                        Bidirectional Dijkstra Algorithm                                              #
#                                                                                                                      #
########################################################################################################################

def bidijkstra(graph,source,end):
    forwardQ = queue.PriorityQueue()
    backwardQ = queue.PriorityQueue()
    setA = set()
    setB = set()
    nodeF = []
    nodeB = []
    costF = []
    costB = []

    start_weight = float("inf")
    for i in graph:
        weight = start_weight
        if source == i:
            weight = 0
        costF.append(weight)
        nodeF.append(None)
    forwardQ.put(([0, source]))

    for i in graph:
        weight = start_weight
        if end == i:
            weight = 0
        costB.append(weight)
        nodeB.append(None)
    backwardQ.put(([0,end]))

    while not forwardQ.empty() and not backwardQ.empty():
        if costF[end] >= costB[source]:
            path = traversalB(source, nodeF, nodeB)
        if forwardQ.qsize() + len(setA) < backwardQ.qsize() + len(setB):
            vertex_row1 = forwardQ.get()
            vertex1 = vertex_row1[1]
            setA.add(vertex1)
            for edge in graph[vertex1]:
                if costF[edge] > costF[vertex1] + graph[vertex1][edge]['weight']:
                    costF[edge] = costF[vertex1] + graph[vertex1][edge]['weight']
                    nodeF[edge] = vertex1
                    forwardQ.put(([costF[edge], edge]))

        else:
            vertex_row2 = backwardQ.get()
            vertex2 = vertex_row2[1]
            setB.add(vertex2)
            for edge in graph[vertex2]:
                if costB[edge] > costB[vertex2] + graph[vertex2][edge]['weight']:
                    costB[edge] = costB[vertex2] + graph[vertex2][edge]['weight']
                    nodeB[edge] = vertex2
                    backwardQ.put(([costB[edge], edge]))
    return path

def main():
    g = graph_generator()
    g = nx.to_dict_of_dicts(g)
    path = dijkstra(g, 0, 3)
    distance = shortestpath_length(g, path)
    print(path,distance)
    path = bidijkstra(g, 0, 3)
    distance = shortestpath_length(g,path)
    print(path,distance)

if __name__ == "__main__":
    main()