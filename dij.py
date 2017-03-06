import queue
import networkx as nx
import sys

MAXINT = sys.maxsize
########################################################################################################################
#                                                                                                                      #
#                                             Directed Dijkstra Algorithm                                              #
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

    path = []
    target = end
    while target is not None:
        path.append(target)
        target = node[target]

    path.reverse()
    return path, cost[end]


def graph_generator():
    # G = nx.DiGraph()
    # G.add_edge(0, 1, weight=1)
    # G.add_edge(0, 5, weight=0.5)
    # G.add_edge(1, 2, weight=0.5)
    # G.add_edge(2, 3, weight=0.5)
    # G.add_edge(5, 4, weight=0.6)
    # G.add_edge(4, 3, weight=2)
    # G.add_edge(4, 2, weight=1)
    # G.add_edge(5, 1, weight=0.4)

    G = nx.Graph()
    G.add_edge(0, 1, weight=4)  # Adding Edges and weight
    G.add_edge(0, 2, weight=2)
    G.add_edge(1, 2, weight=1)
    G.add_edge(1, 3, weight=5)
    G.add_edge(2, 3, weight=8)
    G.add_edge(2, 4, weight=10)
    G.add_edge(4, 3, weight=2)
    G.add_edge(3, 5, weight=6)
    G.add_edge(4, 5, weight=3)
    #pos = nx.spring_layout(G)  # positions for all nodes
    # nx.draw_networkx(G,pos,node_size=700)
    #labels = nx.get_edge_attributes(G, 'weight')
    # nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
    #plt.show()

    # G1 = nx.Graph()
    # G1.add_edge(0, 1, weight=1)
    # G1.add_edge(0, 5, weight=0.5)
    # G1.add_edge(1, 2, weight=0.5)
    # G1.add_edge(2, 3, weight=0.5)
    # G1.add_edge(5, 4, weight=0.6)
    # G1.add_edge(4, 3, weight=2)
    # G1.add_edge(4, 2, weight=1)
    # G1.add_edge(5, 1, weight=0.4)
    return G



########################################################################################################################
#                                                                                                                      #
#                                        Bidirectional Dijkstra Algorithm                                              #
#                                                                                                                      #
########################################################################################################################

def bidijkstra(graph,source,end):
    forwardQ = queue.PriorityQueue()
    backwardQ = queue.PriorityQueue()

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
        vertex_row1 = forwardQ.get()
        vertex_row2 = backwardQ.get()
        vertex1 = vertex_row1[1]
        vertex2 = vertex_row2[1]

        for edge in graph[vertex1]:
            if costF[edge] > costF[vertex1] + graph[vertex1][edge]['weight']:
                costF[edge] = costF[vertex1] + graph[vertex1][edge]['weight']
                nodeF[edge] = vertex1
               forwardQ.put(([costF[edge], edge]))

        for edge in graph[vertex2]:
            if costB[edge] > costB[vertex2] + graph[vertex2][edge]['weight']:
                costB[edge] = costB[vertex2] + graph[vertex2][edge]['weight']
                nodeB[edge] = vertex2
                backwardQ.put(([costB[edge], edge]))

    path = []
    target = end
    while target is not None:
        path.append(target)
        target = nodeF[target]

    path.reverse()
    return path, costF[end], costB[source]


def main():
    g = graph_generator()
    g = nx.to_dict_of_dicts(g)
    path, distance = dijkstra(g, 0, 3)
    print(path,distance)
    path, distance1,distance2 = bidijkstra(g, 0, 3)
    print(path,distance1,distance2)

if __name__ == "__main__":
    main()