import queue
import networkx as nx


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
    G = nx.DiGraph()
    G.add_edge(0, 1, weight=1)
    G.add_edge(0, 5, weight=0.5)
    G.add_edge(1, 2, weight=0.5)
    G.add_edge(2, 3, weight=0.5)
    G.add_edge(5, 4, weight=0.6)
    G.add_edge(4, 3, weight=2)
    G.add_edge(4, 2, weight=1)
    G.add_edge(5, 1, weight=0.4)
    return G

def main():
    g=graph_generator()
    g = nx.to_dict_of_dicts(g)
    path, distance = dijkstra(g, 0, 3)
    print(path,distance)

if __name__ == "__main__":
    main()