import numpy as np
import matplotlib.pyplot as plt

'''
n - number of nodes to put in the roadmap
k - number of closest neighbors to examine for each configuration

V <- None (vertices)
E <- None (edges)
while |V| < n do
    repeat
        q <- a random configuration in Q
    until q is collision-free
    V <- V U {q}
end while
for all q in V do
    Nq <- the k closest neighbors of q chosen from V according to dist
    for all q' in Nq do
        if (q, q') not in E and delta(q, q') not null then
            E <- E U {(q, q')}
        end if
    end for
end for

'''


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.neighbors = []
        self.distance = np.inf
        self.previous = None


class Edge:
    def __init__(self, node1, node2):
        self.x = [node1.x, node2.x]
        self.y = [node1.y, node2.y]

    def reverse(self):
        self.x = [self.x[1], self.x[0]]
        self.y = [self.y[1], self.y[0]]


def roadmap_construction(node_count, neighbor_count=5):
    V = []
    E = []
    i = 0
    while i < node_count:
        V.append(random_configuration())
        i += 1

    for node in V:
        nodes_near = nearest_vertices(node, V, neighbor_count)
        for node_near in nodes_near:
            edge = Edge(node, node_near)
            if edge not in E and edge.reverse() not in E:
                node.neighbors.append(node_near)
                node_near.neighbors.append(node)
                E.append(edge)

    start = random_configuration()
    start.distance = 0
    V.append(start)
    nodes_near = nearest_vertices(start, V, 1)
    start.neighbors.extend(nodes_near)
    nodes_near[0].neighbors.append(start)
    E.append(Edge(start, nodes_near[0]))

    goal = random_configuration()
    V.append(goal)
    nodes_near = nearest_vertices(goal, V, 1)
    goal.neighbors.extend(nodes_near)
    nodes_near[0].neighbors.append(goal)
    E.append(Edge(goal, nodes_near[0]))

    path = get_path(V, goal)
    plt.figure()
    plt.ioff()
    draw_graph(E, '-g')
    draw_graph(path, '-r')
    # draw_vertices(path, ax, 'r')
    plt.title("Node count: {0}; neighbor count: {1}; \nstart: ({2}, {3}); \ngoal: ({4}, {5})".
              format(node_count, neighbor_count, np.round(start.x), np.round(start.y),
                     np.round(goal.x), np.round(goal.y)))
    plt.ioff()
    plt.show()


def draw_graph(edges, color='-g'):
    for edge in edges:
        plt.plot([edge.x[0], edge.x[1]], [edge.y[0], edge.y[1]], color)


def draw_vertices(vertices, color='r'):
    for node in vertices:
        plt.scatter(node.x, node.y, c=color)


def nearest_vertices(q, graph, neighbor_count=5):
    rho_list = [(node.x - q.x) ** 2 + (node.y - q.y) ** 2 for node in graph]
    i = 0
    nodes_near = []
    graph2 = graph.copy()
    while i <= neighbor_count:
        index = rho_list.index(min(rho_list))
        nodes_near.append(graph2.pop(index))
        rho_list.pop(index)
        i += 1
    del nodes_near[0]
    return nodes_near


def random_configuration(min_x=-500, max_x=500, min_y=-500, max_y=500):
    x = (max_x - min_x) * np.random.random_sample() + min_x
    y = (max_y - min_y) * np.random.random_sample() + min_y
    return Node(x, y)


def get_path(graph, goal_node):
    path = []
    Q = graph.copy()
    while len(Q) > 0:
        distance_list = [q.distance for q in Q]
        index = distance_list.index(min(distance_list))
        del(distance_list[index])
        q = Q.pop(index)
        for neighbor in q.neighbors:
            alt = q.distance + distance_between(q, neighbor)
            if alt < neighbor.distance:
                neighbor.distance = alt
                neighbor.previous = q
    node = goal_node
    while node.previous is not None:
        path.append(Edge(node, node.previous))
        node = node.previous
    return path


def distance_between(node1, node2):
    return np.sqrt((node2.x - node1.x) ** 2 + (node2.y - node1.y) ** 2)


if __name__ == '__main__':
    max_nodes, min_nodes = 1000, 100
    max_neighbors, min_neighbors = 5, 3
    i = 0
    while i < 5:
        node_count = np.round((max_nodes - min_nodes) * np.random.random_sample() + min_nodes)
        neighbor_count = np.round((max_neighbors - min_neighbors) * np.random.random_sample() + min_neighbors)
        roadmap_construction(node_count, neighbor_count)
        i += 1
