import numpy as np
import matplotlib.pyplot as plt

'''
BUILD_RRT(q_init, K, delta_q)
1 	G.init(q_init);
2 	for k = 1 to K
3 	q_rand <- RAND_CONF();
4 	q_near <- NEAREST_VERTEX(q_rand, G);
5 	q_new <- NEW_CONF(q_near, delta_q);
6 	G.add_vertex(q_new);
7 	G.add_edge(q_near, q_new);
8 	Return G
'''

'''
NEAREST_VERTEX(q, G)
1 	d <- infinity;
2 	for each v in V
3 	if rho(q, v) < d then
4 	v_new = v; d <- rho(q, v);
5 	Return q;
'''


class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent


def build_rrt(q_init, q_goal, delta_q, max_iter=10000):
    k = 0
    G = [q_init]
    path = []
    while True:
        q_rand = random_configuration()
        q_near = nearest_vertex(q_rand, G)
        q_new = new_config(q_near, delta_q)
        G.append(q_new)
        if is_goal(q_new, q_goal, delta_q):
            print('Goal found!')
            G.append(Node(q_goal.x, q_goal.y, q_new))
            path = get_path(G)
            break
        if k >= max_iter:
            print('Goal not found!')
            break
        k += 1

    plt.figure()
    plt.ioff()
    draw_graph(G)
    draw_graph(path, '-r')
    plt.title("iteration count: {}:\nstep: {};\nstart: ({}, {}); goal: ({}, {})"
              .format(k, delta_q, np.round(q_init.x), np.round(q_init.y), np.round(q_goal.x),
                      np.round(q_goal.y)))
    plt.show()
    pass


def random_configuration(min_x=-100, max_x=100, min_y=-100, max_y=100):
    x = (max_x - min_x) * np.random.random_sample() + min_x
    y = (max_y - min_y) * np.random.random_sample() + min_y
    return Node(x, y)


def nearest_vertex(q, graph):
    rho_list = [(node.x - q.x) ** 2 + (node.y - q.y) ** 2 for node in graph]
    node_near = graph[rho_list.index(min(rho_list))]
    theta_xy = np.arctan2(q.y - node_near.y, q.x - node_near.x)
    return [node_near, theta_xy]


def new_config(q, delta):
    x_new = q[0].x + np.cos(q[1]) * delta
    y_new = q[0].y + np.sin(q[1]) * delta
    return Node(x_new, y_new, q[0])


def draw_graph(graph, color='-g'):
    for node in graph:
        if node.parent is None:
            continue
        plt.plot([node.x, node.parent.x], [node.y, node.parent.y], color)


def is_goal(node, goal, delta):
    rho = (node.x - goal.x) ** 2 + (node.y - goal.y) ** 2
    if rho <= delta ** 2:
        return True
    return False


def get_path(graph):
    node = graph[len(graph) - 1]
    path = []
    while node is not None:
        path.append(node)
        node = node.parent

    path.append(graph[0])
    return path


if __name__ == '__main__':
    min_step, max_step = 5, 30
    i = 0
    while i < 5:
        step = np.round((max_step - min_step) * np.random.random_sample() + min_step)
        q_start = random_configuration()
        q_end = random_configuration()
        build_rrt(q_start, q_end, step)
        i += 1
