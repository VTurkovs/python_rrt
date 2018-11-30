import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt


def main2():
    mpl.rcParams['legend.fontsize'] = 10

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    theta = np.linspace(-4 * np.pi, 4 * np.pi, 100)
    z = np.linspace(-2, 2, 100)
    r = z ** 2 + 1
    x = r * np.sin(theta)
    y = r * np.cos(theta)
    ax.plot(x, y, z, label='parametric curve')
    ax.legend()

    plt.show()


'''
q_init - root vertex
K - number of vertices
rho - metric
delta_q - step towards the next vertex

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




def BuildRrt(q_init, K, delta_q):
    k = 0
    while k < K - 1:
        q_rand = RandomConfiguration()


def RandomConfiguration(dimCount = 2, min=-10, max=10):
    return (max - min) * np.random.random_sample((dimCount,)) + min


if __name__ == '__main__':
    main2()
