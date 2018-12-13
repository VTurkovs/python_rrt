from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt


def get_points():
    # theta = [-71.5650511770785, 0, 0, 0, 0]
    # alpha = [-90, 84.365432533328, -120.977098143875, 30, 0]

    theta = [108.434948822922, 0, 0, 0, 0]
    alpha = [-90, 118.277470290265, 74.7967849742169, 30, 0]

    # theta = [30, 0, 0, 0, 0]
    # alpha = [-90, 60, -30, -90, 0]

    s = [490, 150, 700, 600, 250]
    a = [0, 0, 0, 0, 0]

    theta = np.multiply(theta, np.pi / 180)
    alpha = np.multiply(alpha, np.pi / 180)

    x = [0]
    y = [0]
    z = [0]

    i = 0
    while i < len(theta):
        B = [[np.cos(theta[i]), -np.sin(theta[i]) * np.cos(alpha[i]), np.sin(theta[i]) * np.sin(alpha[i]), a[i] * np.cos(theta[i])],
             [np.sin(theta[i]), np.cos(theta[i]) * np.cos(alpha[i]), -np.cos(theta[i]) * np.sin(alpha[i]), a[i] * np.sin(theta[i])],
             [0, np.sin(alpha[i]), np.cos(alpha[i]), s[i]],
             [0, 0, 0, 1]]
        if i > 0:
            B_result = np.matmul(B_result, B)
        else:
            B_result = B

        x.append(B_result[0][3])
        y.append(B_result[1][3])
        z.append(B_result[2][3])
        i += 1

    print('X=', x[len(x) - 1])
    print('Y=', y[len(y) - 1])
    print('Z=', z[len(z) - 1])
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(x, y, z)
    ax.scatter(x, y, z)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()

    plt.figure()
    plt.plot(y, z)
    plt.scatter(y, z)
    plt.xlabel('Y')
    plt.ylabel('Z')
    plt.grid()


    # print(B_result)
    # print(x)
    # print(y)
    # print(z)



def main():
    # inverse kinematics
    # alpha1 = 84.365432533328
    # alpha2 = -120.977098143875
    # theta1 = -71.5650511770785
    x = [0,     0,      142.302494707577,   207.50390691555,   664.406517190557,  900]
    y = [0,     0,      47.4341649025244,   69.1679689718478,   221.468839063512,  300]
    z = [0,     490,    490,                1186.61784186652,   828.784850523897,  800]

    # direct kinematics
    # x = [0,     0,      75,                 250,                509.807621135331,   572.307621135331]
    # y = [0,     0,      129.903810567666,   433.012701892219,   883.012701892219,   991.265877365274]
    # z = [0,     490,    490,                1096.21778264911,   1396.21778264911,   1179.711431703]

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(x, y, z)
    ax.scatter(x, y, z)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()


if __name__ == '__main__':
    get_points()
