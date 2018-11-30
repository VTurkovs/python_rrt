"""
Path Planning Sample Code with Randomized Rapidly-Exploring Random Trees (RRT)
author: AtsushiSakai(@Atsushi_twi)
"""

import matplotlib.pyplot as plt
import random
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
import math
import copy

show_animation = True


class RRT:
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, randArea, expandDis=1.0, goalSampleRate=5, maxIter=500):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,z,size],...]
        randArea:Random Sampling Area [min,max]
        """
        self.fig = plt.figure()

        self.ax = plt.axes(projection='3d')
        self.start = Node(start[0], start[1], start[2])
        self.end = Node(goal[0], goal[1], goal[2])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList

    def Planning(self, animation=True):
        """
        Pathplanning
        animation: flag for animation on or off
        """

        self.nodeList = [self.start]
        while True:
            # Random Sampling
            if random.randint(0, 100) > self.goalSampleRate:
                rnd = [random.uniform(self.minrand, self.maxrand), random.uniform(
                    self.minrand, self.maxrand), random.uniform(self.minrand, self.maxrand)]
            else:
                rnd = [self.end.x, self.end.y, self.end.z]

            # Find nearest node
            nind = self.GetNearestListIndex(self.nodeList, rnd)
            # print(nind)

            # expand tree
            nearestNode = self.nodeList[nind]
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
            theta1 = math.atan2(rnd[2] - nearestNode.z, math.sqrt((rnd[1] - nearestNode.y)**2 + (rnd[0] - nearestNode.x)**2 ))

            newNode = copy.deepcopy(nearestNode)
            newNode.x += self.expandDis * math.cos(theta)
            newNode.y += self.expandDis * math.sin(theta)
            newNode.z += self.expandDis * math.sin(theta1)
            newNode.parent = nind

            if not self.__CollisionCheck(newNode, self.obstacleList):
                continue

            self.nodeList.append(newNode)
            print("nNodelist:", len(self.nodeList))

            # check goal
            dx = newNode.x - self.end.x
            dy = newNode.y - self.end.y
            dz = newNode.z - self.end.z
            d = math.sqrt(dx * dx + dy * dy + dz * dz)
            if d <= self.expandDis:
                print("Goal!!")
                break

            if animation:
                self.DrawGraph(rnd)

        path = [[self.end.x, self.end.y, self.end.z]]
        last_index = len(self.nodeList) - 1
        while self.nodeList[last_index].parent is not None:
            node = self.nodeList[last_index]
            path.append([node.x, node.y, node.z])
            last_index = node.parent
        path.append([self.start.x, self.start.y, self.start.z])

        return path

    def DrawGraph(self, rnd=None):
        """
        Draw Graph
        """


        #self.ax.clf()
        if rnd is not None:
            self.ax.plot3D(rnd[0], rnd[1], rnd[2])
        for node in self.nodeList:
            if node.parent is not None:
                self.ax.plot3D([node.x, self.nodeList[node.parent].x],
                         [node.y, self.nodeList[node.parent].y],
                         [node.z, self.nodeList[node.parent].z])

        for (ox, oy, oz, size) in self.obstacleList:
            self.ax.plot3D(ox, oy, oz, "ok", ms=30 * size)

        self.ax.plot3D(self.start.x, self.start.y, self.start.z)
        self.ax.plot3D(self.end.x, self.end.y, self.end.z)
        self.ax.axis([-2, 15, -2, 15, -2, 15])
        self.ax.grid(True)
        self.ax.pause(0.01)

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 + (node.z - rnd[2]) ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))
        return minind

    def __CollisionCheck(self, node, obstacleList):

        for (ox, oy, oz, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            dz = oz - node.z
            d = math.sqrt(dx * dx + dy * dy + dz * dz)
            if d <= size:
                return False  # collision

        return True  # safe


class Node():
    """
    RRT Node
    """

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None


def main():
    print("start simple RRT path planning")

    # ====Search Path with RRT====
    obstacleList = [
        (5, 5, 1, 1),
        (3, 6, 2, 2),
        (3, 8, 2, 2),
        (3, 10, 2, 2),
        (7, 5, 2, 2),
        (9, 5, 2, 2)
    ]  # [x,y,z,size]
    # Set Initial parameters
    rrt = RRT(start=[0, 0, 0], goal=[5, 10, 3],
              randArea=[-2, 15], obstacleList=obstacleList)
    path = rrt.Planning(animation=show_animation)

    # Draw final path
    if show_animation:
        rrt.DrawGraph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    main()
