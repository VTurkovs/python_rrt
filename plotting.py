import matplotlib.pyplot as plt
import csv
import constants as const
import math

robotPoints = [[0.0, 0.0, 0.0]]
gliphPoints = [[0.0, 0.0, 0.0]]
gliph = []
timeReactD = []

listColumn = lambda matrix, columnNo : [row[columnNo] for row in matrix]

def getRobotPoints():
    # with open("C:\\Users\\User\\Desktop\\Vilnis\\robobobo\\reactd_0001.log") as csvfile:
    #     reader = csv.DictReader(csvfile, ['time', 'gyro', 'unknown', 'left_encoder', 'right_encoder'])

    with open("C:\\Users\\User\\Desktop\\Vilnis\\robobobo\\reactd_0000.log") as csvfile:
        reader = csv.DictReader(csvfile, ['time', 'gyro', 'gyroX', 'gyroY', 'accelX', 'accelY', 'accelZ', 'Lidar',
                                      'left_encoder', 'right_encoder'])


        reactDList = []
        for row in reader:
            reactDList.append([float(row['time']), float(row['gyro']), float(row['left_encoder']), float(row['right_encoder'])])

        dThetaRobot = 0.0
        thetaRobot = 0.0
        xRobot = 0.0
        yRobot = 0.0
        dS = 0.0
        time = 0.0
        sizeReactD = 0

        gyroErrorSum = 0
        gyroError = 0
        gyroErrorCount = 0

        for [timeG, gyro, leftEncoder, rightEncoder] in reactDList:
            if leftEncoder == 0.0 and rightEncoder == 0.0:
                gyroErrorSum += gyro
                gyroErrorCount += 1
            else:
                gyroError = gyroErrorSum / gyroErrorCount

            timeReactD.append(timeG + time)
            time = timeReactD[sizeReactD]

            thetaRobot = robotPoints[sizeReactD][2] + dThetaRobot
            xRobot = robotPoints[sizeReactD][0] + dS * math.cos(thetaRobot + dThetaRobot / 2)
            yRobot = robotPoints[sizeReactD][1] + dS * math.sin(thetaRobot + dThetaRobot / 2)
            robotPoints.append([xRobot, yRobot, thetaRobot])

            dThetaRobot = (gyro - gyroError) * const.G
            dS = (leftEncoder + rightEncoder) * const.N / 2
            sizeReactD = sizeReactD + 1

        del robotPoints[0]


def getGliphPoints():
    # with open("C:\\Users\\User\\Desktop\\Vilnis\\robobobo\\glifd_0001.log") as csvfile:
    #     reader = csv.DictReader(csvfile,
    #                             ['time', 'unknown', 'dx', 'dy', 'unknown1', 'unknown2', 'unknown3', 'theta_gliph',
    #                              'unknown4'])
    with open("C:\\Users\\User\\Desktop\\Vilnis\\robobobo\\glifd_0000.log") as csvfile:
        reader = csv.DictReader(csvfile, ['time', 'id', 'dx', 'dy', 'dz', 'theta_gliph'])

        timeGliph = 0.0
        dxGliph = 0.0
        dyGliph = 0.0
        startRowReactD = 0
        thetaGliph = 0.0
        for row in reader:
            timeGliph += float(row['time'])
            dxGliph = float(row['dx'])
            dyGliph = float(row['dy'])
            thetaGliph = float(row['theta_gliph'])
            #print(float(row['theta_gliph']) / math.pi * 180)
            j = startRowReactD

            while j < len(robotPoints):
                if math.fabs(timeGliph - timeReactD[j]) <= 0.3:
                    startRowReactD = j
                    gliphPnt = gliphPoint(dxGliph, dyGliph, robotPoints[j][0], robotPoints[j][1], robotPoints[j][2], thetaGliph)
                    gliphPoints.append(gliphPnt)
                    print(gliphPnt[0], gliphPnt[1])
                    break;
                j += 1


        del gliphPoints[0]


def gliphPoint(dxGliph, dyGliph, xRobot, yRobot, thetaRobot, thetaGliph):
    r = math.sqrt(math.pow(dxGliph, 2) + math.pow(dyGliph, 2))
    alpha = math.atan2(dyGliph, dxGliph)
    gamma = alpha - (math.pi / 2 - thetaRobot)
    dX = r * math.cos(gamma)
    dY = r * math.sin(gamma)
    return [xRobot + dX, yRobot + dY, thetaGliph]


def getGliphAverage():
    i = 0
    j = 0
    k = 0
    threshold = 1
    print()
    gliphPoints.sort()
    while j < len(gliphPoints) and k < len(gliphPoints):
        gliph.append([gliphPoints[j][0], gliphPoints[j][1], 1, 0.0, 0.0])
        while k < len(gliphPoints):
            if math.fabs(gliphPoints[j][0] - gliphPoints[k][0]) <= threshold and math.fabs(
                    gliphPoints[j][1] - gliphPoints[k][1]) <= threshold:

                gliph[i][0] += gliphPoints[k][0]
                gliph[i][1] += gliphPoints[k][1]
                gliph[i][2] += 1

            else:
                j = k
                i += 1
                break
            k += 1
    i = 0
    while i < len(gliph):
        gliph[i][3] = gliph[i][0] / gliph[i][2]
        gliph[i][4] = gliph[i][1] / gliph[i][2]
        print(gliph[i][3], gliph[i][4])
        i += 1




def transformPoints():
    # RotMtx = [[math.cos(theta), math.sin(theta)], [-math.sin(theta), math.cos(theta)]]
    # translation by
    xGliph = gliph[0][3]
    yGliph = gliph[0][4]

    theta1 = 0 # rotation by
    theta1 = - gliphPoints[0][2] - math.pi / 15
    cosTheta = math.cos(theta1)
    sinTheta = math.sin(theta1)

    i = 0
    for [dxSum, dySum, count, x, y] in gliph:
        dx = (x - xGliph)
        dy = (y - yGliph)
        gliph[i][3] = dx * cosTheta - dy * sinTheta
        gliph[i][4] = dx * sinTheta + dy * cosTheta
        i += 1

    i = 0
    for [x, y, theta] in robotPoints:
        dx = (x - xGliph)
        dy = (y - yGliph)
        robotPoints[i][0] = dx * cosTheta - dy * sinTheta
        robotPoints[i][1] = dx * sinTheta + dy * cosTheta
        i += 1



def transformPoints2():
    # RotMtx = [[math.cos(theta), math.sin(theta)], [-math.sin(theta), math.cos(theta)]]
    for [x, y, thetaGliph] in gliphPoints:
        for [dxSum, dySum, count, xg, yg] in gliph:
            if math.fabs(xg - x) <= 0.3 and math.fabs(yg - y) <= 0.3:
                xGliph = xg
                yGliph = yg
                break
        thetaGliph = thetaGliph - math.pi
        cosTheta = math.cos(thetaGliph)
        sinTheta = math.sin(thetaGliph)

        gliphCopy = []
        i = 0
        for [dxSum, dySum, count, x, y] in gliph:
            dx = (x - xGliph)
            dy = (y - yGliph)
            gliphCopy.append([dx * cosTheta - dy * sinTheta, dx * sinTheta + dy * cosTheta])
            i += 1

        robotPointCopy = []
        i = 0
        for [x, y, theta] in robotPoints:
            dx = (x - xGliph)
            dy = (y - yGliph)
            robotPointCopy.append([dx * cosTheta - dy * sinTheta, dx * sinTheta + dy * cosTheta])
            i += 1


        plt.plot(listColumn(robotPointCopy, 0), listColumn(robotPointCopy, 1))
        plt.scatter(listColumn(gliphCopy, 0), listColumn(gliphCopy, 1))
        plt.scatter([0, -1.2, -2.6], [0.0, 0.0, 1.2], color='g', marker='x')
        plt.xlim(left=-5, right=5)
        plt.ylim(bottom=-5, top=5)
        plt.show()


if __name__ == '__main__':
    getRobotPoints()
    getGliphPoints()
    getGliphAverage()

    transformPoints()

    plt.plot(listColumn(robotPoints, 0), listColumn(robotPoints, 1))
    plt.scatter(listColumn(gliph, 3), listColumn(gliph, 4))
    # plt.scatter([0, -1.2, -2.6], [0.0, 0.0, 1.2], color='g', marker='x')
    # plt.xlim(left=-5, right=5)
    # plt.ylim(bottom=-5, top=5)
    plt.show()

    # transformPoints2()