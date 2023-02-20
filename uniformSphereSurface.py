import random
import math
import matplotlib.pyplot as plt
import numpy as np
from scipy import spatial
from matplotlib.animation import FuncAnimation
import time

#COMMENT THIS CODE, MAKE CLASSES FOR POINTS, OPTIMIZE, ADD ANIMATION
#PARAMETERIZE TO INCLUDE RADIUS OF SPHERE AS WELL AS BEING CENTERED AROUND X,Y,Z

def checkUniformity(pointList,radius,neighbors):
    returnList = []
    pointArray = np.array([[e[0], e[1], e[2]] for e in pointList])
    kd_tree = spatial.KDTree(pointArray)
    for item in pointList:
        neighborList = kd_tree.query(item,neighbors+1)
        avg_distance = 0
        for distance in neighborList[0]:
            geodesicDistance = 2 * math.asin(((1/2)*distance)/radius) * radius
            avg_distance += geodesicDistance
        avg_distance = avg_distance/neighbors
        returnList.append(avg_distance)

    plt.hist(returnList,30)
    plt.show()

def generateUniformPoints(distributionDistance,radius):
    #generate seedPoint for Algorithm
    points = []
    while(True):
        seedPointX = (random.random() * radius * 2) - radius
        seedPointY = (random.random() * radius * 2) - radius
        seedPointZ = (random.random() * radius * 2) - radius
        seedPoint = (seedPointX,seedPointY,seedPointZ)
        distance = findDistance(seedPoint,(0,0,0))
        #break if point is arbitrarly close to being on surface of sphere
        if distance < radius + 0.00001 and distance > radius - 0.00001:
            break;

    #points.append(seedPoint)
    #print(seedPoint)

    counter = 0
    start = time.time()
    points.append(seedPoint)

    seedPoint = generateNextPoint(seedPoint, distributionDistance, -1, radius)
    points.append(seedPoint)

    while(True):

        pointArray = np.array([[e[0], e[1], e[2]] for e in points])
        # print(pointArray)
        kd_tree = spatial.KDTree(pointArray)

        newPoint = generateNextPoint(seedPoint, distributionDistance, kd_tree, radius)
        #print(newPoint)

        if newPoint != -1:
            print(newPoint)
            seedPoint = newPoint
            #mirrorPoint = (-newPoint[0],-newPoint[1],-newPoint[2])
            points.append(seedPoint)
            #if checkValidity(mirrorPoint,kd_tree,distributionDistance):
            #    points.append(mirrorPoint)
            counter = 0
        else:
            #prune rechecks using kd tree
            #build tree in the loop rather than in the function
            #maybe create a point mirrored on the opposite side to speed up as well??
            neighborList = []
            #dynamically increase radius constant in order to differentiate points with more/less neighbors as time progresses
            maxSearchConstant = radius/distributionDistance - 1
            neighborRadiusConstant = 1 + ((len(points)/(len(points) + 100000)) * maxSearchConstant)
            #print(neighborRadiusConstant * distributionDistance)
            for item in points:
                neighborList.append((kd_tree.query_ball_point(item, neighborRadiusConstant * distributionDistance, return_length=True),item))
            #print(sorted(neighborList))

            #regular
            #seedPoint = neighborList[counter][1]
            sortedNeighborList = sorted(neighborList)

            #sorted
            seedPoint = sortedNeighborList[counter][1]
            #if counter % 2 == 0:
            #    seedPoint = points[(len(points) - 1) - counter]
            #else:
                #counter -= 1
            #    seedPoint = points[counter]
            #    counter += 1

            counter += 1
            print(counter)
            #print(len(points))
        if counter == (len(points)):
            break
    end = time.time()
    print(str(end-start) + " seconds")
    return points



def generateNextPoint(seedPoint,D,kd_tree,radius):

    #generate multiple new pointss??

    returnPoints = []
    seedPointX = seedPoint[0]
    seedPointY = seedPoint[1]
    seedPointZ = seedPoint[2]

    #returnPoints.append((seedPointX,seedPointY,seedPointZ))

    if seedPointZ != 0:
        arbitraryNormalX = 1
        arbitraryNormalY = 1
        arbitraryNormalZ = (seedPointX * arbitraryNormalX + seedPointY * arbitraryNormalY) / (-seedPointZ)
    elif seedPointX != 0:
        arbitraryNormalZ = 1
        arbitraryNormalY = 1
        arbitraryNormalX = (seedPointZ * arbitraryNormalZ + seedPointY * arbitraryNormalY) /(-seedPointX)
    elif seedPointY != 0:
        arbitraryNormalX = 1
        arbitraryNormalZ = 1
        arbitraryNormalY = (seedPointX * arbitraryNormalX + seedPointZ + arbitraryNormalZ) / (-seedPointY)


    arbitraryNormal = np.array([arbitraryNormalX,arbitraryNormalY,arbitraryNormalZ])
    seedArray = np.array([seedPointX,seedPointY,seedPointZ])

    unitArbitraryNormalA = arbitraryNormal / np.sqrt(np.sum(arbitraryNormal**2))
    seedNormal = -(seedArray / np.sqrt(np.sum(seedArray ** 2)))
    unitArbitraryNormalB = np.cross(seedNormal,unitArbitraryNormalA)

    #returnPoints.append((unitArbitraryNormalA[0],unitArbitraryNormalA[1],unitArbitraryNormalA[2]))

    #need to validate geometry here...
    d = (D * D) / (2 * radius)
    #print(D)
    #print(d)

    #print(result)

    R = math.sqrt((D * D) - (d * d))

    #print(R)
    radiusPoint = (d-radius) * seedNormal
    #print(radiusPoint)

    t = random.random() * (math.pi * 2)

    num_tries = 200
    for i in range(num_tries):
        t = t + ((i/num_tries) * math.pi * 2)

        #xLine = seedPointX * i/100
        #yLine = seedPointY * i/100
        #zLine = seedPointZ * i/100

        #normalLineX = arbitraryNormalX * i/100
        #normalLineY = arbitraryNormalY * i / 100
        #normalLineZ = arbitraryNormalZ * i / 100

        #returnPoints.append((xLine,yLine,zLine))
        #returnPoints.append((normalLineX,normalLineY,normalLineZ))

        newX = R * math.cos(t) * unitArbitraryNormalA[0] + R * math.sin(t) * unitArbitraryNormalB[0]+ radiusPoint[0]
        newY = R * math.cos(t) * unitArbitraryNormalA[1] + R * math.sin(t) * unitArbitraryNormalB[1]+ radiusPoint[1]
        newZ = R * math.cos(t) * unitArbitraryNormalA[2] + R * math.sin(t) * unitArbitraryNormalB[2]+ radiusPoint[2]

        newPoint = ((newX,newY,newZ))

        #print(len(points))
        if(kd_tree == -1):
            return newPoint

        if checkValidity(newPoint,kd_tree,D):
            return newPoint

        #returnPoints.append(newPoint)


    return -1

def checkValidity(newPoint,kd_tree,D):
    if kd_tree == -1:
        print(kd_tree)
    #commenting out if kd_tree is equal to -1 then return true breaks code?
    if (kd_tree.query_ball_point(newPoint,D,return_length = True)) == 0:
        return True
    return False
    #for item in points:
    #    #print(findDistance(newPoint,item))
    #    #print(D)
    #    #could do something like if distance to nearest neighbor is blah then, could also store distances between points
    #    #everytime its calculated to prevent doubles from occurring which is whats taking the most time at the moment as the
    #    # the list size increases the number of valid points that need to be checked keeps increasing
    #    # maybe could be smart with derivates when calculating the circle?? bit of a stretch tho
    #    # best idea is to make a data structure that makes traversing this list of points as easy as possible, save some computation
    #    # when it comes to finding distance
    #    if findDistance(newPoint,item) < D and item != seedPoint:
    #        return False
    #return True

def findDistance(point1,point2):
    return math.sqrt(math.pow(point1[0] - point2[0],2) + math.pow(point1[1] - point2[1],2) + math.pow(point1[2] - point2[2],2))

def plotPoints(points,radius):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    xdata = []
    ydata = []
    zdata = []
    ax.axes.set_xlim3d(left=-radius, right=radius)
    ax.axes.set_ylim3d(bottom=-radius, top=radius)
    ax.axes.set_zlim3d(bottom=-radius, top=radius)

    for item in points:
        xdata.append(item[0])
        ydata.append(item[1])
        zdata.append(item[2])
    ax.scatter3D(xdata, ydata, zdata, c=zdata);
        #plt.pause(0.05)
    plt.show()


