from uniformSphereSurface import *

if __name__ == '__main__':
    radius = 1
    distributionDistance = 0.3
    pointList = generateUniformPoints(distributionDistance,radius)
    print(len(pointList))
    plotPoints(pointList,radius)
    print(checkUniformity(pointList,radius,3))
