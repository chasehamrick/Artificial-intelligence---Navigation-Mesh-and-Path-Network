'''
 * Copyright (c) 2014, 2015 Entertainment Intelligence Lab, Georgia Institute of Technology.
 * Originally developed by Mark Riedl.
 * Last edited by Mark Riedl 05/2015
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
'''

import sys, pygame, math, numpy, random, time, copy, operator
from pygame.locals import *

from constants import *
from utils import *
from core import *

from random import shuffle

# Creates a pathnode network that connects the midpoints of each navmesh together
def myCreatePathNetwork(world, agent = None):
    nodes = []
    edges = []
    polys = []
    ### YOUR CODE GOES BELOW HERE ###
	     ### William Hamrick ###
    
    allnodes = world.getPoints()
    shuffle(allnodes)
    
    for node1 in allnodes:
        for node2 in allnodes:
            for node3 in allnodes:
                if node1 == node2 or node1 == node3 or node2 == node3:
                    continue
                if not polygoncollides(node1, node2, node3, world, polys):
                    polys = addPolygonNoDuplicates(node1, node2, node3, polys)
    
    repeat = True

    while repeat:
        repeat = False
        for poly in polys:
            for poly2 in polys:
                if poly == poly2:
                    continue
                if polygonsAdjacent(poly, poly2):
                    newpoly = combinePolygons(poly, poly2)
                    if isConvex(newpoly):
                        polys.remove(poly)
                        polys.remove(poly2)
                        polys.append(newpoly)
                        repeat = True
                        break
                        
            if repeat:
                break
                
                
    for poly in polys:
        polynodes = []
        for i in range(len(poly)):
            hitObstacle = False
            if i >= len(poly) - 1:
                midpoint = ((poly[i][0] + poly[0][0])/2, (poly[i][1] + poly[0][1])/2)
                for obstacle in world.obstacles:
                    if (poly[i], poly[0]) in obstacle.getLines() or (poly[0], poly[i]) in obstacle.getLines():
                        hitObstacle = True
                        break
                if not hitObstacle:
                    nodes.append(midpoint)
                    polynodes.append(midpoint)
            else:
                midpoint = ((poly[i][0] + poly[i+1][0])/2, (poly[i][1] + poly[i+1][1])/2)
                for obstacle in world.obstacles:
                    if (poly[i], poly[i+1]) in obstacle.getLines() or (poly[i+1], poly[i]) in obstacle.getLines():
                        hitObstacle = True
                        break
                if not hitObstacle:
                    nodes.append(midpoint)
                    polynodes.append(midpoint)

        agentSize = world.movers[0].getMaxRadius()

        for i in range(len(polynodes)):
            if i >= len(polynodes) - 1:
                if not collides(world.obstacles, (polynodes[i], polynodes[0]), agentSize):
                    edges.append((polynodes[i], polynodes[0]))
            else:
                if not collides(world.obstacles, (polynodes[i], polynodes[i+1]), agentSize):
                    edges.append((polynodes[i], polynodes[i+1]))

    


    ### YOUR CODE GOES ABOVE HERE ###
    return nodes, edges, polys

def collides(obstacles, line, agentSize):
    for obstacle in obstacles:
        for point in obstacle.getPoints():
            if minimumDistance(line, point) < agentSize:
                return True
    return False

def sortPolygon(points):
    polygon = []

    midpoint = [0,0]
    for point in points:
        midpoint[0] += point[0]
        midpoint[1] += point[1]
    midpoint[0] /= len(points)
    midpoint[1] /= len(points)

    def computeAngle(point):
        changeInY = midpoint[1] - point[1]
        changeInX = midpoint[0] - point[0]
        angle = math.atan2(changeInY, changeInX) * 180 / math.pi
        return angle

    polygon = sorted(points, key = computeAngle)
    return polygon

def combinePolygons(poly1, poly2):
    points = []
    polygon = []
    for point in poly1:
        points.append(point)
    for point in poly2:
        if point not in points:
            points.append(point)

    polygon = sortPolygon(points)
    return polygon

def polygoncollides(node1, node2, node3, world, polys):
    worldlines = world.getLines()
    for (n1, n2, n3) in polys:
        worldlines.extend(((n1, n2), (n1, n3), (n2, n3)))

    if rayTraceWorldNoEndPoints(node1, node2, worldlines) != None and (node1, node2) not in worldlines and (node2, node1) not in worldlines:
        return True
    elif rayTraceWorldNoEndPoints(node1, node3, worldlines) != None and (node1, node3) not in worldlines and (node3, node1) not in worldlines:
        return True
    elif rayTraceWorldNoEndPoints(node2, node3, worldlines) != None and (node2, node3) not in worldlines and (node3, node2) not in worldlines:
        return True


    for obstacle in world.obstacles:
        if pointInsidePolygonLines(((node1[0] + node2[0])/2, (node1[1] + node2[1])/2), obstacle.getLines()) and (node1, node2) not in worldlines and (node2, node1) not in worldlines:
            return True
        elif pointInsidePolygonLines(((node1[0] + node3[0])/2, (node1[1] + node3[1])/2), obstacle.getLines()) and (node1, node3) not in worldlines and (node3, node1) not in worldlines:
            return True
        elif pointInsidePolygonLines(((node2[0] + node3[0])/2, (node2[1] + node3[1])/2), obstacle.getLines()) and (node2, node3) not in worldlines and (node3, node2) not in worldlines:
            return True

        midpoint = [0,0]
        for point in obstacle.getPoints():
            midpoint[0] += point[0]
            midpoint[1] += point[1]
        midpoint[0] /= len(obstacle.getPoints())
        midpoint[1] /= len(obstacle.getPoints())
        if pointInsidePolygonPoints(midpoint, (node1, node2, node3)):
            return True

    return False

def addPolygonNoDuplicates(node1, node2, node3, polys):
    if (node1, node2, node3) not in polys and (node2, node1, node3) not in polys and (node2, node3, node1) not in polys and (node1, node3, node2) not in polys and (node3, node1, node2) not in polys and (node3, node2, node1) not in polys:
        polys.append((node1, node2, node3))
    return polys
    
