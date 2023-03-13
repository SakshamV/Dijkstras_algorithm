# -*- coding: utf-8 -*-
"""
Created on Thu Mar  2 12:34:29 2023

@author: saksham
"""

import numpy as np
import pygame
import time
from sys import exit

#%%
## Making obsticle map space for point p, clearence 5

# Rectangles
def obs1(p):
    if p[0] > (100-5) and p[0]<(150+5) and p[1]<(100+5):
        return False
    else:
        return True
def obs2(p):
    if p[0] > (100-5) and p[0]<(150+5) and p[1]>(150-5):
        return False
    else:
        return True  
# Hexagon
P_act = [[235,162.5],[300,200],[365,162.5],[365,87.5],[300,50],[235,87.5]]
P     = [[230.7,165],[300,205],[369.3,165],[369.3,85],[300,45],[230.7,85]]
L1 = np.polyfit([P[0][0],P[1][0]] , [P[0][1],P[1][1]] , 1)
L2 = np.polyfit([P[1][0],P[2][0]] , [P[1][1],P[2][1]] , 1)
L3 = np.polyfit([P[3][0],P[4][0]] , [P[3][1],P[4][1]] , 1)
L4 = np.polyfit([P[4][0],P[5][0]] , [P[4][1],P[5][1]] , 1)

def obs3(p):
    L1_ = p[1] - L1[0]*p[0] - L1[1] <0
    L2_ = p[1] - L2[0]*p[0] - L2[1] <0
    L3_ = p[1] - L3[0]*p[0] - L3[1] >0
    L4_ = p[1] - L4[0]*p[0] - L4[1] >0
    if L1_ and L2_ and L3_ and L4_ and p[0]>230.7 and p[0]<369.3:
        return False
    else:
        return True

# triangle
Q_act = [[460,225],[510,125],[460,25]]
Q     = [[455,232],[517,125],[455,18]]
Q1 = np.polyfit([Q[0][0],Q[1][0]] , [Q[0][1],Q[1][1]] , 1)
Q2 = np.polyfit([Q[1][0],Q[2][0]] , [Q[1][1],Q[2][1]] , 1)

def obs4(p):
    Q1_ = p[1] - Q1[0]*p[0] - Q1[1] <0
    Q2_ = p[1] - Q2[0]*p[0] - Q2[1] >0
    if Q1_ and Q2_ and p[0]>460:
        return False
    else:
        return True
#%%

# Checker if a node falls in the obstacle space
def checkFeasibility(node):
    if obs1(node) and obs2(node) and obs3(node) and obs4(node):
        if node[0]>=5 and node[0]<=595 and node[1]>=5 and node[1]<=245:
            return True
        else:
            return False
    else:
        return False
#%%
shifter = [[-1,1],[0,1],[1,1],[1,0],[1,-1],[0,-1],[-1,-1],[-1,0]]
shiftCost = [1.4, 1, 1.4, 1, 1.4, 1, 1.4, 1]

# Given a node and its current c2c, returns the child nodes
def shift(node,cost):
    for i in range(len(shifter)):
        childNode = (node[0]+shifter[i][0], node[1]+shifter[i][1])
        if checkFeasibility(childNode):
            yield childNode, cost+shiftCost[i]

#%%

# main algorithm
def Djk(startState,goalState):
    
    # Cheking time
    startTime = time.time()
    
    if not checkFeasibility(startState) or not checkFeasibility(goalState):
        print('Infeasable states! Check Input')
        return None
    
    closedNodes = {}
    openNodes = {startState:(0,0,0)}
    
    child = 1
    
    while True:
        # popping first node
        parent=list(openNodes.keys())[0]
        closedNodes[parent] = openNodes[parent]
        
        if parent==goalState:
            print("Goal Found after",len(closedNodes),"nodes in ",time.time()-startTime, " seconds!")
            break
        
        # If node in open nodes, update cost ........
        for node,cost in shift(parent,openNodes[parent][0]):
            if node in openNodes:
                openNodes[node] = (min(cost,openNodes[node][0]),openNodes[node][1],openNodes[node][2])
                pass
            # ...... and if not, add child
            else:
                if node not in closedNodes and node != None:
                    openNodes[node] = (cost,openNodes[parent][2],child)
                    child = child + 1
       
        del openNodes[parent]
        
        # Sort the dict before popping next
        openNodes = dict(sorted(openNodes.items(), key=lambda x:x[1]))
    
    # backtracking
    backTrack = [node,parent]
    child = closedNodes[parent][1]
    while child >0:
        for key, value in closedNodes.items():
            
            if value[2] == child:
                node = key
                child = value[1]
                backTrack.append(node)
                
    backTrack.append(startState)
    backTrack.reverse()
    
    return backTrack,closedNodes,openNodes
#%%
# main code

start = input("Enter start X and Y coordinates separated by comma :")
goal  = input("Enter goal X and Y coordinates separated by comma :")

start = tuple(map(int, start.split(",")))
goal = tuple(map(int, goal.split(",")))


try:
    backTrack,closedNodes,openNodes = Djk(start,goal)
except:
    exit()

#%%

pygame.init()

screen = pygame.display.set_mode([1200, 500])
running = True
clock = pygame.time.Clock()

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill((255, 255, 255))

    pygame.draw.rect(screen, (0,0,0), pygame.Rect(100*2, 150*2, 50*2, 100*2)) #dist from left, top, w,h
    pygame.draw.rect(screen, (0,0,0), pygame.Rect(100*2, 0*2, 50*2, 100*2))
    pygame.draw.polygon(screen,(0,0,0), (np.array(Q_act)*2).tolist())
    pygame.draw.polygon(screen,(0,0,0), (np.array(P_act)*2).tolist())
    
    pygame.draw.rect(screen, (0,0,200), pygame.Rect(start[0]*2, 500-start[1]*2, 4*2, 4*2))
    pygame.draw.rect(screen, (0,0,200), pygame.Rect(goal[0]*2, 500-goal[1]*2, 4*2, 4*2))
    
    for node in closedNodes:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                break
        if not running: break
    
        pygame.draw.rect(screen, (0,100,0), pygame.Rect(node[0]*2, 500-node[1]*2, 1*2, 1*2))
        clock.tick(4800)
        pygame.display.update()
        
    for node in backTrack:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                break
        if not running: break
    
        pygame.draw.rect(screen, (200,0,0), pygame.Rect(node[0]*2, 500-node[1]*2, 1*2, 1*2))
        clock.tick(100)
        pygame.display.update()
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                pygame.quit()
pygame.quit()
