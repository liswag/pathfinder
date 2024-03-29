from pathfinder import pathfinder as pf
from random import *

# Generate a randomly populated map
f = pf.Map(250,250,3)
obstacle_list = []
for x in range(0,25):
    obstacle_list.append(pf.Obstacle(pf.Point(randint(0,250),randint(0,250)), randint(5,10) ) )
f.addObstacleList(obstacle_list)

# Obstacles can be added individually
# f.addObstacle(pf.Obstacle(pf.Point(50,50),10))
# f.addObstacle(pf.Obstacle(pf.Point(125,125),10))
# f.addObstacle(pf.Obstacle(pf.Point(200,200),10))

# Generate Path and visualize map
f.generatePath(pf.Point(3,3), pf.Point(249,249))
f.visualizeMap()

# Saving/Loading a map
# pf.saveMap(f, 'foo.map')

# x = pf.loadMap('demo.map')

# x.generatePath(pf.Point(3,3), pf.Point(249,249))
# x.visualizeMap()