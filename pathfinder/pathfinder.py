import math
import matplotlib.pyplot as plt
import networkx as nx
import pickle


class Map:
    """Define a map of size MxN containing a robot with radius R"""
    def __init__(self, m, n, r):
        # Keep values in the positive quadrant for simplicity
        self.rows = abs(m)
        self.cols = abs(n)
        self.robot_radius = abs(r)
        self.obstacles = []
    
    def addObstacle(self, obstacle):
        """Create an obstacle from a point object and a radius and add it to the map"""
        
        if obstacle.position.x < 0 or obstacle.position.x > self.cols or obstacle.position.y < 0 or obstacle.position.y > self.rows:
            print("ERROR: Obstacle out of bounds! Obstacle not added.")
            return

        for obs in self.obstacles:
            if obstacle == obs:
                print("ERROR: Obstacle already exists! Obstacle not added.")
                return
            
            if computeDistance(obstacle.position, obs.position) < (obstacle.radius + obs.radius) * math.sqrt(2):
                print("ERROR: Obstacle overlaps an existing obstacle! Obstacle not added.")
                return
        # Clear out the stored path data when the obstacles change
        self.xpath = []
        self.ypath = []
        self.obstacles.append(obstacle)       
    
    def addObstacleList(self, obstacle_list):
        """Add a list of Obstacle objects to the Map"""
        for obs in obstacle_list:
            self.addObstacle(obs)

    def visualizeMap(self):
        """Use matplotlib to visualize the current map and generated path"""
        plt.axes()
        
        # Plot obstacles
        for obs in self.obstacles:
            circle = plt.Circle((obs.position.x,obs.position.y), obs.radius, fc='blue', ec="black")
            plt.gca().add_patch(circle)
        
        # Plot paths
        line = plt.Line2D(self.xpath,self.ypath, color='black', lw=1.5)
        plt.gca().add_line(line)
        
        # Plot robot start position
        circle = plt.Circle((self.xpath[0],self.ypath[0]), self.robot_radius, fc='orange', ec="black")
        plt.gca().add_patch(circle)

        plt.axis('scaled')
        plt.axis([0, self.rows + self.robot_radius, 0, self.cols + self.robot_radius])
        plt.show()
    
    def generatePath(self, start_point, end_point):
        """Generate a path from point A to Point B with the goal of avoiding obstacles
            Return: List of Points describing the shortest path from start point to end point while avoiding obstacles"""
        if start_point.x == end_point.x and start_point.y1 == end_point.y:
            print("ERROR: Given points are the same! Try again.")
            return
        
        intersected = []
        initial_path = Path(start_point, end_point)
        
        # Check if a straight line from start to end point collides with any objects
        for obs in self.obstacles:
            # make sure obstacle is not on a start/end point
            # TODO: better way to handle this?
            if computeDistance(initial_path.start_point,obs.position) < (obs.radius + self.robot_radius):
                print("ERROR: Start point intersects with an obstacle! Try again.")
                return []
            elif computeDistance(initial_path.end_point,obs.position) < (obs.radius + self.robot_radius):
                print("ERROR: End point intersects with an obstacle! Try again.")
                return []
            
            if self.detectCollision(initial_path, obs):
                intersected.append(obs)

        if not intersected:
            # No obstacles intersect with a direct path from start to end; A straight connecting line is best
            # append end point to path and return early
            print("No object collisions detected.")
            # Store x and y components of path information separately for visualization
            self.xpath = [initial_path.start_point.x, initial_path.end_point.x]
            self.ypath = [initial_path.start_point.y, initial_path.end_point.y]
            return [initial_path.start_point, initial_path.end_point]
             
        path_segments = self.reroutePath(initial_path, intersected)

        # generate free space graph from path segments
        fsg = nx.Graph()
        for path in path_segments:            
            fsg.add_edge(path.start_point, path.end_point, weight=path.length)

        # Uncomment for Graph Visualization            
        # subax1 = plt.subplot(121)
        # nx.draw(fsg, with_labels=False, font_weight='bold')
        # plt.show()

        shortest_path = nx.shortest_path(fsg, source=initial_path.start_point,target=initial_path.end_point, weight='weight')
        
        self.xpath = []
        self.ypath = []
        # add path data by x and y components for plotting
        for node in shortest_path:
            self.xpath.append(node.x)
            self.ypath.append(node.y)

        # Return a list of Points describing the shortest path from start point to end point
        return shortest_path
    
    def reroutePath(self, path, obs_list):
        """Reroute a given path around intersecting obstacles"""
        # sort intersecting obstacles by distance from start
        obs_list.sort(key = lambda obs: obs.distance_from_start)
        intersecting_obs_list = []
        # circumscribe squares around each obstacle and store paths
        path_segments = self.circumscribeObstacles(path, obs_list)
        intersecting_paths = []

        # Check if rerouted paths intersect with any obstacles
        for path in path_segments:
            for obs in self.obstacles:
                if self.detectCollision(path, obs):
                    intersecting_obs_list.append(obs)
                    intersecting_paths.append(path)                    
        # If there were intersections, reroute the paths in question
        if intersecting_paths:
            for path in intersecting_paths:
                if path in path_segments:
                        # remove the path since it's not valid
                        path_segments.remove(path)
                if intersecting_obs_list:
                    for obs in intersecting_obs_list:
                        obs.distance_from_start = computeDistance(path.start_point, obs.position)
                path_segments += self.reroutePath(path, intersecting_obs_list)
                return path_segments
        else:
            return path_segments
    
    def circumscribeObstacles(self, ref_path, intersected):
        """Generate path segments that surround obstacles in the given list
            Param ref_path: reference path to reroute
            Param intersected: list of obstacles that intersect the reference path
            Return: List of paths"""
        # Get the slope of our initial path in rad
        ref_angle = math.atan((ref_path.end_point.y - ref_path.start_point.y)/(ref_path.end_point.x - ref_path.start_point.x))
        # first obstacle is connected to start point
        path_segments = self.circumscribeObstacle(ref_path.start_point, intersected[0], ref_angle)

        for obs in range(0,len(intersected)-1):
            path_segments += self.circumscribeObstacle(intersected[obs].endpoint_top, intersected[obs+1], ref_angle)
            path_segments += self.circumscribeObstacle(intersected[obs].endpoint_bot, intersected[obs+1], ref_angle)
        
        # last obstacle is connected to end point
        path_segments += self.circumscribeObstacle(ref_path.end_point, intersected[-1], ref_angle)
        
        return path_segments
    
    def circumscribeObstacle(self, start_point, obs, ref_angle):
        """Generate line segments from a start point to the vertexes of a square circumscribed around a given obstacle
            Reference angle is the slope of the line being rerouted
            Square edges should be at a right angle or parallel to reference line
            Resulting shape is more of a pentagon
            Return: List of Paths"""
            
        segments = []
        dist = (obs.radius + self.robot_radius) * math.sqrt(2)
        point1 = calculatePoint(obs.position, dist, ref_angle + math.pi*3/4)    # 135 deg
        point2 = calculatePoint(obs.position, dist, ref_angle + math.pi/4)      # 45 deg
        point3 = calculatePoint(obs.position, dist, ref_angle + math.pi*5/4)    # 225 deg
        point4 = calculatePoint(obs.position, dist, ref_angle + math.pi*7/4)    # 315 deg

        # segments from start point; get the closer one to account for start points on the opposite side, e.g. end point
        if (computeDistance(start_point, point1) < computeDistance(start_point, point2)):
            segments.append(Path(start_point, point1))
            obs.endpoint_top = point2   # store the point on the opposite side of the figure to use as next start point
        else:
            segments.append(Path(start_point, point2))
            obs.endpoint_top = point1

        if (computeDistance(start_point, point3) < computeDistance(start_point, point4)):
            segments.append(Path(start_point, point3))
            obs.endpoint_bot = point4
        else:
            segments.append(Path(start_point, point4))
            obs.endpoint_bot = point3
          
        segments.append(Path(point1, point2))   # segments defining rest of square
        segments.append(Path(point3, point4))
        segments.append(Path(point1, point3))
        segments.append(Path(point2, point4))
        # Return list of paths
        return segments
    
    def detectCollision(self, path, obs):
        """ Calculate the distance from an obstacle's center to the given line segment defined by a start and end point. 
            Return True if distance from center to line is less than obstacle radius plus robot radius."""
        
        a = computeDistance(path.start_point, path.end_point)                # distance from start point to end point
        b = computeDistance(path.start_point, obs.position)  # distance from start point to obstacle center
        c = computeDistance(obs.position, path.end_point)  # distance from obstacle center to end point

        obs.distance_from_start = b     # Keep track of distance from start for generating free space graph

        # Verify that obstacle isn't past an endpoint of the path
        if b > a + self.robot_radius or c > a + self.robot_radius:
            return False
        
        # Use law of Cosines to determine angle between start point and obstacle center 
        theta = max( min(1, ((math.pow(a, 2) + math.pow(b, 2) - math.pow(c,2)) / (2 * a * b))), -1) # limit theta to range of acos(x)
        theta = math.acos(theta)

        # Distance to Line = distance from Start to Object center * tan(theta)
        d2l = b * math.tan(theta)

        # If we alter the size of the obstacle radius to account for the robot radius we can treat the robot as a point
        return d2l < (obs.radius + self.robot_radius)


# Utility functions
def calculatePoint(ref_point, dist, angle):
    """Calculate the location of a point at a given distance and angle from reference point"""
    x = ref_point.x + dist * math.cos(angle)
    y = ref_point.y + dist * math.sin(angle)
    return Point(x, y)
    
def computeDistance(start_point, end_point):
    """Compute distance between two points"""
    return math.sqrt(math.pow((start_point.x - end_point.x), 2) + math.pow((start_point.y - end_point.y), 2))

def saveMap(map, filename):
    """Saves the given map to a file in the local directory with the given filename"""
    with open(filename, 'wb') as map_file:
        pickle.dump(map, map_file)

def loadMap(filename):
    """Returns a map object loaded from a file in the local directory with the given filename"""
    with open(filename, 'rb') as map_file:
        return pickle.load(map_file)
        
# Class definitions
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        return hash((self.x, self.y))

class Path:
    def __init__(self, point1, point2):
        self.start_point = point1
        self.end_point = point2
        self.length = computeDistance(self.start_point, self.end_point)
    
    def __eq__(self, other):
        return self.start_point == other.start_point and self.end_point == other.end_point
    
    def __hash__(self):
        return hash((self.start_point, self.end_point))
    
class Obstacle:
    def __init__(self, point, r):
        self.position = point
        self.radius = r
        self.distance_from_start = None
        self.endpoint_top = None   # endpoints for generating circumscribing square for next obstacle in a sequence
        self.endpoint_bot = None

    def __eq__(self, other):
        return self.position == other.position and self.radius == other.radius
    
    def __hash__(self):
        return hash((self.position, self.radius))