#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np
import heapq

vel = Twist()

map = np.array([
       [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,1,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
       [0,0,1,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
       [0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0],
       [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0],
       [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]],dtype=object)


def callback(odom):

    global x, y, theta

    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    theta = odom.pose.pose.orientation.z
    return 0

def vis_path(path):
    
    vis_map = np.where(map==1, '1', '0')
    
    for node in path:
        if node != start_node and node != goal_node:
            vis_map[node] = '*'
        
    for row in vis_map:
        print(" ".join(row))
    
    return 0

def get_ancestors(parent_childs):
    # parent lookup dictionary
    child_parent = {}
    for parent, children in parent_childs.items():
        for child in children:
            child_parent[child] = parent
    
    # finding parents
    path = [goal_node]
    parent = goal_node
    while parent != start_node:
        parent = child_parent[parent]
        path.append(parent)
    
    return path


def aStar(map_):
    
    open_list = []
    open_dic = {}
    closed_list = set()
    parent_childs = {}

    # start node indicies
    i, j = start_node

    # goal node indicies
    x, y = goal_node

    # g, h costs of start node
    h_cost = abs(x - i) + abs(y - j)
    g_cost = 0
    f_cost = h_cost + g_cost

    # add start node to open
    heapq.heappush(open_list, (f_cost, g_cost, (i, j)))
    open_dic[(i, j)] = 0

    while True:

        # get current node with min f-cost
        # remove current node from from open
        _, g_cost_current, current_node = heapq.heappop(open_list)


        # add current node to the closed list
        closed_list.add(current_node)

        # if path found exit loop
        if current_node == goal_node:
            print("shortest path found")
            break

        # finding neighbours to current node
        i, j = int(current_node[0]), int(current_node[1])
        neighbours = [(i+1, j), (i-1, j), (i, j+1), (i, j-1)]

        # g-cost of neighbours
        gcost_neighbour = g_cost_current + 10

        # set current node as parrent
        parent_childs[current_node] = []
        
        # exploring neighbours of the current node
        for neighbour in neighbours:
            
            # out of bounds neighbour
            if not (0 <= neighbour[0] < map_.shape[0] and 0 <= neighbour[1] < map_.shape[1]):
                continue

            # Convert to Python int
            neighbour = (int(neighbour[0]), int(neighbour[1]))

            # not valid neighbour
            if map_[neighbour] == 1 or neighbour in closed_list:
                continue

            if neighbour not in open_dic.keys() or gcost_neighbour < open_dic[neighbour]:
                p, q = neighbour
                hcost = abs(x - p) + abs(y - q)
                fcost = gcost_neighbour + hcost
                open_dic[neighbour] = gcost_neighbour
                heapq.heappush(open_list, (fcost, gcost_neighbour, neighbour))
                parent_childs[current_node].append(neighbour)

    path = get_ancestors(parent_childs)
    path.reverse()
    return path

# this function moves the robot to given coordinates in the map
def go_to(goalx, goaly):
    global linearDist

    # define robot and goal angle
    robotAngle = math.asin(theta)
    dy,dx = (goaly - y), (goalx- x)
    slop = dy/dx

    if dx<0 and dy>0:
        goalAngle = math.pi + math.atan(slop)
    elif dx<0 and dy<0:
        goalAngle = math.atan(slop) - math.pi
    else:
        goalAngle = math.atan(slop)

    # linear and angular distance from goal
    angleDiff = goalAngle - 2*robotAngle
    linearDist = math.sqrt((goalx - x)**2 + (goaly - y)**2)

    # rotate toward aim node
    if abs(angleDiff) > 0.01:
        angular_vel = 1.5*angleDiff
        linear_vel = 0
    else:
        angular_vel = 0
        linear_vel = 2

    return linear_vel, angular_vel


def main():

    global start_node, goal_node

    rospy.init_node('evader', anonymous = False)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/odom", Odometry, callback)
    rospy.sleep(1)

    # goal parameters
    gx = rospy.get_param('~goalx')
    gy = rospy.get_param('~goaly')

    # start node (r, c)
    start_node = round(9.6 - y), round(x + 8.6)
    # goal node (r, c)
    goal_node = round(9.6 - gy), round(gx + 8.6)

    print(start_node, goal_node)

    # list of path nodes in grid (n,m)
    path = aStar(map)  
    print(path)

    # visulaize path
    vis_path(path)

    idx = 1
    atGoal = False
    rate = rospy.Rate(10) 
    while not atGoal:

        
        if path == []:
            print("path not found!")
            break

        if idx > len(path)-1:
            print("at Goal!")
            atGoal = True
            linVel, angVel = 0, 0
            break

        # defining variable goal coordinates
        (r, c) = path[idx]
        node_x, node_y = c-8.5, 9.5-r

        # go to the given path coorinate
        linear_vel, angular_vel = go_to(node_x, node_y)
        if linearDist < 0.1:
            idx += 1

        # define velocity message
        vel.linear.x = linear_vel
        vel.angular.z = angular_vel

        pub.publish(vel)
        rate.sleep()
            



if __name__ == '__main__':

    try:
        main()
    except rospy.ROSInterruptException:
        pass