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

# A start algorithm : O(mn*log(mn)) > returns path
import heapq

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

    p_map = np.pad(map_, pad_width=1, mode='constant', constant_values='O')
    
    global start_node, goal_node

    open_list = []
    open_dic = {}
    closed_list = set()
    parent_childs = {}

    # start node
    s_indices = np.where(p_map=='S')
    # Convert to a tuple (i, j)
    s_node = tuple(zip(s_indices[0], s_indices[1]))[0]
    start_node = int(s_node[0]), int(s_node[1])
    i, j = start_node

    # goal node
    g_indices = np.where(p_map=='G')
    # Convert to a tuple (i, j)
    g_node = tuple(zip(g_indices[0], g_indices[1]))[0]
    goal_node = int(g_node[0]), int(g_node[1])
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
        if current_node == g_node:
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

            # Convert to Python int
            neighbour = (int(neighbour[0]), int(neighbour[1]))

            # not valid neighbour
            if p_map[neighbour] == 'O' or neighbour in closed_list:
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
    rospy.init_node('evader', anonymous = False)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/odom", Odometry, callback)

    # goal parameters
    gx = rospy.get_param('~goalx')
    gy = rospy.get_param('~goaly')


    # start node (n,m)
    start_node = round(x + 8.6), round(9.4 - y)
    # goal node (n,m)
    goal_node = round(gx + 8.6), round(9.4 - gy)

    # list of path nodes in grid (n,m)
    path = aStar(map)  

    idx = 0
    atGoal = False
    rate = rospy.Rate(10) 
    rospy.sleep(1)
    while not atGoal:

        if path == []:
            print("path not found!")
            break

        if linearDist < 0.5:
            print("at Goal!")
            linVel, angVel = 0, 0
            break

        # defining variable goal coordinates
        (n,m) = path[idx]
        idx += 1
        node_x, node_y = n-8.5, 9.5-m

        # go to the given path coorinate
        linVel, angVel = go_to(node_x, node_y)

        # define velocity message
        vel.linear.x = linVel
        vel.angular.x = angVel

        pub.publish(vel)
        rate.sleep()
            



if __name__ == '__main__':

    try:
        main()
    except rospy.ROSInterruptException:
        pass