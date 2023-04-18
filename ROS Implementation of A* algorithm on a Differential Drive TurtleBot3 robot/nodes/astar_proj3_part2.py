# Importing Libraries
import numpy as np
import math
import matplotlib.pyplot as plt
import time
import heapq
from math import dist
import matplotlib.patches as patches

# Importing matplotlib and turning on interactive mode
plt.ion()

# Definition of the Node class that represents a node in a graph
class Node:

    def __init__(self, x, y, parent, current_theta, change_theta, UL, UR, c2c, c2g, total_cost):
        self.x = x
        self.y = y
        self.parent = parent
        self.current_theta = current_theta
        self.change_theta = change_theta
        self.UL = UL
        self.UR = UR
        self.c2c = c2c
        self.c2g = c2g
        self.total_cost = total_cost 
        
    def __lt__(self,other):
        return self.total_cost < other.total_cost

# Checks if the move is valid or not 
def valid_move(x, y, r, c):
    
    # Check if the move is valid or not
    if obstacle_space_check(x, y, r, c):
        return False
    else:
        return True

# Checks if the goal has been reached 
def check_goal(current, goal):

    # Calculate the distance from goal point to current point
    dt = dist((current.x, current.y), (goal.x, goal.y))

    # If distance is less than 0.15 meter, return true
    if dt < 0.15:
        return True
    else:
        return False

### Function to calculate the euclidean distance between two coordinates ###
def dist(pos, goal):
    xp, yp = pos
    xg, yg = goal
    distance = np.sqrt((xp-xg)**2 + (yp-yg)**2)
    return distance

### Function to plot dubins curves ###
def plot_curve(Xi, Yi, Thetai, UL, UR,c, plot, Nodes_List, Path_List):

    # Xi, Yi,Thetai: Input point's coordinates
    # X_start, Y_start: Start point coordinates for plot function
    # X_end, _end, Theta_end: End point coordintes
    
    t = 0
    r = 0.040
    L = 0.160
    dt = 0.1
    cost = 0
    X_end = Xi
    Y_end = Yi
    Theta_end = 3.14 * Thetai / 180

    while t < 1:
        t = t + dt
        X_start = X_end
        Y_start = Y_end
        X_end += r*0.5 * (UL + UR) * math.cos(Theta_end) * dt
        Y_end += r*0.5 * (UL + UR) * math.sin(Theta_end) * dt
        Theta_end += (r / L) * (UR - UL) * dt
        # plt.plot([Xs, Xn], [Ys, Yn], color="blue")
        if  valid_move(X_end, Y_end, r, c):
            if plot == 0:
                # plt.plot([Xs, Xn], [Ys, Yn], color="blue")
                c2g = dist((X_start, Y_start), (X_end, Y_end))
                cost = cost + c2g
                Nodes_List.append((X_end, Y_end))
                Path_List.append((X_start, Y_start))
            if plot == 1:
                plt.plot([X_start, X_end], [Y_start, Y_end], color="red")
        else:
            return None
    Theta_end = 180 * (Theta_end) / 3.14
    return [X_end, Y_end, Theta_end, cost, Nodes_List, Path_List]

# To check if the node is in obstacle
def obstacle_space_check(x, y, radius, clearance):
    
    total_space = radius + clearance # Defining the total buffer space

    obstacle1 = ((np.square(x - 4)) + (np.square(y - 1.1)) <= np.square(0.5 + total_space)) # Circular obstacle     
    obstacle2 = (x >= 1.5 - total_space) and (x <= 1.65 + total_space) and (y >= 0.75 - total_space)   # Rectangle obstacle
    obstacle3 = (x >= 2.5 - total_space) and (x <= 2.65 + total_space) and (y <= 1.25 + total_space)   # Rectangle obstacle
 
    # Border define   
    border1 = (x <= 0 + total_space)     
    border2 = (x >= 5.99 - total_space)
    border3 = (y <= 0 + total_space)
    border4 = (y >= 1.99 - total_space)

    # Check the presence of obstacel and border
    if obstacle1 or obstacle2 or obstacle3 or border1 or border2 or border3 or border4:
        return True
    else:
        return False
        
        
# Generateing unique key
def key(node):
    key = 1000*node.x + 111*node.y 
    return key

# Function to implement A star 
def Astar(start_node, goal_node, rpm1, rpm2, radius, clearance):

    # Check if the goal node is reached 
    if check_goal(start_node, goal_node):
        return 1, None, None
    
    start_node = start_node
    start_node_id = key(start_node)
    goal_node = goal_node

    Nodes_List = []  # List to store all the explored nodes
    Path_List = []  # List to store the final path from start to goal node

    closed_node = {}  # Dictionary to store all the closed nodes
    open_node = {}  # Dictionary to store all the open nodes
    
    open_node[start_node_id] = start_node   # Add the start node to the open nodes dictionary

    priority_list = []  # Priority queue to store nodes based on their total cost
    
    # All the possible moves of the robot
    moves = [[rpm1, 0], 
             [0, rpm1], 
             [rpm1, rpm1], 
             [0, rpm2], 
             [rpm2, 0], 
             [rpm2, rpm2], 
             [rpm1, rpm2],
             [rpm2, rpm1]]

    # Push the start node into the priority queue with its total cost
    heapq.heappush(priority_list, [start_node.total_cost, start_node])

    while (len(priority_list) != 0):

        # Pop the node with the minimum cost from the priority queue
        current_nodes = (heapq.heappop(priority_list))[1]
        current_id = key(current_nodes)

        # Check if the popped node is the goal node
        if check_goal(current_nodes, goal_node):
            goal_node.parent = current_nodes.parent
            goal_node.total_cost = current_nodes.total_cost
            print("Goal Node found")
            return 1, Nodes_List, Path_List
        
        # Add the popped node to the closed nodes dictionary
        if current_id in closed_node:  
            continue
        else:
            closed_node[current_id] = current_nodes
        
        del open_node[current_id]
        
        # Loop through all the possible moves
        for move in moves:
            action = plot_curve(current_nodes.x, current_nodes.y, current_nodes.current_theta, move[0], move[1],
                            clearance, 0, Nodes_List, Path_List)
           
            # Check if the move is valid
            if (action != None):
                angle = action[2]
                
                # Round off the coordinates and the angle to nearest integer
                theta_lim = 15
                x = (round(action[0] * 10) / 10)
                y = (round(action[1] * 10) / 10)
                theta = (round(angle / theta_lim) * theta_lim)
                
                # Calculate the new orientation and the cost to move to the new node
                current_theta = current_nodes.change_theta - theta
                c2g = dist((x,y), (goal_node.x, goal_node.y))
                new_node = Node(x, y, current_nodes, theta, current_theta, move[0], move[1], current_nodes.c2c+action[3], c2g, current_nodes.c2c+action[3]+c2g)

                new_node_id = key(new_node)
                
                # Check if the new node is valid and has not already been visited
                if not valid_move(new_node.x, new_node.y, radius, clearance):
                    continue
                elif new_node_id in closed_node:
                    continue

                # Update the node information if it already exists in the open list
                if new_node_id in open_node:
                    if new_node.total_cost < open_node[new_node_id].total_cost:
                        open_node[new_node_id].total_cost = new_node.total_cost
                        open_node[new_node_id].parent = new_node

                # Add the new node to the open list if it doesn't already exist        
                else:
                    open_node[new_node_id] = new_node
                    heapq.heappush(priority_list, [ open_node[new_node_id].total_cost, open_node[new_node_id]])
            
    return 0, Nodes_List, Path_List


# Generating the path 
def back_track(goal_node):  

    x_path = []
    y_path = []
    theta_path = []
    RPM_Left_Wheel = []
    RPM_Right_Wheel = []
    x_path.append(goal_node.x)
    y_path.append(goal_node.y)
    theta_path.append(goal_node.current_theta)
    RPM_Left_Wheel.append(goal_node.UL)
    RPM_Right_Wheel.append(goal_node.UR)
    parent_node = goal_node.parent

    while parent_node != -1:
        x_path.append(parent_node.x)
        y_path.append(parent_node.y)
        theta_path.append(parent_node.current_theta)
        RPM_Left_Wheel.append(parent_node.UL)
        RPM_Right_Wheel.append(parent_node.UR)
        parent_node = parent_node.parent
        
    x_path.reverse()
    y_path.reverse()
    theta_path.reverse()
    RPM_Left_Wheel.reverse()
    RPM_Right_Wheel.reverse()

    RPM_Left = np.array(RPM_Left_Wheel)
    RPM_Right = np.array(RPM_Right_Wheel)
    x = np.asarray(x_path)
    y = np.asarray(y_path)
    theta = np.array(theta_path)
    return x,y,theta,RPM_Left,RPM_Right

### To check is orientation is valid ####
def validorient(theta):
    if((theta%30)==0):
        return theta
    else:
        return False


if __name__ == '__main__':

    width = 6
    height = 2
    robot_radius  = 0.033
    clearance = input("Enter obstacle clearance for robot ")
    clearance = float(clearance)

    Rpms = input("Enter left wheel and right wheel RPMs")
    RPM1,RPM2 = Rpms.split()
    RPM1 = int(RPM1)
    RPM2 = int(RPM2)

    start_coordinates = input("Enter start coordinates: ")
    start_x, start_y = start_coordinates.split()
    # Start point stored as float (points in meter)
    start_x = float(start_x)
    start_y = float(start_y)
    
    goal_coordinates = input("Enter goal coordinates: ")
    goal_x, goal_y = goal_coordinates.split()
    # Goal point stored as float (points in meter)
    goal_x = float(goal_x)
    goal_y = float(goal_y)
	
# Check if the start and goal nodes are in valid positions
    if not valid_move(start_x, start_y, robot_radius, clearance):
        print("In valid start node or in Obstacle space")
        exit(-1)
        
    if not valid_move(goal_x, goal_y, robot_radius, clearance):
        print("In valid goal node or in Obstacle space")
        exit(-1)
    
    start_theta = input("Enter Orientation of the robot at start node: ")
    start_theta = int(start_theta)
    
    if not validorient(start_theta):
        print("Orientation has to be a multiple of 30")
        exit(-1)

    
    
    timer_start = time.time()

    # Compute the distance from start node to goal node
    c2g = dist((start_x,start_y), (goal_x, goal_y))
    total_cost =  c2g
    
    # Create start node and goal node
    start_node = Node(start_x, start_y,-1, start_theta, 0, 0, 0, 0, c2g, total_cost)
    goal_node = Node(goal_x, goal_y, -1, 0, 0, 0, 0, c2g, 0, total_cost)

 # Use the A* algorithm to find the optimal path
    flag, Nodes_List, Path_List = Astar(start_node, goal_node,RPM1,RPM2,robot_radius,clearance)
                    
    if (flag)==1:
        x_path,y_path,theta_path,RPM_Left,RPM_Right = back_track(goal_node)
    else:
        print("Path not found")
        exit(-1)

    # plot(start_node,goal_node,x_path,y_path,Nodes_List,Path_List,RPM1,RPM2,theta_path)
    figure, axes = plt.subplots()
    axes.set(xlim=(0, 6), ylim=(0,2))
    

    # Define the obstacles as circles and rectangles
    obstacle1 = plt.Circle((4, 1.1), 0.5, fill = 'True', color ='green')    
    obstacle2 = patches.Rectangle((1.5, 0.75), 0.15, 1.25, color ='green')
    obstacle3 = patches.Rectangle((2.5, 0), 0.15, 1.25, color ='green')


   # Add the obstacles to the plot
    axes.set_aspect( 'equal' )
    axes.add_artist(obstacle1)
    axes.add_artist(obstacle2)
    axes.add_patch(obstacle3)

    plt.plot(start_node.x, start_node.y, "Dw")
    plt.plot(goal_node.x, goal_node.y, "Dg")

    l = 0
    for l in range(len(Nodes_List)):
        plt.plot([Path_List[l][0], Nodes_List[l][0]], [Path_List[l][1], Nodes_List[l][1]], color="yellow")
        l+=1

    plt.plot(x_path,y_path, ':r')
    timer_stop = time.time()
    
    Count_time = timer_stop - timer_start
    print("The Total Runtime is:  ", Count_time)

    plt.show()
    plt.pause(100)
    plt.close('all')
    
    print("publishing node initialising")
    print (len(x_path))
    print (theta_path)
    print (RPM_Left)
    print (RPM_Right)
