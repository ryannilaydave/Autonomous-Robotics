from numpy.core.fromnumeric import reshape
from Robot import *
from State import State
from geometry_msgs.msg import Twist
import rospy
import numpy as np
import OccupancyGridMethods as og
from Queue import Empty, PriorityQueue
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time
import math
import heapq

class Frontier(State):
    def __init__(self):
        State.__init__(self, "Frontier", 4)

    # Method to find the world position of the goal pose, and use move_base to 
    # patrol towards that goal
    def patrol_to_goal(self, robot):
        if robot.goal_pose == None:
            
            robot.visualise_occupancy_grid()
            return

        world_point = og.to_world(
            int(robot.goal_pose[0]), 
            int(robot.goal_pose[1]), 
            robot.occupancy_grid.info.origin, 
            robot.map.grid_size, 
            robot.occupancy_grid.info.resolution
        )

        goal = self.goal_pose(world_point)

        while robot.is_frontier:
            robot.move_base.send_goal(goal)
            robot.move_base.wait_for_result(rospy.Duration(15))

            if robot.move_base.get_state() == 4 or  \
                robot.move_base.get_state() == 3:
                
                robot.invalid_frontiers.append(
                    (robot.goal_pose[0], robot.goal_pose[1])
                )

                break

            # if robot.move_base.get_state() == 3:
            #     break
                
            

        robot.goal_pose = None

        # robot.visualise_occupancy_grid()

    # Finds the next frontier that the robot should move to, and sets this pose 
    # as the robots goal_pose
    def execute_state(self, robot):
        robot.is_frontier = True

        if robot.goal_pose == None:
            robot.goal_pose = self.find_next_node(robot)
        
        if robot.goal_pose == None:
            rospy.loginfo("scripts.states.Frontier: Exploration complete in: " +
                str(time.time() - robot.start_time) + " seconds")

            robot.goal_manager.print_all_label_count()
            robot.visualise_occupancy_grid()
            robot.is_random_walk = True

        self.patrol_to_goal(robot)
        
        
    # Method to generate MoveBaseGoal objects
    def goal_pose(self, pose):  
        
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.pose.position.x = pose[0]
        goal_pose.target_pose.pose.position.y = pose[1]
        goal_pose.target_pose.pose.position.z = 0.0
        goal_pose.target_pose.pose.orientation.x = 0.0
        goal_pose.target_pose.pose.orientation.y = 0.0
        goal_pose.target_pose.pose.orientation.z = 0.0
        goal_pose.target_pose.pose.orientation.w = 1.0
        return goal_pose

    # Finds all the frontiers, along with the number of unknown neighbours that 
    # frontier has
    def get_num_neighbours(self, occupancy_grid, robot):
        # Wraps the current grid with 0s
        padded = np.pad(
            occupancy_grid, 
            (1,1), 
            mode='constant', 
            constant_values=0
            )

        padded = np.where(padded == -1, 1, 0)
        k1 = np.where(occupancy_grid <= robot.free_thresh, 1, 0)
        k2 = np.where(occupancy_grid > 0, 1, 0)
        known = k1 * k2

        # How the grid would look if it was shifted in one direction
        shift_right = padded[1:-1,:-2]
        shift_left = padded[1:-1,2:]
        shift_up = padded[2:,1:-1]
        shift_down = padded[:-2,1:-1]
        shift_down_right = padded[:-2,:-2]
        shift_down_left = padded[:-2,2:]
        shift_up_left = padded[2:,2:]
        shift_up_right = padded[2:,:-2]

        # Adds all the shifted positions together, and multiplies by known to 
        # have a grid that only shows the number of unknown neighbours to the
        # known frontiers
        neighbours = known * (shift_right + shift_left + shift_up + shift_down +
            shift_down_right + shift_down_left + shift_up_left + shift_up_right)
        return neighbours

    # Gets the distance between two points
    def get_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)
    
    # Gets the value of the costmap at a specific location
    def get_costmap_val(self, robot, x, y):
        i = og.to_index(x, y, robot.map.grid_size[0])
        return robot.global_cost_map[i]

    # Groups the frontiers so you don't have to check as many frontiers, the 
    # number of frontiers can increase substantially the more you explore the 
    # map
    def group_frontiers(self, frontiers, neighbours, robot, grid):
        group_threshold = robot.map.map_size * robot.map.resolution * (2.0/3.0)
        groups = []

        for (x,y) in frontiers:

            costmap_val = self.get_costmap_val(robot, y, x)
            if costmap_val >= robot.cost_map_thresh:
                continue

            #check boundary box
            surround_arr = self.get_boundary_grid(robot, x, y, grid)
            if 0 < len(np.argwhere(surround_arr >= robot.occupied_thresh)):
                continue
            


            n = neighbours[x,y]
            if groups == []:
                groups.append((x,y,n))
            else:

                close_groups = []

                for _ in groups:
                    (gx, gy, gn) = groups.pop(0)
                    dist = self.get_distance(x,y,gx,gy)

                    if dist < group_threshold:
                        close_groups.append((gx,gy,gn))
                    else:
                        groups.append((gx,gy,gn))

                if close_groups == []:
                    groups.append((x,y,n))
                else:
                    xs = [i for (i,_,_) in close_groups]
                    ys = [i for (_,i,_) in close_groups]
                    ns = [i for (_,_,i) in close_groups]

                    ax = float(sum(xs) + x)/float(len(xs)+1)
                    ay = float(sum(ys) + y)/float(len(ys)+1)
                    an = sum(ns) + n

                    groups.append((ax,ay,an))

        all_groups = [(int(x),int(y)) for (x,y,_) in groups]

        all_neighbours = [n for (_,_,n) in groups]

        return all_groups, all_neighbours


    def get_boundary_grid(self, robot, x, y, grid):
        x_bound_start = (x - robot.boundary_size) if (x - robot.boundary_size) > 0 else 0
        x_bound_end = (x + robot.boundary_size + 1) if (x + robot.boundary_size + 1) < len(grid) else len(grid)

        y_bound_start = (y - robot.boundary_size) if (y - robot.boundary_size) > 0 else 0
        y_bound_end = (y + robot.boundary_size + 1) if (y + robot.boundary_size + 1) < len(grid[0]) else len(grid[0])

        subset_arr = grid[x_bound_start:x_bound_end, y_bound_start:y_bound_end]

        return subset_arr



    # Gets the straight line distance between two points
    def heuristic_cost(self, a, b):
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    # Only consider frontiers that are away from a wall
    def get_clear_frontiers(self, occupancy_grid, frontier_locations, robot):
        # Add a ring of 0s to the occupancy grid
        padded = np.pad(
            occupancy_grid, 
            (1,1), 
            mode='constant', 
            constant_values=0
        )

        # If the cell is occupied set as 0 else 1
        padded = np.where(padded >= robot.occupied_thresh, 0, 1)

        # Shift the occupancy grid in all directions
        shift_right = padded[1:-1,:-2]
        shift_left = padded[1:-1,2:]
        shift_up = padded[2:,1:-1]
        shift_down = padded[:-2,1:-1]
        shift_down_right = padded[:-2,:-2]
        shift_down_left = padded[:-2,2:]
        shift_up_left = padded[2:,2:]
        shift_up_right = padded[2:,:-2]

        # Multiplying the frontiers 0 if any neighbours are obstacles else 1, 
        # causes only frontiers that have no obstacle neighbours to be 
        # considered
        return frontier_locations * shift_right * shift_left * shift_up * \
            shift_down * shift_down_right * shift_down_left * shift_up_left * \
            shift_up_right

    # Gets the straight line between the current location and a frontier, adding
    # weight if the straight line goes through a wall/obstacle
    def get_weighted_distance(self, robot, grid, start, end):
        
        count = 0

        # start = (start[1], start[0])
        end = (end[1], end[0])

        points = robot.get_line(start, end)
        

        for p in points:
            if grid[p[1], p[0]] >= robot.occupied_thresh:
                count += robot.weighted_val
            else:
                count += 1


            i = og.to_index(p[0], p[1], robot.map.grid_size[0])
        
            if robot.global_cost_map[i] >= 100:
                count += robot.global_cost_map[i] * robot.weighted_val
            else:
                count += robot.global_cost_map[i]

        return count

    # Finds the next frontier that the robot should try and traverse to
    def find_next_node(self, robot):
        occupancy_grid = np.array(robot.occupancy_grid.data)
        grid = np.reshape(occupancy_grid, robot.map.grid_size)
        num_neighbours = self.get_num_neighbours(grid, robot)
        num_neighbours = self.get_clear_frontiers(grid, num_neighbours, robot)
        frontier_locations = np.argwhere(num_neighbours > 0)

        frontier_locations, all_neighbours = self.group_frontiers(
            frontier_locations, 
            num_neighbours, 
            robot, 
            grid
        )

        frontier = None
        current_p_cell = None

        # Loop through all of the frontiers to try and find the best one to 
        # traverse to
        for i, (x,y) in enumerate(frontier_locations):
            unknown_neighbours = all_neighbours[i]

            start_grid = og.to_grid(
                robot.current_pose.x, 
                robot.current_pose.y, 
                robot.occupancy_grid.info.origin, 
                robot.map.grid_size, 
                robot.occupancy_grid.info.resolution
            )

            occupancy_grid = np.array(robot.occupancy_grid.data)
            grid = np.reshape(occupancy_grid, robot.map.grid_size)

            distance = self.get_weighted_distance(
                robot, 
                grid, 
                start_grid, 
                (x,y)
            )

            # Gets the p_cell value of the frontier, to consider both the amount
            # of neighbours this frontier will explore, and the distance needed
            # to get to the frontier
            p_cell = float(unknown_neighbours)/float(distance)

            if not frontier or p_cell > current_p_cell:
                
                costmap_val = self.get_costmap_val(robot, y, x)
                if costmap_val < robot.cost_map_thresh:
                    is_valid = True
                    for (fx,fy) in robot.invalid_frontiers:
                        dist = self.get_distance(y, x, fx, fy)
                        if dist < robot.invalid_thresh:
                            is_valid = False
                            break
                    surround_arr = self.get_boundary_grid(robot, x, y, grid)
                    bound =  0 == len(np.argwhere(surround_arr >= robot.occupied_thresh))
                        
                    if is_valid and bound:
                        frontier = (y, x)
                        current_p_cell = p_cell
                    
        #TODO Uncomment for occupancy update
        robot.save_occupancy()
        return frontier
