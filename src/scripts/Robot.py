import math
import rospy
import message_filters
from Pose import *
import numpy as np
import matplotlib.pyplot as plt
import tf
import actionlib
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionResult
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import OccupancyGridMethods as og
from matplotlib import colors
from Map import *
from StateManager import *
from GoalManager import *
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from darknet_ros_msgs.msg import BoundingBoxes
from Goal import *

# Robot class to control all information and behaviour of the robot
class Robot:
    def __init__(self):

        #+++++++++++++++DO NOT REMOVE THIS LINE+++++++++++++++++
        rospy.init_node('robot')
        #+++++++++++++++++++++++++++++++++++++++++++++++++++++++

        #Pose history
        self.original_pose = None
        self.current_pose = None

        #Initialisers of objects
        self.state_manager = StateManager(self)
        self.goal_manager = GoalManager()

        # Publishers
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.map = Map()
        self.occupancy_grid = self.initialise_occupancy()
        self.boundary_size = 2
        self.cost_map_thresh = 70
        self.invalid_frontiers = []
        self.invalid_thresh = 15.0
        self.weighted_val = 1000
        self.frontier_time_limit = 60
        self.min_box_size = 20

        self.visited_val = 0.05
        self.occupied_thresh = 0.65
        self.free_thresh = 0.196
        self.max_occ = 0.98

        self.global_cost_map = []
        self.cost_map = rospy.Subscriber(
            '/move_base/global_costmap/costmap', 
            OccupancyGrid, 
            self.cost_map_callback)

        # Yolo info
        self.yolo_count_limit = 10 #20
        self.yolo_angular_thresh = 0.03
        self.yolo_dist_thresh = 0.35 #0.35
        
        # Subscribers
        self.original_pose = None
        self.current_pose = None
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # YOLO subs
        odom_sub = message_filters.Subscriber('/odom', Odometry)

        depth_sub = message_filters.Subscriber(
            "/camera/depth/points", 
            PointCloud2)

        bounding_sub = message_filters.Subscriber(
            '/darknet_ros/bounding_boxes',
            BoundingBoxes)

        # Synchronise to single callback
        ts = message_filters.ApproximateTimeSynchronizer(
            [odom_sub, depth_sub, bounding_sub], 
            1, 
            1)
        ts.registerCallback(self.yolo_callback)

        # Global variables
        self.sensor_offset = 0.07
        self.goal_manager = GoalManager()

        # In format: self.yolo_Values = 
        # [(callback_count,point_count,
        #  (total_centre.x, total_centre.y),
        #  (total_surface.x,total_surface.y))]
        self.yolo_values = []
        self.previous_boxes = None
        
        #Initialise robot data
        # States
        self.is_blocked_left = False
        self.is_blocked_right = False
        self.is_blocked = False
        self.is_frontier = False
        self.is_random_walk = False
        self.sees_green = False
        
        # Movement
        self.x_vel = 0.2
        self.base_ang_vel = math.pi/8
        self.limit = math.pi / self.base_ang_vel
        self.callback_returned = False
        self.distance = 3
        self.min_distance = 0.5
        self.angular = None

        # Other info
        self.goal_pose = None
        self.move_base = actionlib.SimpleActionClient(
            'move_base',
            MoveBaseAction)  

        self.move_base.wait_for_server()
        self.move_base_result = rospy.Subscriber(
            '/move_base/status', 
            MoveBaseActionResult)

        self.move_base_status = 0
        
        self.cmap = colors.ListedColormap(['black', 'yellow', 'green', 'red'])
        self.bounds=[-2,-.99,0.051,0.197,0.66]
        self.norm = colors.BoundaryNorm(self.bounds, self.cmap.N)

        # Makes sure the bot only turns up to pi rad (180 deg)
        self.limit = math.pi / self.base_ang_vel

        self.start_time = time.time()
        
    '''Callbacks'''
    def odom_callback(self, msg):
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (_, _, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        yaw = yaw if yaw <= math.pi and yaw >= 0 else 2*math.pi + yaw

        self.current_pose = Pose(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw)
        
        self.update_visited_cell() 

        self.angular = abs(msg.twist.twist.angular.z)

    # Function : Synchronised callback to perform object pose estimation and 
    # save newly detected goals
    def yolo_callback(self, odom, depth, box):

        self.odom_callback(odom)
        # Read in pointcloud data
        point_list = list(point_cloud2.read_points(
            depth,
            field_names=("x", "y", "z"), 
            skip_nans=False))

        # Reset Yolo callback reset if no bounding boxes are found or number of 
        # boxes has changed
        if len(box.bounding_boxes) == 0 or \
            (self.previous_boxes != None and len(box.bounding_boxes) < \
                len(self.previous_boxes)):

            self.yolo_values = []

        # For all boxes in frame
        for box_index in range(0, len(box.bounding_boxes)):
            # Ensure that the lists contain an entry for this bounding box
            if len(self.yolo_values) <= box_index :
                # self.add_blank_callback_value(self.yolo_values)
                self.yolo_values.append((0, 0, [0,0], [0,0]))


            
            # Check object instance
            self.do_bounding_box_iteration(
                self.current_pose,
                self.angular,
                depth,
                point_list,
                box.bounding_boxes[box_index],
                box_index)



        # Store previous boxes so can compare
        self.previous_boxes = box.bounding_boxes

    # Function : Check object instance for new goal
    def do_bounding_box_iteration(
        self,pose,angular,depth,point_list,box,box_index):
        
        (callback_count, point_count, sum_centre, sum_surface) = (
            self.yolo_values[box_index][0],
            self.yolo_values[box_index][1], 
            self.yolo_values[box_index][2], 
            self.yolo_values[box_index][3])

        # Count how many points are calculated - perform this x amount of times 
        # before we calculate the overall value
        if(callback_count <= self.yolo_count_limit):
            # Get centroid of bounding box
            i = (box.xmin + box.xmax)/2
            j = (box.ymin + box.ymax)/2

            # Index into point cloud
            index = self.index_pixel(i, j, depth.width)

            # Check within range and not mid-rotation
            if str(point_list[index][0]) != "nan" and \
                str(point_list[index][2]) != "nan" and \
                    angular<self.yolo_angular_thresh and \
                        (box.xmax - box.xmin) > self.min_box_size:
                

                # Get object centre pose
                object_centre = self.get_object_pose(
                    pose, 
                    box.Class, 
                    point_list[index][0], 
                    point_list[index][2], 
                    True)

                object_surface = self.get_object_pose(
                    pose, 
                    box.Class,
                    point_list[index][0], 
                    point_list[index][2], 
                    False)

                #Increase total pos for this box index
                sum_centre = (
                    object_centre[0] + sum_centre[0], 
                    object_centre[1] + sum_centre[1])

                sum_surface = (
                    object_surface[0] + sum_surface[0], 
                    object_surface[1] + sum_surface[1])

                #Increment total number of points calculated
                point_count += 1
                #Increment callback count
                callback_count += 1

                self.yolo_values[box_index] = (
                    callback_count, 
                    point_count, 
                    sum_centre, 
                    sum_surface)
        else: 
            if(point_count > 0): 
                avg_centre_position = self.get_average_position(
                    self.yolo_values[box_index][2],
                    self.yolo_values[box_index][1])

                avg_surface_position = self.get_average_position(
                    self.yolo_values[box_index][3],
                    self.yolo_values[box_index][1])

                # Check if goal previously seen
                goal_already_used = self.goal_manager.check_existing_goals(
                    self.get_object_radius(box.Class), 
                    avg_centre_position)

                # If not previously seen, add to goal manager
                if not goal_already_used:
                    # Add goal to active goal list 
                    i = (box.xmin + box.xmax)/2
                    j = (box.ymin + box.ymax)/2
                    index = self.index_pixel(i, j, depth.width)

                    new_goal = Goal(
                        str(box.Class),
                        avg_centre_position, 
                        avg_surface_position)

                    self.is_frontier = False
                    self.move_base.cancel_all_goals()
                    self.goal_manager.add_goal(new_goal)
                    rospy.loginfo("scripts.Robot: " + 
                        new_goal.get_formatted_name())

                    #Set active goal to new one if not defined
                    if(self.goal_manager.active_goal == None):
                        self.goal_manager.active_goal = new_goal
            
            #Reset box index value
            # self.yolo_values[box_index] = ((0, 0, [0,0], [0,0]))
            self.yolo_values = []

    # Function : Use pointcloud distances to calculate world position
    def get_object_pose(
        self, pose, object_label, x_relative, z_relative, centre_flag):

        # Robot/Sensor offset
        z_relative += self.sensor_offset

        # If calculating centre (not surface)
        if centre_flag :
            z_relative += self.get_object_radius(object_label)
        else: 
            z_relative -= self.yolo_dist_thresh

        # Calculate angle to determine quadrant
        relative_angle = -(math.atan(x_relative/z_relative))
        detection_angle = (pose.yaw + relative_angle) % (2*math.pi)

        # Get angle and associated factors
        if detection_angle < math.pi/2:
            angle = detection_angle
            sign = (1,1)
        elif detection_angle < math.pi:
            angle = math.pi - detection_angle
            sign = (-1,1)
        elif detection_angle < 1.5*math.pi:
            angle = detection_angle - math.pi
            sign = (-1,-1)
        else:
            angle = 2*math.pi - detection_angle
            sign = (1,-1)

        # Trig calculation to get x,y
        h = math.sqrt(x_relative**2 + z_relative**2)
        x = pose.x + (h*math.cos(angle) * sign[0])
        y = pose.y + (h*math.sin(angle) * sign[1])
        
        return (x,y,object_label)

    # Function : Get average position based (tuple)
    def get_average_position(self, l, point_count):
        x_val = l[0]
        y_val = l[1]
        return (x_val/point_count,y_val/point_count)

    # Function : Append blank counts for box
    def add_blank_callback_value(self, l):
        l.append((0, 0, [0,0], [0,0]))

    # Function : Get index of 2d array ## Note : this is the same as toIndex 
    # from minitask 4 and should be ported over ##
    def index_pixel(self, px, py, width):
        return px + (py * width)

    # Function : Return radius of goal objects
    def get_object_radius(self, object_label):
        if object_label == FIRE_HYDRANT_LABEL:
            return 0.4/2
        elif object_label == GREEN_BOX_LABEL:
            return 0.5/2
        elif object_label == BLUE_MAILBOX_LABEL:
            return 0.667/2
        else:
            rospy.loginfo("scripts.Robot: ERROR IN CLASS DETECTION")


    # Scan callback that uses the current position of the turtlebot to determine
    # the location of obstacles detected by the Laser scanner
    def scan_callback(self, msg):

        while not self.current_pose:
            continue

        increment = msg.angle_increment
        ranges = msg.ranges     
        current_pose = self.current_pose

        for i in range(len(ranges)):
            sensor_angle = increment * i
            theta = current_pose.yaw
            r = ranges[i]
            
            angle, x_sign, y_sign = self.calculate_angle(sensor_angle, theta) 

            if r == float('inf'):
                y = msg.range_max*math.sin(angle) * y_sign
                x = msg.range_max*math.cos(angle) * x_sign
            else:
                y = r*math.sin(angle) * y_sign
                x = r*math.cos(angle) * x_sign
            
            object_pose_x = current_pose.x + x
            object_pose_y = current_pose.y + y

            to_grid_result = og.to_grid(
                object_pose_x, 
                object_pose_y, 
                self.occupancy_grid.info.origin, 
                (self.map.map_size, self.map.map_size), 
                self.occupancy_grid.info.resolution)

            to_bot_grid_result = og.to_grid(
                current_pose.x, current_pose.y, 
                self.occupancy_grid.info.origin, 
                (self.map.map_size, self.map.map_size), 
                self.occupancy_grid.info.resolution)
            
            if(to_grid_result != None and to_bot_grid_result != None):
                grid_x, grid_y = to_grid_result
                bot_grid_x, bot_grid_y = to_bot_grid_result

                free_spaces = self.get_line(
                    (bot_grid_x, bot_grid_y), 
                    (grid_x, grid_y))

                for j in range(len(free_spaces)-1):
                    x_2d = free_spaces[j][0]
                    y_2d = free_spaces[j][1]
                    free_1d = og.to_index(x_2d, y_2d, self.map.map_size)

                    if self.occupancy_grid.data[free_1d] != self.visited_val \
                        and self.occupancy_grid.data[free_1d] <= \
                            self.occupied_thresh:

                        self.occupancy_grid.data[free_1d] = self.free_thresh
                        
                    
                index_1d = og.to_index(grid_x, grid_y, self.map.map_size)

                range_prob = ((msg.range_max - r)/msg.range_max)

                if sensor_angle > math.pi:
                    sensor_angle = math.pi - sensor_angle
                
                angle_prob = ((math.pi - sensor_angle)/(math.pi))

                occ_prob = ((range_prob + angle_prob)/2) * self.max_occ

                if occ_prob > self.occupancy_grid.data[index_1d]:
                    self.occupancy_grid.data[index_1d] = occ_prob

        #Increase to make frontal cone larger on left side
        front_cone_size = 20
        #Make frontal cone mirrored
        front_right_lim = 360 - front_cone_size

        nothing_left = True
        nothing_right = True

        left_count = 0
        right_count = 0
        #Check if ranges are in min distance, if so then blocking path
        for i in range(0, front_cone_size):
            if msg.ranges[i] < self.min_distance:
                self.is_blocked = True
                nothing_left = False
                left_count += 1
        
        for i in range(front_right_lim, len(msg.ranges)):
            if msg.ranges[i] < self.min_distance:
                self.is_blocked = True
                nothing_right = False
                right_count += 1

        #Check if more obstacles on lef than right
        if(left_count >= right_count):
            self.is_blocked_left = True
        else:
            self.is_blocked_left = False
            
        if nothing_left and nothing_right:
            #Observe obstacles in frontal cone of robot
            self.is_blocked = False

        self.callback_returned = True
    

    def cost_map_callback(self, msg):
        self.global_cost_map = msg.data
        

    '''Auxiliary Methods'''

    # Method to visualise the occupancy grid
    def visualise_occupancy_grid(self):
        arr = np.array(self.occupancy_grid.data)
        arr_2d = np.reshape(arr, (self.map.grid_size[0], self.map.grid_size[1]))
        img = plt.imshow(arr_2d, cmap=self.cmap, norm=self.norm, origin='lower')
        plt.show()

    def save_occupancy(self):
        arr = np.array(self.occupancy_grid.data)
        arr_2d = np.reshape(arr, (self.map.grid_size[0], self.map.grid_size[1]))
        img = plt.imshow(arr_2d, cmap=self.cmap, norm=self.norm, origin='lower')
        plt.title('Occupancy Grid at time = ' + str(time.time() - self.start_time) + ' seconds')
        plt.savefig('occupancy_grid.png')

    # Gets the distance between two Poses
    def get_distance(self, a, b):
        return math.sqrt((a.x - b.x)**2 + (a.y-b.y)**2)

    # Initialises the occupancy grid to have values of -1, and sets the 
    # resolution and origin
    def initialise_occupancy(self):
        grid = OccupancyGrid()
        grid.data = [-1 for _ in range(self.map.map_size * self.map.map_size)]
        grid.info.resolution = self.map.resolution
        grid.info.origin = self.map.origin
        return grid

    # Calculates the angle relative to the whole world
    # Also need to return the signs to get the distance in the correct 
    # direction:
    #               |
    #  (x:-1, y:1)  | (x:1, y:1)
    #               |           
    # ----------------------------
    #               |
    #  (x:-1, y:-1) | (x:1, y:-1)
    #               |           
    def calculate_angle(self, sensor_angle, theta):
        detection_angle = (theta + sensor_angle) % (2*math.pi)

        if detection_angle < math.pi/2:
            angle = detection_angle
            x_sign = 1
            y_sign = 1
        elif detection_angle < math.pi:
            angle = math.pi - detection_angle
            x_sign = -1
            y_sign = 1
        elif detection_angle < 1.5*math.pi:
            angle = detection_angle - math.pi
            x_sign = -1
            y_sign = -1
        else:
            angle = 2*math.pi - detection_angle
            x_sign = 1
            y_sign = -1


        return angle, x_sign, y_sign

    '''https://newbedev.com/
    python-bresenhams-line-drawing-algorithm-python-code-example'''
    def get_line(self, start, end):
        """Bresenham's Line Algorithm
        Produces a list of tuples from start and end
    
        >>> points1 = get_line((0, 0), (3, 4))
        >>> points2 = get_line((3, 4), (0, 0))
        >>> assert(set(points1) == set(points2))
        >>> print points1
        [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
        >>> print points2
        [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
        """
        # Setup initial conditions
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1
    
        # Determine how steep the line is
        is_steep = abs(dy) > abs(dx)
    
        # Rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
    
        # Swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
    
        # Recalculate differentials
        dx = x2 - x1
        dy = y2 - y1
    
        # Calculate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1
    
        # Iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
    
        # Reverse the list if the coordinates were swapped
        if swapped:
            points.reverse()
        return points

    # Shows where the turtlebot has moved 
    def update_visited_cell(self):  
        
        #Ensure that result does not return None
        to_grid_result = og.to_grid(
            self.current_pose.x, 
            self.current_pose.y, 
            self.map.origin, 
            self.map.grid_size, 
            self.map.resolution)

        if(to_grid_result != None):
            (grid_x, grid_y) = to_grid_result

            # Obtain 1d array index from grid data
            index_val = int(og.to_index(grid_x, grid_y, self.map.grid_size[0]))

            # Ensure that occupancy grid is initialised before adding index
            try:
                if(self.occupancy_grid):
                    # Set index to related visited value so we know the cell has 
                    # been visited
                    self.occupancy_grid.data[index_val] = self.visited_val
                else:
                    raise Exception("scripts.Robot: Occupancy grid not defined \
                        when attempting to updated visited cell")
            except Exception as e:
                rospy.loginfo(str(e))

    def robot_control(self):
        self.state_manager.set_state("Obstacle Avoidance")

        while True:
            if self.is_blocked:
                self.state_manager.set_state(OBSTACLE_AVOIDANCE_STATE)
            elif self.goal_manager.active_goal_list != []:
                self.state_manager.set_state(BEACON_STATE)
            else:
                self.state_manager.set_state(FRONTIER_STATE)
                
                if self.is_random_walk:
                    self.state_manager.set_state(RANDOM_WALK_STATE)
                    self.is_random_walk = False
