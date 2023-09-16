import rospy

# Converts the world coordinates into grid coordinates
def to_grid(px, py, origin, size, resolution):

    try:
         #Check that entered values are numeric type
        if((type(px) != int and type(px) != float) or \
            (type(py) != int and type(py) != float)):

            raise ValueError("World points provided using non-(int/float) \
                type: type(px) = " + str(type(px)) + ", type(py) = " + 
                str(type(py)))

        #Calculate min max boundaries for grid
        x_max = origin[0] + (resolution * size[0])
        y_max = origin[1] + (resolution * size[1])

        #Error check grid bounds
        if(px > x_max or py > y_max):

            raise ValueError("World points provided exceed maximum bounds: \
                px = " + str(px) + ", py = " + str(py) + " - x_max = " + 
                str(x_max) + ", y_max = " + str(y_max))
        
        origin_x = origin[0]
        origin_y = origin[1]

        grid_x = int((px - origin_x)/resolution)
        grid_y = int((py - origin_y)/resolution)

        if(grid_x < 0 or grid_y < 0):

            raise ValueError("Grid points generated in to_grid are negative: \
                gx = " + str(grid_x) + ", gy = " + str(grid_y))
        
        if(grid_x >= size[0] or grid_y >= size[1]):

            raise ValueError("Calculated grid positions exceed the max \
                boundaries of the grid: gx = " + str(grid_x) + ", gy = " + 
                str(grid_y) + " - size x = " + str(size[0]) + ", size y = " + 
                str(size[1]))

        return (grid_x, grid_y)

    except Exception as e:
        rospy.loginfo("scripts.OccupancyGridMethods.to_grid: " + str(e))
        return None
    
# Converts grid coordinates into world coordinates
def to_world(gx, gy, origin, size, resolution):
    try:    
        #Check that entered grid indices are int
        if(type(gx) != int or type(gy) != int):
            raise ValueError("Grid points provided are not integers: type(gx)\
                 = " + str(type(gx)) +  ", type(gy) = " + str(type(gy)))
        
        #Check that entered grid values are within the bounds of the grid
        if(gx < 0 or gy < 0 or gx >= size[0] or gy >= size[1]):
            raise ValueError("Grid points provided are not within grid \
                bounds/negative: gx = " + str(gx) + ", gy = " + str(gy) + 
                " - x bounds = (0," + str(size[0]) + "), y bounds = (0" + ", " +
                str(size[1]) + ")")
        
        origin_x = origin[0]
        origin_y = origin[1]

        #add resolution/2 to get centre, making sure it is converted to float
        world_x = ((gx  * resolution) + origin_x)  + resolution/2.0
        world_y = ((gy  * resolution) + origin_y) + resolution/2.0

        return (world_x, world_y)
    
    except Exception as e:
        rospy.loginfo("scripts.OccupancyGridMethods.to_world: " + str(e))
        return None

def to_index(gx, gy, size_x):
    return gy * size_x + gx
