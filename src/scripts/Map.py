# Data class to store information about the map
class Map:
    def __init__(self):
        self.map_size = 384
        self.resolution = 0.05
        self.origin = (-10.0, -10.0)
        self.grid_size = (self.map_size, self.map_size)
