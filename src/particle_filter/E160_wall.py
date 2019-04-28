class E160_wall:

    
    def __init__(self, wall_points, slope):
        
        # set up walls
        self.slope = slope
        self.radius = 0.025
        
        # assume top point is first
        if slope == "vertical":
            self.points = [wall_points[0]-self.radius, wall_points[1]+self.radius,
                    wall_points[0]+self.radius, wall_points[1]+self.radius,
                    wall_points[2]+self.radius, wall_points[3]-self.radius,
                    wall_points[2]-self.radius, wall_points[3]-self.radius]
        
        # assume left point is first
        elif slope == "horizontal":
            self.points = [wall_points[0]-self.radius, wall_points[1]-self.radius,
                    wall_points[0]-self.radius, wall_points[1]+self.radius,
                    wall_points[2]+self.radius, wall_points[3]+self.radius,
                    wall_points[2]+self.radius, wall_points[3]-self.radius]
        
        
