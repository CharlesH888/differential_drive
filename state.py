class State():
    def __init__(self, x, y, theta):
        if not (x is None or y is None or theta is None):
            self.x = x
            self.y = y
            self.theta = theta
        else:
            self.x = 0
            self.y = 0
            self.theta = 0

    def __str__(self):
        return str(self.x)+","+str(self.y)+","+str(self.theta)