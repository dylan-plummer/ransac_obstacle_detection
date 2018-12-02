class Obstacle:

    def __init__(self, id, x, y, z, diameter):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.diameter = diameter

    def __str__(self):
        print(self.id + ':', self.x, self.y, self.z, self.diameter)
