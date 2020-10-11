import math

class vector3:
    def __init__(self, e1, e2, e3):
        self.e = [e1, e2, e3]
    #vector3 + vector3
    def __add__(self, other):
        return vector3(self.e[0] + other.e[0], self.e[1] + other.e[1], self.e[2] + other.e[2])
    #vector3 - vector3
    def __sub__(self, other):
        return vector3(self.e[0] - other.e[0], self.e[1] - other.e[1], self.e[2] - other.e[2])
    #vector3 * other
    def __mul__(self, other):
        #vector3 * vector3
        if isinstance(other, vector3):
            return vector3(self.e[0] * other.e[0], self.e[1] * other. e[1], self.e[2] * other.e[2])
        #vector3 * num
        else:
            return vector3(self.e[0] * other, self.e[1] * other, self.e[2] * other)
    #vector3 / other
    def __truediv__(self, other):
        #vector3 / vector3
        if isinstance(other, vector3):
            return vector3(self.e[0]/other.e[0], self.e[1]/other.e[1], self.e[2]/other.e[2])
        #vector3 / num
        else:
            return vector3(self.e[0]/other, self.e[1]/other, self.e[2]/other)
    
    def length_squared(self):
        return (self.e[0] * self.e[0]) + (self.e[1] * self.e[1]) + (self.e[2] * self.e[2])

    def length(self):
        return math.sqrt(self.length_squared())

    def x(self):
        return self.e[0]
    
    def y(self):
        return self.e[1]
    
    def z(self):
        return self.e[2]

    def r(self):
        return self.e[0]
    
    def g(self):
        return self.e[1]
    
    def b(self):
        return self.e[2]

    def toint(self):
        return vector3(int(self.e[0]), int(self.e[1]), int(self.e[2]))

point3 = vector3
color = vector3