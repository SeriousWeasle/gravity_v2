import math

class vector2:
    #init vector
    def __init__(self, e0, e1):
        self.e = [e0, e1]
    #vector2 + vector2
    def __add__(self, other):
        return vector2(self.e[0] + other.e[0], self.e[1] + other.e[1])
    #vector2 - vector2
    def __sub__(self, other):
        return vector2(self.e[0] - other.e[0], self.e[1] - other.e[1])
    #vector2 * other
    def __mul__(self, other):
        #vector2 * vector2
        if isinstance(other, vector2):
            return vector2(self.e[0] * other.e[0], self.e[1] * other.e[1])
        #vector2 * num
        else:
            return vector2(self.e[0] * other, self.e[1] * other)
    #vector2 / other
    def __truediv__(self, other):
        #vector2 / vector2
        if isinstance(other, vector2):
            return vector2(self.e[0]/other.e[0], self.e[1]/other.e[1])
        #vector2 / num
        else:
            return vector2(self.e[0]/other, self.e[1]/other)

    def length_squared(self):
        return ((self.e[0] * self.e[0]) +(self.e[1] * self.e[1]))

    def length(self):
        return math.sqrt(self.length_squared())

    def x(self):
        return self.e[0]

    def y(self):
        return self.e[1]

    def clamp(self, length):
        scalar = self.length()/length
        return self / scalar

    def __str__(self):
        return str(self.e[0]) + ", " + str(self.e[1]) 
point2 = vector2