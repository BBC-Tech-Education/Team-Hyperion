# IMPORTS #
import math
# By Thomas McCabe 2026
# -- VECTOR LIBRARY -- #
class Vector2D:
    '''Vector2D is a data type used to represent a vector in two dimentional space.'''
    def __init__(self, ii: float, jj: float, isPolar: bool = False):
        '''This is a variable type, it has all the propities of a vector.

        :param ii: The i conponent.
        :type ii: float
        :param jj: The j conponent.
        :type jj: float
        :param isPolar: Changes params from i and j --> Arg and Mag.
        :type isPolar: bool
        :returns String/Vector2D: Vect arg, Vect mag.
        '''
        self.i = ii
        '''The i (x) conponent of the vector'''
        self.j = jj
        '''The j (y) conponent of the vector'''
        self.arg = math.degrees(math.atan2(self.j,self.i))
        '''The argument / angle of the vector'''
        self.mag = math.sqrt(ii**2 + jj**2)
        '''The mag / lenght of the vector'''
        pass
    def __str__(self):
        return f"Vector2D: Arg: {self.arg}, Mag: {self.mag}"
        pass
    def __add__(self, other):
        return Vector2D(self.i+other.i, self.j+other.j)
        pass
    def __sub__(self, other):
        return Vector2D(self.i-other.i, self.j-other.j)
        pass
    def __abs__(self):
        return Vector2D(abs(self.i),abs(self.j))
        pass

# -- POINT LIBRARY -- #
class Point2D:
    '''Point2D is a data type that stores a 2D point'''
    def __init__(self, xx: float = 0.0, yy: float = 0.0):
        '''This is a variable type used to store a 2D point.
        
        :param xx: The x position
        :type xx: float
        :param yy: The y position
        :type yy: float
        :returns Point2D/string: x, y
        '''
        self.x = xx
        '''The X position of the point.'''
        self.y = yy
        '''The Y position of the point.'''
        pass
    def __str__(self):
        return f"Point2D: x: {self.x}, y: {self.y}"
        pass
    def __add__(self, other):
        return Point2D(self.x + other.x, self.y + other.y)
        pass
    def __sub__(self, other):
        return Point2D(self.x - other.x, self.y - other.y)
        pass
    def __mul__(self, other):
        if(type(other) == type(self)): return Point2D(self.x * other.x, self.y * other.y)
        else: return Point2D(self.x * other, self.y * other)
        pass
    def __truediv__(self, other):
        if(type(other) == type(self)): return Point2D(self.x / other.x, self.y / other.y)
        else: return Point2D(self.x / other, self.y / other)
        pass
    def __pow__(self, other):
        return (self.x**other + self.y**other)
        pass
    def distance_to_point(self, *other):
        '''Returns the distance to another point2D.
        
        :param point: The point to find the distance to.
        :type point: Point2D, tuple
        :return distance: The distance in the units provided.
        '''
        if(type(other[0]) == type(self)): return math.sqrt((other[0].x-self.x)**2 + (other[0].y-self.y)**2)
        else: return math.sqrt((other[0]-self.x)**2 + (other[1]-self.y)**2)
    def angle_to_point(self, *other):
        '''Returns the angle to another point2D.
        
        :param point: The point to find the angle to.
        :type point: Point2D, tuple
        :return angle: The angle in degrees.'''
        if(type(other[0]) == type(self)): 
            other = other[0]
            return math.degrees(math.atan2(other.y-self.y, other.x-self.x))
        else: return math.degrees(math.atan2(other[1]-self.y, other[0]-self.x))


