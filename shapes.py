from enum import Enum

from pyray import *
import vec_math


class ShapeType(Enum):
    NO_TYPE = 0
    POLYGON = 1


class Shape:
    def __init__(self):
        self.shape_type = ShapeType.NO_TYPE
        self.area = 0
        self.center_of_mass = Vector2(0, 0)
        self.com_overide = False
        self.moment_of_inertia = 1

    def find_furthest_point(self, direction):
        pass #Not Implemented error

    def transform(self, position, rotation, around = None, ):
        pass #Not Implemented error

    def draw(self):
        pass #Not Implemented error

    def get_center(self):
        pass #Not Implemented error

    def get_center_of_mass(self):
        pass #Not Implemented error


class Polygon(Shape):
    def __init__(self, points: list[Vector2]):
        self.shape_type = ShapeType.POLYGON
        self.area = abs(sum(vec_math.cross(points[i - 1], points[i]) for i in range(len(points))) / 2)

        self.points = points
        self.transformed_points = points

        self.update_center_of_mass()

        self.update_moment_of_inertia()
        
    def find_furthest_point(self, direction: Vector2):
        direction = vec_math.norm(direction)
        distances = [vec_math.dot(point, direction) for point in self.transformed_points]
        furthest_distance = max(distances)
        furthest_point_index = distances.index(furthest_distance)
        furthest_point = self.transformed_points[furthest_point_index]
        return furthest_point

    def transform(self, position: Vector2, rotation: float, around: Vector2 = None):
        if around is None:
            around = position

        self.transformed_points = [vec_math.add(point, position) for point in self.points]
        self.transformed_points = [vec_math.rotate_around(point, around, rotation) for point in self.transformed_points]

    def draw(self):
        for i in range(len(self.transformed_points)):
            draw_line_v(self.transformed_points[i - 1], self.transformed_points[i], WHITE)

    def get_center(self):
        return vec_math.center(self.transformed_points)

    def set_center_of_mass(self, center_of_mass):
        self.com_overide = True
        self.center_of_mass = center_of_mass
        self.update_moment_of_inertia()

    def update_center_of_mass(self):
        self.center_of_mass = Vector2(  sum((self.points[i].x + self.points[(i + 1) % len(self.points)].x) * 
                                            vec_math.cross(self.points[(i + 1) % len(self.points)], self.points[i]) 
                                            for i in range(len(self.points))) / (6 * self.area),
                                        sum((self.points[i].y + self.points[(i + 1) % len(self.points)].y) * 
                                            vec_math.cross(self.points[(i + 1) % len(self.points)], self.points[i]) 
                                            for i in range(len(self.points))) / (6 * self.area))
        
    def update_moment_of_inertia(self):
        p = [vec_math.sub(point, self.center_of_mass) for point in self.points]
        self.moment_of_inertia = abs(1 / 12 * 
                    sum((abs(vec_math.cross(p[(i + 1) % len(p)], p[i])) * 
                        (vec_math.dot(p[(i + 1) % len(p)], p[(i + 1) % len(p)]) + 
                        vec_math.dot(p[(i + 1) % len(p)], p[i]) + 
                        vec_math.dot(p[i], p[i]))) 
                        for i in range(len(p))) / self.area)