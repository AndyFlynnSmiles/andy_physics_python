class Polygon:
    def __init__(self, points, mass=1):
        self.points = points #Must be anti clockwise
        self.transformed_points = None
        self.mass = mass
        self.squared_extent = max(point.x ** 2 + point.y ** 2 for point in points)
        self.area = sum(vec_math.cross(points[i + 1], points[i]) for i in range(len(points) - 1)) / 2
        self.com = Vector2(sum((points[i].x + points[(i + 1) % len(points)].x) * 
                                vec_math.cross(points[(i + 1) % len(points)], points[i]) 
                                for i in range(len(points))) / (6 * self.area),
                           sum((points[i].y + points[(i + 1) % len(points)].y) * 
                                vec_math.cross(points[(i + 1) % len(points)], points[i]) 
                                for i in range(len(points))) / (6 * self.area))
        p = [vec_math.sub(point, self.com) for point in points]
        self.moi = (mass / 12 * 
                    sum((abs(vec_math.cross(p[(i + 1) % len(p)], p[i])) * 
                        (vec_math.dot(p[(i + 1) % len(p)], p[(i + 1) % len(p)]) + 
                        vec_math.dot(p[(i + 1) % len(p)], p[i]) + 
                        vec_math.dot(p[i], p[i]))) 
                        for i in range(len(p))) / self.area)

    def transform_points(self, position, angle, around_com=True):
        if around_com:
            self.transformed_points = [vec_math.rotate_around(point, angle, self.com) for point in self.points]
        else:
            self.transformed_points = [vec_math.rotate(point, angle) for point in self.points]
        self.transformed_points = [vec_math.add(point, position) for point in self.shifted_points]

    def draw(self):
        for i, point in enumerate(self.transformed_points):
            draw_line_v(self.transformed_points[i - 1], point, WHITE)


class Rigidbody:
    def __init__(self, polygon: Polygon, x, y, vel_x=0, vel_y=0, angle=0, angular_velocity=0, 
                restitution=0, friction=0, static=False):
        self.polygon = polygon
        self.position = Vector2(x, y)
        self.velocity = Vector2(vel_x, vel_y)
        self.angle = angle
        self.angular_velocity = angular_velocity
        self.restitution = restitution
        self.friction = friction
        self.static = static

    def update(self, dt):
        vec_math.add_on(self.position, vec_math.scale(self.velocity, dt))
        self.angle += self.angular_velocity * dt
        self.polygon.transform_points(self.position, self.angle)

    def draw(self): #Remember to rotate image around shifted center of mass
        draw_circle_v(self.position, 2, BLUE)
        self.polygon.draw()


class PhysicsManager:
    def __init__(self):
        self.rigidbodys = []
        self.constraints = []

    def update(self, dt):
        for rigidbody in self.rigidbodys:
            rigidbody.update(dt)

    def collide(self, a: Rigidbody, b: Rigidbody):
        pass

    def draw(self):
        #restrict to screen only
        for rigidbody in self.rigidbodys:
            rigidbody.draw()