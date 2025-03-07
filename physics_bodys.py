import global_varibles

from pyray import *

import vec_math

from shapes import Shape


class PhysicsBody:
    def __init__(self):
        self.id = 0
        self.awake = True

        self.position = Vector2(0, 0)
        self.velocity = Vector2(0, 0)
        self.force = Vector2(0, 0)

        self.rotation = 0
        self.rotational_velocity = 0
        self.torque = 0
        
        self.shape = None
        self.gravity = Vector2(0, 0)
        self.mass = 1
        self.center_of_mass = Vector2(0, 0)

    def translate(self, offset: Vector2):
        self.position = vec_math.add(self.position, offset)

    def apply_velocity(self, velocity: Vector2):
        self.velocity = vec_math.add(self.velocity, velocity)

    def apply_force_at_point(self, force: Vector2, point: Vector2):
        if self.center_of_mass is None:
            com = self.shape.center_of_mass
        else:
            com = self.center_of_mass

        r_a = vec_math.sub(point, vec_math.add(self.position, vec_math.rotate(com, self.rotation)))
        xrn = vec_math.cross(r_a, vec_math.norm(force))

        self.velocity = vec_math.add(self.velocity, vec_math.scale(force, 1 / self.mass))

        if self.shape is None:
            moi = self.mass
        else:
            moi = self.shape.moment_of_inertia * self.mass

        self.rotational_velocity += vec_math.length(force) * xrn * vec_math.RAD2DEG * (1 / moi)

    def update(self):
        pass

    def draw(self):
        pass


class RigidBody(PhysicsBody):
    def __init__(self, position: Vector2, velocity: Vector2 = Vector2(0, 0), rotation = 0, rotational_velocity = 0, shape: Shape = None, gravity: Vector2 = Vector2(0, 98.1), physics_manager = None, mass = 1, center_of_mass = None):
        self.awake = True
        
        if physics_manager is None:
            physics_manager = global_varibles.PHYSICS_MANAGER
        physics_manager.add_physics_body(self)
        self.id = physics_manager.max_id
        physics_manager.max_id += 1
        self.physics_manager = physics_manager

        self.position = position
        self.velocity = velocity
        self.force = Vector2(0, 0)

        self.rotation = rotation
        self.rotational_velocity = rotational_velocity
        self.torque = 0

        self.shape = shape
        self.gravity = gravity
        self.mass = mass

        self.center_of_mass = center_of_mass

    def update(self):
        self.velocity = vec_math.add(self.velocity, vec_math.scale(self.gravity, get_frame_time()))
        self.position = vec_math.add(self.position, vec_math.scale(self.velocity, get_frame_time()))

        self.rotation += self.rotational_velocity * get_frame_time()

        if self.center_of_mass is None:
            com = self.shape.center_of_mass
        else:
            com = self.center_of_mass

        if not self.shape is None:
            offset = vec_math.rotate(com, self.rotation)
            shape_position = vec_math.sub(self.position, offset)
            self.shape.transform(shape_position, self.rotation)

    def draw(self):
        self.shape.draw()