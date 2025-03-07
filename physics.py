import global_varibles

from enum import Enum
from itertools import combinations

from pyray import *
import vec_math

from shapes import Shape
from physics_bodys import PhysicsBody
from gjk import gjk
from epa import epa


REST_SPEED = 1


def are_colliding(shape_a: Shape, shape_b: Shape):
    simplex = gjk(shape_a, shape_b)
    return not simplex is None


def get_collision_penetration(shape_a: Shape, shape_b: Shape): #Penetration from shape_a
    polytope = gjk(shape_a, shape_b)
    if polytope is None:
        return None
    return epa(polytope, shape_a, shape_b)
                         

def get_collision_edge(penetration: Vector2, shape_a: Shape, shape_b: Shape, edge_angle: float = 0.2, mass_ratio_a = 0.5, debug = True): #Returns list of (v1, v2) for edge or v for point
    penetration = vec_math.scale(penetration, 1 / 2)
    penetration_normal = vec_math.norm(penetration)

    anticlockwise_normal = vec_math.rotate(penetration_normal, -edge_angle)
    clockwise_normal = vec_math.rotate(penetration_normal, edge_angle)

    anticlockwise_edge_point_a = shape_a.find_furthest_point(anticlockwise_normal)
    clockwise_edge_point_a = shape_a.find_furthest_point(clockwise_normal)

    anticlockwise_edge_point_b = shape_b.find_furthest_point(vec_math.neg(anticlockwise_normal))
    clockwise_edge_point_b = shape_b.find_furthest_point(vec_math.neg(clockwise_normal))

    penetration_tangent = vec_math.tang(penetration)

    anticlockwise_edge_project_a = vec_math.dot(penetration_tangent, anticlockwise_edge_point_a)
    clockwise_edge_project_a = vec_math.dot(penetration_tangent, clockwise_edge_point_a)
    
    anticlockwise_edge_project_b = vec_math.dot(penetration_tangent, anticlockwise_edge_point_b)
    clockwise_edge_project_b = vec_math.dot(penetration_tangent, clockwise_edge_point_b)

    if anticlockwise_edge_project_a <= clockwise_edge_project_b:
        if clockwise_edge_project_a <= anticlockwise_edge_project_b:
            contact_point = vec_math.add(anticlockwise_edge_point_a, anticlockwise_edge_point_b)
            contact_point = vec_math.scale(contact_point, 1 / 2)
            contact_point = vec_math.add(contact_point, vec_math.scale(penetration, (0.5 - mass_ratio_a) * -2))
            overlap = abs(anticlockwise_edge_project_a - anticlockwise_edge_project_b)
        else:
            contact_point = vec_math.add(anticlockwise_edge_point_a, clockwise_edge_point_a)
            contact_point = vec_math.scale(contact_point, 1 / 2)
            contact_point = vec_math.add(contact_point, vec_math.scale(penetration, (1 - mass_ratio_a) * -2))
            overlap = abs(anticlockwise_edge_project_a - clockwise_edge_project_a)
    else:
        if clockwise_edge_project_a >= anticlockwise_edge_project_b:
            contact_point = vec_math.add(clockwise_edge_point_a, clockwise_edge_point_b)
            contact_point = vec_math.scale(contact_point, 1 / 2)
            contact_point = vec_math.add(contact_point, vec_math.scale(penetration, (0.5 - mass_ratio_a) * -2))
            overlap = abs(clockwise_edge_project_a - clockwise_edge_project_b)
        else:
            contact_point = vec_math.add(anticlockwise_edge_point_b, clockwise_edge_point_b)
            contact_point = vec_math.scale(contact_point, 1 / 2)
            contact_point = vec_math.add(contact_point, vec_math.scale(penetration, mass_ratio_a * 2))
            overlap = abs(anticlockwise_edge_project_b - clockwise_edge_project_b)

    if debug:
        draw_circle_v(anticlockwise_edge_point_a, 3, YELLOW)
        draw_circle_v(clockwise_edge_point_a, 3, YELLOW)
        draw_circle_v(anticlockwise_edge_point_b, 3, PINK)
        draw_circle_v(clockwise_edge_point_b, 3, PINK)
        draw_circle_v(contact_point, 5, RED)

    return contact_point, overlap


class PhysicsManager:
    def __init__(self):
        self.max_id = 0
        self.physics_bodys = []
        self.contact_points = {} #add move toward, remove when move away - (3, 7): (Vec)

    def add_physics_body(self, physics_body: PhysicsBody):
        self.physics_bodys.append(physics_body)

    def resolve_collision(self, physics_body_a: PhysicsBody, physics_body_b: PhysicsBody):
        if physics_body_a.shape is None or physics_body_b.shape is None:
            return
        
        penetration = get_collision_penetration(physics_body_a.shape, physics_body_b.shape)
        if penetration is None:
            return

        contact_offset_a = Vector2(0, 0)
        contact_offset_b = Vector2(0, 0)
        if physics_body_a.mass != float('inf') and physics_body_b.mass != float('inf'):
            mass_ratio_a = physics_body_a.mass / (physics_body_a.mass + physics_body_b.mass)
            mass_ratio_b = physics_body_b.mass / (physics_body_a.mass + physics_body_b.mass)
            contact_offset_a = vec_math.scale(vec_math.neg(penetration), mass_ratio_b)
            contact_offset_b = vec_math.scale(penetration, mass_ratio_a)
        elif physics_body_a.mass == float('inf') and physics_body_b.mass != float('inf'):
            mass_ratio_a = 1
            mass_ratio_b = 0
            contact_offset_b = penetration
        elif physics_body_b.mass == float('inf') and physics_body_a.mass != float('inf'):
            mass_ratio_a = 0
            mass_ratio_b = 1
            contact_offset_a = vec_math.neg(penetration)
        else:
            return
        
        contact_point, edge_overlap = get_collision_edge(penetration, physics_body_a.shape, physics_body_b.shape, mass_ratio_a=mass_ratio_a)
        
        physics_body_a.translate(contact_offset_a)
        physics_body_b.translate(contact_offset_b)

        m_a = physics_body_a.mass
        m_b = physics_body_b.mass

        if physics_body_a.shape is None:
            i_a = m_a
        else:
            i_a = physics_body_a.shape.moment_of_inertia * m_a

        if physics_body_b.shape is None:
            i_b = m_b
        else:
            i_b = physics_body_b.shape.moment_of_inertia * m_b

        v_a = physics_body_a.velocity
        v_b = physics_body_b.velocity

        w_a = physics_body_a.rotational_velocity * vec_math.DEG2RAD
        w_b = physics_body_b.rotational_velocity * vec_math.DEG2RAD

        if physics_body_a.center_of_mass is None:
            com_a = physics_body_a.shape.center_of_mass
        else:
            com_a = physics_body_a.center_of_mass

        if physics_body_b.center_of_mass is None:
            com_b = physics_body_b.shape.center_of_mass
        else:
            com_b = physics_body_b.center_of_mass

        n = vec_math.norm(penetration)

        r_a = vec_math.sub(contact_point, vec_math.add(physics_body_a.position, vec_math.rotate(com_a, physics_body_a.rotation)))
        r_b = vec_math.sub(contact_point, vec_math.add(physics_body_b.position, vec_math.rotate(com_b, physics_body_b.rotation)))

        contact_speed_a = vec_math.add(v_a, vec_math.scale(vec_math.tang(r_a), w_a))
        contact_speed_b = vec_math.add(v_b, vec_math.scale(vec_math.tang(r_b), w_b))

        contact_velocity = vec_math.sub(contact_speed_a, contact_speed_b)

        restitution = 0.5 #(shapeA.material.restitution + shapeB.material.restitution) / 2;

        xrn_a = vec_math.cross(r_a, n)
        xrn_b = vec_math.cross(r_b, n)

        j_num = ((-1 - restitution) * (vec_math.dot(v_a, n) - vec_math.dot(v_b, n) + w_a * xrn_a - w_b * xrn_b))
        j_den = ((1 / m_a) + (1 / m_b)) + (1 / i_a) * vec_math.cross(r_a, n) ** 2 + (1 / i_b) * vec_math.cross(r_b, n) ** 2
        j = -j_num / j_den
        
        if j < 0:
            j = 0
        
        force = vec_math.scale(n, j)

        #adjust force based on material force function with priority value
        #Default function should zero force aligning with penetration

        physics_body_a.apply_force_at_point(vec_math.neg(force), vec_math.add(contact_point, contact_offset_a))
        physics_body_b.apply_force_at_point(force, vec_math.add(contact_point, contact_offset_b))

        #Friction
        static_friction = 0.25
        dynamic_friction = 0.129

        t = vec_math.tang(n)

        xrt_a = vec_math.cross(r_a, t)
        xrt_b = vec_math.cross(r_b, t)

        j_t = ((vec_math.dot(v_a, t) - vec_math.dot(v_b, t) + w_a * xrt_a - w_b * xrt_b)) / (1 / m_a + 1 / m_b + xrt_a ** 2 / i_a + xrt_b ** 2 / i_b)

        if abs(j_t) <= abs(j) * static_friction: #Static friction
            friction_force = j_t * restitution
        else:
            friction_force = j * (j_t / abs(j_t)) * dynamic_friction
        friction_force = vec_math.scale(t, friction_force)

        physics_body_a.apply_force_at_point(vec_math.neg(friction_force), vec_math.add(contact_point, contact_offset_a))
        physics_body_b.apply_force_at_point(friction_force, vec_math.add(contact_point, contact_offset_b))

    def resolve_collisions(self): #Add a broadphase
        for physics_body_a, physics_body_b in combinations(self.physics_bodys, 2):
            if physics_body_a is physics_body_b:
                continue
            self.resolve_collision(physics_body_a, physics_body_b)

    def update(self):
        for physics_body in self.physics_bodys:
            physics_body.update()

        self.resolve_collisions()

    def draw(self):
        for physics_body in self.physics_bodys:
            physics_body.draw()


global_varibles.PHYSICS_MANAGER = PhysicsManager()