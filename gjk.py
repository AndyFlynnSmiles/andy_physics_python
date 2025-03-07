from pyray import *

import vec_math
from shapes import Shape


def find_support_point(shape_a: Shape, shape_b: Shape, direction: Vector2):
    furthest_point_a = shape_a.find_furthest_point(direction)
    furthest_point_b = shape_b.find_furthest_point(vec_math.neg(direction))
    support_point = vec_math.sub(furthest_point_a, furthest_point_b)
    return support_point


def next_simplex_from_line(simplex: list[Vector2], direction: Vector2):
    a = simplex[0]
    b = simplex[1]

    a_to_b = vec_math.sub(b, a)
    a_to_origin = vec_math.neg(a)

    if vec_math.same_dir(a_to_b, a_to_origin):
        direction = vec_math.aligned_tang(a_to_b, a_to_origin)
    else:
        simplex = [a]
        direction = a_to_origin
    
    return False, simplex, direction


def next_simplex_from_triangle(simplex: list[Vector2], direction: Vector2):
    a = simplex[0]
    b = simplex[1]
    c = simplex[2]

    a_to_b = vec_math.sub(b, a)
    a_to_c = vec_math.sub(c, a)
    a_to_origin = vec_math.neg(a)
 
    if not vec_math.same_dir(vec_math.aligned_tang(a_to_c, a_to_b), a_to_origin): #Outside on ac edge
        if vec_math.same_dir(a_to_c, a_to_origin):
            simplex = [a, c]
            direction = vec_math.aligned_tang(a_to_c, a_to_origin)
            return False, simplex, direction
        else:
            return next_simplex_from_line([a, b], direction)
    else:
        if not vec_math.same_dir(vec_math.aligned_tang(a_to_b, a_to_c), a_to_origin):
            return next_simplex_from_line([a, b], direction)
        else:
            return True, simplex, direction


def next_simplex(simplex: list[Vector2], direction: Vector2):
    if len(simplex) == 2:
        return next_simplex_from_line(simplex, direction)
    return next_simplex_from_triangle(simplex, direction)


def draw_simplex(simplex, offset):
    for i in range(len(simplex)):
        start = vec_math.add(simplex[i - 1], offset)
        end = vec_math.add(simplex[i], offset)
        draw_line_v(start, end, WHITE)

    point_colors = [RED, GREEN, BLUE]
    for i in range(len(simplex)):
        x = int(simplex[i].x + offset.x)
        y = int(simplex[i].y + offset.y)
        draw_circle(x, y, 5, point_colors[i])

    draw_circle_v(offset, 5, PURPLE)


def gjk(shape_a: Shape, shape_b: Shape, debug = False):
    #Initial setup
    support = find_support_point(shape_a, shape_b, vec_math.right())
    simplex = []
    simplex.insert(0, support)
    direction = vec_math.neg(support)

    #Simplex loop
    while True:
        support = find_support_point(shape_a, shape_b, direction)

        if vec_math.dot(support, direction) <= 0:
            return None
        
        simplex.insert(0, support)

        if debug:
            draw_simplex(simplex, Vector2(get_screen_width() / 2, get_screen_height() / 2))

        colliding, simplex, direction = next_simplex(simplex, direction)

        if colliding:
            return simplex