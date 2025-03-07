import math

from pyray import *

import vec_math

from shapes import Shape
from gjk import find_support_point


SEPARATION_BUFFER = 0.01


def draw_polytope(simplex, offset):
    for i in range(len(simplex)):
        start = vec_math.add(simplex[i - 1], offset)
        end = vec_math.add(simplex[i], offset)
        draw_line_v(start, end, WHITE)


def epa(polytope: list[Vector2], shape_a: Shape, shape_b: Shape, debug = False):
    min_index = 0
    min_distance = float('inf')
    min_normal = None
    e = 0
    
    while min_distance == float('inf'):
        for i in range(len(polytope)):
            start = polytope[i]
            end = polytope[(i + 1) % len(polytope)]
            edge_vec = vec_math.sub(end, start)
            normal = vec_math.aligned_tang(edge_vec, end)
            distance = vec_math.dot(normal, start)

            if distance < min_distance:
                min_distance = distance
                min_normal = normal
                min_index = (i + 1) % len(polytope)
                e = edge_vec

        support = find_support_point(shape_a, shape_b, min_normal)
        support_distance = vec_math.dot(min_normal, support)
        separation = abs(support_distance - min_distance)

        #Fixs inverted normal due to small float error
        if abs(min_distance) < SEPARATION_BUFFER:
            neg_normal = vec_math.neg(min_normal)
            neg_support = find_support_point(shape_a, shape_b, neg_normal)
            neg_support_distance = vec_math.dot(neg_normal, neg_support)
            neg_min_distance = vec_math.dot(neg_normal, polytope[min_index])
            neg_separation = abs(neg_support_distance - neg_min_distance)
            min_distance = 0
            if neg_separation < separation:
                separation = neg_separation
                support = neg_support
                min_normal = neg_normal

        if separation > 0.01:
            min_distance = float('inf')
            polytope.insert(min_index, support)

    if debug:
        draw_polytope(polytope, Vector2(get_screen_width() / 2, get_screen_height() / 2))
        draw_circle_v(vec_math.add(vec_math.scale(min_normal, 50), Vector2(get_screen_width() / 2, get_screen_height() / 2)), 5, ORANGE)
    return vec_math.scale(min_normal, min_distance + SEPARATION_BUFFER)