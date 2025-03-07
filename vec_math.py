import math
from pyray import Vector2

PI = 3.14159265358979323846
DEG2RAD = PI / 180
RAD2DEG = 180 / PI

def right():
    return Vector2(1, 0)

def copy(v):
    return Vector2(v.x, v.y)

def add(v1, v2):
    return Vector2(v1.x + v2.x, v1.y + v2.y)

def add_on(v1, v2):
    v1.x += v2.x
    v1.y += v2.y

def add_val(v, val):
    return Vector2(v.x + val, v.y + val)

def sub(v1, v2):
    return Vector2(v1.x - v2.x, v1.y - v2.y)

def sub_val(v, val):
    return Vector2(v.x - val, v.y - val)

def length_sqr(v):
    return v.x ** 2 + v.y ** 2

def length(v):
    l = length_sqr(v)
    if l != 0:
        return l ** 0.5
    return 0

def dot(v1, v2):
    return v1.x * v2.x + v1.y * v2.y

def cross(v1, v2):
    return v1.x * v2.y - v1.y * v2.x

def dis_sqr(v1, v2):
    return (v1.x - v2.x) ** 2 + (v1.y - v2.y) ** 2

def dis(v1, v2):
    return dis_sqr(v1, v2) ** 0.5

def ang_btw(v1, v2):
    ang = (math.atan2(v1.y, v1.x) - math.atan2(v2.y, v2.x)) * RAD2DEG
    return ang + 360 * (ang < 0)

def same_dir(v1, v2):
    dp = dot(v1, v2)
    return dp >= -0.0001

def scale(v, scale):
    return Vector2(v.x * scale, v.y * scale)

def mult(v1, v2):
    return Vector2(v1.x * v2.x, v1.y * v2.y)

def neg(v):
    return Vector2(-v.x, -v.y)

def div(v1, v2):
    return Vector2(v1.x / v2.x, v1.y / v2.y)

def norm(v):
    l = length(v)
    if l != 0:
        return Vector2(v.x / l, v.y / l)
    return Vector2(0, 0)

def tang(v):
    n = norm(v)
    return Vector2(n.y, -n.x)

def aligned_tang(v, d):
    t = tang(v)
    if not same_dir(t, d):
        t = neg(t)
    return t

def lerp(v1, v2, amount):
    return add(v1, scale(sub(v2, v1), amount))

def reflect(v, n):
    return sub(v, scale(n, 2 * dot(v, n)))

def rotate(v, ang):
    ang *= DEG2RAD
    ang_cos = math.cos(ang)
    ang_sin = math.sin(ang)
    return Vector2(v.x * ang_cos - v.y * ang_sin, 
                   v.x * ang_sin + v.y * ang_cos)

def rotate_around(v1, v2, ang):
    return add(rotate(sub(v1, v2), ang), v2)

def invert(v):
    return Vector2(1 / v.x, 1 / v.y)

def clamp(v, v_min, v_max):
    return Vector2(max(v_min.x, min(v.x, v_max.x)), 
                   max(v_min.y, min(v.y, v_max.y)))

def clamp_val(v, min_val, max_val):
    l = length(v)
    if l != 0:
        return scale(v, max(min_val, min(l, max_val)) / l)
    return Vector2(0, 0)

def equals(v1, v2):
    return math.isclose(v1.x, v2.x) and math.isclose(v1.y, v2.y)

def as_str(v):
    return f"({v.x}, {v.y})"

def center(vl: list[Vector2]):
    s = vl[0]
    for i in range(1, len(vl)):
        s = add(s, vl[i])
    return scale(s, 1 / len(vl))