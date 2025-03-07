from pyray import Vector2, Rectangle

def center(r):
    return Vector2(r.x + r.width / 2, r.y + r.height / 2)

def scale(r, sx, sy, origin = Vector2(0, 0)):
    return Rectangle(r.x, r.y, r.width * sx, r.height * sy)

def from_tex(tex):
    return Rectangle(0, 0, tex.width, tex.height)

def from_vecs(vp, vs):
    return Rectangle(vp.x, vp.y, vs.x, vs.y)

def pos(r):
    return Vector2(r.x, r.y)

def shift(r, v):
    return Rectangle(r.x + v.x, r.y + v.y, r.width, r.height)

def size(r):
    return Vector2(r.width, r.height)

def conv(r):
    return Rectangle(r.x, r.y, r.width, r.height)