from pyray import *
import vec_math

a = [Vector2(0, 0) for _ in range(6)]

a[0].x = 7

print(vec_math.as_str(a[0]))
print(vec_math.as_str(a[3]))