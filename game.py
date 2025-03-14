import global_varibles

from pyray import *
import vec_math, rec_math
import physics
import shapes
import physics_bodys

"""Setup window"""
SCREEN_WIDTH = 960
SCREEN_HEIGHT = 540
init_window(SCREEN_WIDTH, SCREEN_HEIGHT, "Growing Greater")
set_target_fps(60)
#toggle_fullscreen()

#left floor
points = [Vector2(-300, 10), Vector2(300, 10), Vector2(300, -10), Vector2(-300, -10)]
polygon = shapes.Polygon(points)
floor_left = physics_bodys.RigidBody(Vector2(300, 300), rotation=45, shape=polygon, mass=float('inf'), gravity=Vector2(0, 0))

#right floor
points = [Vector2(-300, 10), Vector2(300, 10), Vector2(300, -10), Vector2(-300, -10)]
polygon = shapes.Polygon(points)
floor_right = physics_bodys.RigidBody(Vector2(600, 300), rotation=-45, shape=polygon, mass=float('inf'), gravity=Vector2(0, 0))

#A
points = [Vector2(-80, 40), Vector2(80, 40), Vector2(80, -40), Vector2(-80, -40)]
polygon = shapes.Polygon(points)
rigidbody_a = physics_bodys.RigidBody(Vector2(400, 0), shape=polygon, mass=10, velocity=Vector2(0, 0), gravity=Vector2(0, 98.1))

#B
points = [Vector2(-80, 80), Vector2(80, 80), Vector2(80, -80)]
polygon = shapes.Polygon(points)
rigidbody_b = physics_bodys.RigidBody(Vector2(0, 0), rotation=45, rotational_velocity=0, shape=polygon, gravity=Vector2(0, 0), mass=float('inf'))


while not window_should_close():
    mouse_position = get_mouse_position()
    
    rigidbody_b.position = mouse_position

    global_varibles.PHYSICS_MANAGER.update()

    begin_drawing()
    clear_background(BLACK)
    
    #begin_mode_2d(camera)

    global_varibles.PHYSICS_MANAGER.draw()

    #end_mode_2d()
    
    end_drawing()
close_window()